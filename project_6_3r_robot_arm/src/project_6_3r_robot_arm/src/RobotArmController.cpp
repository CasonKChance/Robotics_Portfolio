#include "project_6_3r_robot_arm/RobotArmController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace std::placeholders;

static constexpr double kDefaultGoalPositionTolerance = 5e-2;  // meters
static constexpr double kJointAngleTolerance = 5e-2;           // rad
static constexpr double kStoppedVelocityTolerance = 1e-3;      // rad/s
static constexpr int kIKIterations = 150;

/* Public Member Functions */

RobotArmController::RobotArmController(const rclcpp::NodeOptions & options)
: Node("robot_arm_controller", options), controller_state_(ControllerState::Idle)
{
  goal_position_tolerance_ = declare_parameter<double>("goal_position_tolerance", kDefaultGoalPositionTolerance);
  if (!std::isfinite(goal_position_tolerance_) || goal_position_tolerance_ <= 0.0) {
    throw std::invalid_argument("goal_position_tolerance must be positive and finite");
  }

  robot_state_subscription_ = create_subscription<project_6_3r_robot_arm_interfaces::msg::RobotState>(
    "robot_state", 10, std::bind(&RobotArmController::robotStateTopicCallback, this, _1));

  joint_velocities_publisher_ = create_publisher<JointVelocities>("joint_velocities", 10);

  go_to_position_action_server_ = rclcpp_action::create_server<GoToPosition>(
    this, "go_to_position",
    std::bind(&RobotArmController::handleGoal, this, _1, _2),
    std::bind(&RobotArmController::handleCancel, this, _1),
    std::bind(&RobotArmController::handleAccepted, this, _1));

  control_timer_ = create_wall_timer(10ms, std::bind(&RobotArmController::controlLoop, this));
}

/* Private Member Functions */

rclcpp_action::GoalResponse RobotArmController::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoToPosition::Goal> goal)
{
  (void)uuid;
  if (active_goal_handle_ || controller_state_ != ControllerState::Idle) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: Another goal is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal || !std::isfinite(goal->x) || !std::isfinite(goal->y) || !std::isfinite(goal->z)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: Target position must be finite.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!validRobotState()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: Robot state is invalid.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepted goal position: (%.3f, %.3f, %.3f)", goal->x, goal->y, goal->z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotArmController::handleCancel(
  const std::shared_ptr<GoalHandleGoToPosition> goalHandle)
{
  if (!active_goal_handle_ || goalHandle != active_goal_handle_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal cancelation: No active goal.");
    return rclcpp_action::CancelResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Cancellation requested.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotArmController::handleAccepted(
  const std::shared_ptr<GoalHandleGoToPosition> goalHandle)
{
  active_goal_handle_ = goalHandle;
  controller_state_ = ControllerState::RotatingToGoalPosition;
}

void RobotArmController::controlLoop()
{
  const auto handle = active_goal_handle_;
  if (!handle) {
    return;
  }

  const auto goal = handle->get_goal();
  const Eigen::Vector3d target(goal->x, goal->y, goal->z);
  const auto result = std::make_shared<GoToPosition::Result>();
  const auto fillResult = [&]() {
      result->x = current_robot_state_.end_effector_x;
      result->y = current_robot_state_.end_effector_y;
      result->z = current_robot_state_.end_effector_z;
    };
  const auto stopped = [&]() {
      const auto velocities = jointVelocities();
      return std::all_of(velocities.begin(), velocities.end(),
        [](double v) {return std::abs(v) <= kStoppedVelocityTolerance;});
    };
  const auto failGoal = [&](const char * reason) {
      RCLCPP_ERROR(get_logger(), "%s", reason);
      publishCommandJointVelocities({0.0, 0.0, 0.0});
      fillResult();
      handle->abort(result);
      active_goal_handle_.reset();
      controller_state_ = ControllerState::Idle;
      goal_angles_ = std::vector<double>{};
    };

  if (handle->is_canceling()) {
    controller_state_ = ControllerState::GoalCanceled;
  }
  if (controller_state_ == ControllerState::GoalCanceled) {
    publishCommandJointVelocities({0.0, 0.0, 0.0});

    if (stopped()) {
      fillResult();
      handle->canceled(result);
      active_goal_handle_.reset();
      controller_state_ = ControllerState::Idle;
      goal_angles_ = std::vector<double>{};
      RCLCPP_INFO(get_logger(), "Goal canceled after joint motion stopped.");
    }

    return;
  }

  if (!validRobotState()) {
    failGoal("Robot state became invalid during goal execution.");

    return;
  }

  if (goal_angles_.empty()) {
    try {
      goal_angles_ = solveGoalAngles(target);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "IK error: %s", error.what());
    }
    if (goal_angles_.size() != 3) {
      failGoal("No inverse-kinematics solution for target position.");
      return;
    }
  }

  const auto remaining = getRotationRemaining(goal_angles_);
  const auto feedback = std::make_shared<GoToPosition::Feedback>();
  feedback->joint_pedestal_arm1_rotation_remaining = remaining[0];
  feedback->joint_arm1_arm2_rotation_remaining = remaining[1];
  feedback->joint_arm2_arm3_rotation_remaining = remaining[2];
  handle->publish_feedback(feedback);

  const double position_error = (target - measuredPosition()).norm();

  switch (controller_state_) {
    case ControllerState::Idle:
      return;

    case ControllerState::RotatingToGoalPosition: {
        const auto velocities = jointVelocities();
        const auto max_velocities = maximumJointVelocities();
        const auto max_accelerations = maximumJointAccelerations();
        std::vector<double> commands{0.0, 0.0, 0.0};

        bool all_braking = true;
        for (std::size_t i = 0; i < 3; ++i) {
          const double stopping_distance = getStoppingDistance(velocities[i], max_accelerations[i]);

          if (std::abs(remaining[i]) > kJointAngleTolerance && std::abs(remaining[i]) > stopping_distance) {
            commands[i] = std::copysign(max_velocities[i], remaining[i]);
            all_braking = false;
          }
        }

        publishCommandJointVelocities(commands);
        if (all_braking) {
          controller_state_ = ControllerState::WaitingForRotationToGoalPositionStop;
        }
        return;
    }

    case ControllerState::WaitingForRotationToGoalPositionStop:
      publishCommandJointVelocities({0.0, 0.0, 0.0});
      if (stopped()) {
        controller_state_ = ControllerState::GoalPositionReached;
      }
      
      return;

    case ControllerState::GoalPositionReached:
      publishCommandJointVelocities({0.0, 0.0, 0.0});
      if (position_error > goal_position_tolerance_) {
        // Re-run IK from the measured configuration to correct residual error.
        controller_state_ = ControllerState::RotatingToGoalPosition;
        goal_angles_ = std::vector<double>{};
        return;
      }
      fillResult();
      handle->succeed(result);
      active_goal_handle_.reset();
      controller_state_ = ControllerState::Idle;
      goal_angles_ = std::vector<double>{};
      RCLCPP_INFO(get_logger(), "Goal reached.");
      return;

    case ControllerState::GoalCanceled:
      return;  // handled above
  }
}

void RobotArmController::publishCommandJointVelocities(const std::vector<double> & jointVelocities)
{
  if (jointVelocities.size() != 3) {
    throw std::invalid_argument("Three joint velocity commands are required.");
  }

  JointVelocities message;
  message.joint_pedestal_arm1_velocity = jointVelocities[0];
  message.joint_arm1_arm2_velocity = jointVelocities[1];
  message.joint_arm2_arm3_velocity = jointVelocities[2];

  joint_velocities_publisher_->publish(message);
}

void RobotArmController::robotStateTopicCallback(project_6_3r_robot_arm_interfaces::msg::RobotState::UniquePtr message)
{
  current_robot_state_.end_effector_x = message->end_effector_x;
  current_robot_state_.end_effector_y = message->end_effector_y;
  current_robot_state_.end_effector_z = message->end_effector_z;
  current_robot_state_.arm1_length = message->arm1_length;
  current_robot_state_.arm2_length = message->arm2_length;
  current_robot_state_.arm3_length = message->arm3_length;
  current_robot_state_.joint_pedestal_arm1_angle = message->joint_pedestal_arm1_angle;
  current_robot_state_.joint_arm1_arm2_angle = message->joint_arm1_arm2_angle;
  current_robot_state_.joint_arm2_arm3_angle = message->joint_arm2_arm3_angle;
  current_robot_state_.joint_pedestal_arm1_velocity = message->joint_pedestal_arm1_velocity;
  current_robot_state_.joint_arm1_arm2_velocity = message->joint_arm1_arm2_velocity;
  current_robot_state_.joint_arm2_arm3_velocity = message->joint_arm2_arm3_velocity;
  current_robot_state_.joint_pedestal_arm1_maximum_acceleration = message->joint_pedestal_arm1_maximum_acceleration;
  current_robot_state_.joint_arm1_arm2_maximum_acceleration = message->joint_arm1_arm2_maximum_acceleration;
  current_robot_state_.joint_arm2_arm3_maximum_acceleration = message->joint_arm2_arm3_maximum_acceleration;
  current_robot_state_.joint_pedestal_arm1_maximum_velocity = message->joint_pedestal_arm1_maximum_velocity;
  current_robot_state_.joint_arm1_arm2_maximum_velocity = message->joint_arm1_arm2_maximum_velocity;
  current_robot_state_.joint_arm2_arm3_maximum_velocity = message->joint_arm2_arm3_maximum_velocity;
  current_robot_state_.x_offset = message->x_offset;
  current_robot_state_.y_offset = message->y_offset;
  current_robot_state_.z_offset = message->z_offset;
}

std::vector<double> RobotArmController::getRotationRemaining(const std::vector<double> & goalAngles)
{
  if (goalAngles.size() != 3) {
    throw std::invalid_argument("Three target joint angles are required.");
  }

  const auto current = jointAngles();
  return {normalizeAngle(goalAngles[0] - current[0]), normalizeAngle(goalAngles[1] - current[1]), normalizeAngle(goalAngles[2] - current[2])};
}

double RobotArmController::getStoppingDistance(
  const double velocity, const double acceleration) const
{
  if (!std::isfinite(velocity) || !std::isfinite(acceleration) || acceleration <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return velocity * velocity / (2.0 * acceleration);
}

std::vector<double> RobotArmController::solveGoalAngles(const Eigen::Vector3d & goal)
{
  const auto lengths = linkLengths();
  const auto axes = jointAxes();
  const Eigen::Vector3d offset = measuredOffset();
  const Eigen::Vector3d ik_error_tolerance = Eigen::Vector3d::Constant(goal_position_tolerance_ / std::sqrt(3.0));

  std::vector<double> solution = Kinematics::numericalInverseKinematics(goal - offset, lengths, axes, jointAngles(), ik_error_tolerance, kIKIterations);
  if (solution.size() != 3 || !std::all_of(solution.begin(), solution.end(), [](double angle) { return std::isfinite(angle); })) {
    return {};
  }

  for (double & angle : solution) {
    angle = normalizeAngle(angle);
  }

  const Eigen::Vector3d predicted_position = Kinematics::forwardKinematics(solution, lengths, axes) + offset;

  return ((goal - predicted_position).norm() <= goal_position_tolerance_) ? solution : std::vector<double>{};
}

double RobotArmController::normalizeAngle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<Eigen::VectorXd> RobotArmController::jointAxes()
{
  // The pedestal rotates around Z; the two remaining joints rotate around Y.
  return {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY()};
}

std::vector<double> RobotArmController::linkLengths()
{
  return {current_robot_state_.arm1_length, current_robot_state_.arm2_length, current_robot_state_.arm3_length};
}

std::vector<double> RobotArmController::jointAngles()
{
  return {current_robot_state_.joint_pedestal_arm1_angle, current_robot_state_.joint_arm1_arm2_angle, current_robot_state_.joint_arm2_arm3_angle};
}

std::vector<double> RobotArmController::jointVelocities()
{
  return {current_robot_state_.joint_pedestal_arm1_velocity, current_robot_state_.joint_arm1_arm2_velocity, current_robot_state_.joint_arm2_arm3_velocity};
}

std::vector<double> RobotArmController::maximumJointAccelerations()
{
  return {current_robot_state_.joint_pedestal_arm1_maximum_acceleration, current_robot_state_.joint_arm1_arm2_maximum_acceleration, current_robot_state_.joint_arm2_arm3_maximum_acceleration};
}

std::vector<double> RobotArmController::maximumJointVelocities()
{
  return {current_robot_state_.joint_pedestal_arm1_maximum_velocity, current_robot_state_.joint_arm1_arm2_maximum_velocity, current_robot_state_.joint_arm2_arm3_maximum_velocity};
}

Eigen::Vector3d RobotArmController::measuredPosition()
{
  return {current_robot_state_.end_effector_x, current_robot_state_.end_effector_y, current_robot_state_.end_effector_z};
}

Eigen::Vector3d RobotArmController::measuredOffset()
{
  return {current_robot_state_.x_offset, current_robot_state_.y_offset, current_robot_state_.z_offset};
}

bool RobotArmController::validRobotState()
{
  const auto lengths = linkLengths();
  const auto angles = jointAngles();
  const auto velocities = jointVelocities();
  const auto maximum_accelerations = maximumJointAccelerations();
  const auto maximum_velocities = maximumJointVelocities();

  return std::all_of(lengths.begin(), lengths.end(), [](double length) {return std::isfinite(length) && length > 0.0;}) &&
         std::all_of(angles.begin(), angles.end(), [](double angle) {return std::isfinite(angle);}) &&
         std::all_of(velocities.begin(), velocities.end(), [](double velocity) {return std::isfinite(velocity);}) &&
         std::all_of(maximum_accelerations.begin(), maximum_accelerations.end(), [](double maximum_acceleration) {return std::isfinite(maximum_acceleration);}) &&
         std::all_of(maximum_velocities.begin(), maximum_velocities.end(), [](double maximum_velocity) {return std::isfinite(maximum_velocity);}) &&
         measuredPosition().allFinite() &&
         measuredOffset().allFinite();
}
