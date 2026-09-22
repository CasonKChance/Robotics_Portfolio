#include "project_3_ros_unicycle_robot_sim_2d_v2/RobotControllerNode.h"

#include <numbers>
#include <cmath>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

using GoToPose = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

/* Public Member Functions */

RobotControllerNode::RobotControllerNode(const rclcpp::NodeOptions & options)
: Node("robot_controller", options)
{
  // Set up action server
  goToPoseActionServer_ = rclcpp_action::create_server<GoToPose>(
      this,
      "go_to_pose",
      std::bind(&RobotControllerNode::handleGoal, this, _1, _2),
      std::bind(&RobotControllerNode::handleCancel, this, _1),
      std::bind(&RobotControllerNode::handleAccepted, this, _1)
  );

  // Set up command velocity publisher
  commandVelocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Set up robot pose subscription
  robotPoseSubscription_ = this->create_subscription<project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState>(
    "robot_state", 10,
    std::bind(&RobotControllerNode::robotPoseTopicCallback, this, std::placeholders::_1));

  controllerState_ = ControllerState::Idle;

  controlTimer_ = this->create_wall_timer(
      20ms,
      std::bind(&RobotControllerNode::controlLoop, this));
}

/* Private Member Functions */

rclcpp_action::GoalResponse RobotControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoToPose::Goal> goal)
{
  if (activeGoalHandle_) {
    RCLCPP_WARN(this->get_logger(), "Rejecting goal: another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Recieved goal request to go to pose:\n"
                                  "\tx: %.2f\n"
                                  "\ty: %.2f\n"
                                  "\ttheta: %.2f\n", goal->x, goal->y, normalizeAngle(goal->theta));
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handleCancel(
  const std::shared_ptr<GoalHandleGoToPose> goalHandle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal\n");
  (void)goalHandle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControllerNode::handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle)
{
  activeGoalHandle_ = goalHandle;

  controllerState_ = ControllerState::RotatingToGoalPosition;
}

void RobotControllerNode::controlLoop()
{
  const auto goalHandle = activeGoalHandle_;
  if (!goalHandle) {
    return;
  }

  if (goalHandle->is_canceling()) {
    // Command robot to stop
    publishCommandVelocity(0.0, 0.0);

    controllerState_ = ControllerState::GoalCanceled;
    return;
  }

  const auto goal = goalHandle->get_goal();
  const Pose goalPose = Pose{goal->x, goal->y, normalizeAngle(goal->theta)};
  auto feedback = std::make_shared<GoToPose::Feedback>();
  auto result = std::make_shared<GoToPose::Result>();

  switch(controllerState_) {
    case ControllerState::Idle: {
        // Loop until a new goal is accepted
        return;
      }
    case ControllerState::RotatingToGoalPosition: {
        // Rotate to goal position
        double rotationToGoalPositionRemaining = getRotationToGoalPositionRemaining(goalPose);
        double distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
        currentRobotState_.angularAcceleration);
        if (std::abs(rotationToGoalPositionRemaining) > distanceToStop) {
          const double direction = rotationToGoalPositionRemaining >= 0.0 ? 1.0 : -1.0;
          publishCommandVelocity(0.0, currentRobotState_.maximumAngularVelocity * direction);

          feedback->distance_remaining = getDistanceRemaining(goalPose);
          feedback->rotation_remaining = getRotationRemaining(goalPose);
          goalHandle->publish_feedback(feedback);

          return;
        }

        // Command robot to stop
        publishCommandVelocity(0.0, 0.0);
        controllerState_ = ControllerState::WaitingForRotationToGoalPositionStop;
        return;
      }
    case ControllerState::WaitingForRotationToGoalPositionStop: {
        // Make sure robot has completed stop, loop until completed.
        if (std::abs(currentRobotState_.angularVelocity) > 0) {
          return;
        }

        controllerState_ = ControllerState::DrivingToGoalPose;
        return;
      }
    case ControllerState::DrivingToGoalPose: {
        // Translate to goal pose
        double distanceRemaining = getDistanceRemaining(goalPose);
        double distanceToStop = getStoppingDistance(currentRobotState_.linearVelocity,
          currentRobotState_.linearAcceleration);
        if (distanceRemaining > distanceToStop) {
          publishCommandVelocity(currentRobotState_.maximumLinearVelocity, 0.0);

          feedback->distance_remaining = getDistanceRemaining(goalPose);
          feedback->rotation_remaining = getRotationRemaining(goalPose);
          goalHandle->publish_feedback(feedback);

          return;
        }

        // Command robot to stop
        publishCommandVelocity(0.0, 0.0);
        controllerState_ = ControllerState::WaitingForDrivingToGoalPoseStop;
        return;
      }
    case ControllerState::WaitingForDrivingToGoalPoseStop: {
        // Make sure robot has completed stop, loop until completed.
        if (std::abs(currentRobotState_.linearVelocity) > 0) {
          return;
        }

        controllerState_ = ControllerState::RotatingToGoalPose;
        return;
      }
    case ControllerState::RotatingToGoalPose: {
        // Rotate to goal pose
        double rotationRemaining = getRotationRemaining(goalPose);
        double distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
          currentRobotState_.angularAcceleration);
        if (std::abs(rotationRemaining) > distanceToStop) {
          const double direction = rotationRemaining >= 0.0 ? 1.0 : -1.0;
          publishCommandVelocity(0.0, currentRobotState_.maximumAngularVelocity * direction);

          feedback->distance_remaining = getDistanceRemaining(goalPose);
          feedback->rotation_remaining = getRotationRemaining(goalPose);
          goalHandle->publish_feedback(feedback);

          return;
        }

        // Command robot to stop
        publishCommandVelocity(0.0, 0.0);
        controllerState_ = ControllerState::WaitingForRotationToGoalPoseStop;
        return;
      }
    case ControllerState::WaitingForRotationToGoalPoseStop: {
        // Make sure robot has completed stop, loop until completed.
        if (std::abs(currentRobotState_.angularVelocity) > 0) {
          return;
        }

        controllerState_ = ControllerState::GoalPoseReached;
        return;
      }
    case ControllerState::GoalPoseReached: {
        result->x = currentRobotState_.x;
        result->y = currentRobotState_.y;
        result->theta = currentRobotState_.theta;
        goalHandle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded\n");

        controllerState_ = ControllerState::Idle;
        activeGoalHandle_.reset();

        return;
      }
    case ControllerState::GoalCanceled: {
        // Make sure robot has completed stop, loop until completed.
        if (std::abs(currentRobotState_.linearVelocity) > 0 ||
          std::abs(currentRobotState_.angularVelocity) > 0)
        {
          return;
        }

        result->x = currentRobotState_.x;
        result->y = currentRobotState_.y;
        result->theta = currentRobotState_.theta;

        goalHandle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled\n");

        controllerState_ = ControllerState::Idle;
        activeGoalHandle_.reset();

        return;
      }
    default: {
        RCLCPP_INFO(this->get_logger(), "Unknown controller state. Canceling goal.\n");
        publishCommandVelocity(0.0, 0.0);
        controllerState_ = ControllerState::GoalCanceled;
        return;
      }
  }
}

double RobotControllerNode::getDistanceRemaining(const Pose & goalPose) const
{
  return std::hypot(goalPose.x - currentRobotState_.x, goalPose.y - currentRobotState_.y);
}

double RobotControllerNode::getRotationRemaining(const Pose & goalPose) const
{
  return normalizeAngle(goalPose.theta - currentRobotState_.theta);
}

double RobotControllerNode::getRotationToGoalPositionRemaining(const Pose & targetPose) const
{
  return normalizeAngle(std::atan2(targetPose.y - currentRobotState_.y,
    targetPose.x - currentRobotState_.x) - currentRobotState_.theta);
}

double RobotControllerNode::getStoppingDistance(
  const double velocity,
  const double acceleration) const
{
  return std::pow(velocity, 2) / (2 * acceleration);
}

double RobotControllerNode::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}

void RobotControllerNode::publishCommandVelocity(
  const double linearVelocity,
  const double angularVelocity)
{
  auto message = geometry_msgs::msg::Twist();

  message.linear.x = linearVelocity;
  message.angular.z = angularVelocity;

  commandVelocityPublisher_->publish(message);
}

void RobotControllerNode::robotPoseTopicCallback(
  project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState::UniquePtr message)
{
  currentRobotState_.x = message->x;
  currentRobotState_.y = message->y;
  currentRobotState_.theta = message->theta;
  currentRobotState_.linearVelocity = message->linear_velocity;
  currentRobotState_.angularVelocity = message->angular_velocity;
}
