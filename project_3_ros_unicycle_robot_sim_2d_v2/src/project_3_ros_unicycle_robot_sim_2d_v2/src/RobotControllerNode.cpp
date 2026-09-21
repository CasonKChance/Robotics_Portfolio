#include "project_3_ros_unicycle_robot_sim_2d_v2/RobotControllerNode.h"

#include <numbers>
#include <cmath>

using GoToPose = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

/* Public Member Functions */

RobotControllerNode::RobotControllerNode(const rclcpp::NodeOptions & options)
: Node("robot_controller", options)
{
  // Set up action server
  using namespace std::placeholders;

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
}

/* Private Member Functions */

rclcpp_action::GoalResponse RobotControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GoToPose::Goal> goal)
{
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

  // Stop robot movement when action is canceled
  publishCommandVelocity(0.0, 0.0);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControllerNode::handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle)
{
  // Spawn detached thread for asynchronous control loop execution
  auto executeInThread = [this, goalHandle](){return this->execute(goalHandle);};
  std::thread{executeInThread}.detach();
}

void RobotControllerNode::execute(const std::shared_ptr<GoalHandleGoToPose> goalHandle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goalHandle->get_goal();
  const Pose goalPose = Pose{goal->x, goal->y, normalizeAngle(goal->theta)};
  auto feedback = std::make_shared<GoToPose::Feedback>();
  auto result = std::make_shared<GoToPose::Result>();
  rclcpp::Rate loopRate(50.0);

  double distanceRemaining = getDistanceRemaining(goalPose);
  double rotationRemaining = getRotationRemaining(goalPose);

  // Rotate towards goal position
  double rotationToGoalPositionRemaining = getRotationToGoalPositionRemaining(goalPose);
  double distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
    currentRobotState_.angularAcceleration);
  while (rclcpp::ok() && std::abs(rotationToGoalPositionRemaining) > distanceToStop) {
    if (goalHandle->is_canceling()) {
      result->x = currentRobotState_.x;
      result->y = currentRobotState_.y;
      result->theta = currentRobotState_.theta;

      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    const double direction = rotationToGoalPositionRemaining >= 0.0 ? 1.0 : -1.0;
    publishCommandVelocity(0.0, currentRobotState_.maximumAngularVelocity * direction);

    rotationToGoalPositionRemaining = getRotationToGoalPositionRemaining(goalPose);
    distanceToStop = getStoppingDistance(currentRobotState_.linearVelocity,
      currentRobotState_.linearAcceleration);

    rotationRemaining = getRotationRemaining(goalPose);
    distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
      currentRobotState_.angularAcceleration);

    feedback->distance_remaining = distanceRemaining;
    feedback->rotation_remaining = rotationRemaining;
    goalHandle->publish_feedback(feedback);

    loopRate.sleep();
  }

  publishCommandVelocity(0.0, 0.0);

  // Wait for robot to stop
  while (rclcpp::ok() && std::abs(currentRobotState_.angularVelocity) > 0) {
    loopRate.sleep();
  }

  // Translate to goal pose
  distanceRemaining = getDistanceRemaining(goalPose);
  distanceToStop = getStoppingDistance(currentRobotState_.linearVelocity,
  currentRobotState_.linearAcceleration);
  while (rclcpp::ok() && distanceRemaining > distanceToStop) {
    if (goalHandle->is_canceling()) {
      result->x = currentRobotState_.x;
      result->y = currentRobotState_.y;
      result->theta = currentRobotState_.theta;

      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    publishCommandVelocity(currentRobotState_.maximumLinearVelocity, 0.0);

    distanceRemaining = getDistanceRemaining(goalPose);
    distanceToStop = getStoppingDistance(currentRobotState_.linearVelocity,
      currentRobotState_.linearAcceleration);

    feedback->distance_remaining = distanceRemaining;
    feedback->rotation_remaining = rotationRemaining;
    goalHandle->publish_feedback(feedback);

    loopRate.sleep();
  }

  publishCommandVelocity(0.0, 0.0);

  // Wait for robot to stop
  while (rclcpp::ok() && std::abs(currentRobotState_.linearVelocity) > 0) {
    loopRate.sleep();
  }

  // Rotate to goal pose
  rotationRemaining = getRotationRemaining(goalPose);
  distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
    currentRobotState_.angularAcceleration);
  while (rclcpp::ok() && std::abs(rotationRemaining) > distanceToStop) {
    if (goalHandle->is_canceling()) {
      result->x = currentRobotState_.x;
      result->y = currentRobotState_.y;
      result->theta = currentRobotState_.theta;

      goalHandle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    const double direction = rotationRemaining >= 0.0 ? 1.0 : -1.0;
    publishCommandVelocity(0.0, currentRobotState_.maximumAngularVelocity * direction);

    rotationRemaining = getRotationRemaining(goalPose);
    distanceToStop = getStoppingDistance(currentRobotState_.angularVelocity,
      currentRobotState_.angularAcceleration);

    feedback->distance_remaining = distanceRemaining;
    feedback->rotation_remaining = rotationRemaining;
    goalHandle->publish_feedback(feedback);

    loopRate.sleep();
  }

  publishCommandVelocity(0.0, 0.0);

  if (rclcpp::ok()) {
    result->x = currentRobotState_.x;
    result->y = currentRobotState_.y;
    result->theta = currentRobotState_.theta;
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
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
