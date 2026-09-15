#include "project_2_ros_unicycle_robot_sim_2d/SimulatorNode.h"

#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using project_2_ros_unicycle_robot_sim_2d::srv::SendWorldData;

static const double kDefaultWorldMaxX = 10.0;
static const double kDefaultWorldMaxY = 10.0;
static const double kUpdateRobotTimestep = 0.01;

/* Public Member Functions */

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions & options)
: Node("simulator_node", options),
  robot_(Pose{1.0, 1.0, 0.0}),
  world_(0.0, 0.0),
  status_{SimulationStatus::Running}
{
  buildWorld();

  commandVelocitySubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&SimulatorNode::topicCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    10ms, std::bind(&SimulatorNode::updateLoop, this));

  robotPosePublisher_ =
    this->create_publisher<project_2_ros_unicycle_robot_sim_2d::msg::RobotPose>("robot_pose", 10);

  status_ = checkCollision();
}

void SimulatorNode::handleWorldDataService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<SendWorldData::Request> request,
  const std::shared_ptr<SendWorldData::Response> response) const
{
  (void)request_header;
  (void)request;

  RCLCPP_INFO(this->get_logger(), "Sending world data to visualization node.\n");

  response->max_x = world_.getMaxX();
  response->max_y = world_.getMaxY();

  std::optional<Goal> goal = world_.getGoal();
  if (goal.has_value()) {
    project_2_ros_unicycle_robot_sim_2d::msg::Goal messageGoal;

    messageGoal.center.x = goal->x;
    messageGoal.center.y = goal->y;
    messageGoal.radius = goal->radius;

    response->goal.push_back(messageGoal);
  }

  for (const auto & obstacle : world_.getObstacles()) {
    project_2_ros_unicycle_robot_sim_2d::msg::Obstacle messageObstacle;

    messageObstacle.center.x = obstacle.x;
    messageObstacle.center.y = obstacle.y;
    messageObstacle.radius = obstacle.radius;

    response->obstacles.push_back(messageObstacle);
  }

  response->robot_pose.x = robot_.getPose().x;
  response->robot_pose.y = robot_.getPose().y;
  response->robot_pose.theta = robot_.getPose().theta;
}

/* Private Member Functions */

void SimulatorNode::topicCallback(geometry_msgs::msg::Twist::UniquePtr message)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Listening for velocity command updates...\n");

  if (message->linear.x == robot_.getVelocityCommand().linearVelocity &&
    message->angular.z == robot_.getVelocityCommand().angularVelocity)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Updating velocity command with: \n"
                                    "\tLinear: %.2f m/s\n"
                                    "\tAngular: %.2f rad/s\n", message->linear.x,
                                                               message->angular.z);

  // Update target velocity command on robot model
  robot_.setVelocityCommand({
    .linearVelocity = message->linear.x,
    .angularVelocity = message->angular.z
  });
}

void SimulatorNode::publishRobotPose() const
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Publishing Robot Pose...\n");

  auto message = project_2_ros_unicycle_robot_sim_2d::msg::RobotPose();
  auto robotPose = robot_.getPose();

  message.x = robotPose.x;
  message.y = robotPose.y;
  message.theta = robotPose.theta;

  this->robotPosePublisher_->publish(message);
}

void SimulatorNode::updateLoop()
{
  if (status_ != SimulationStatus::Running) {
    return;
  }

  robot_.update(kUpdateRobotTimestep);

  // Send new pose to visualization node
  publishRobotPose();

  status_ = checkCollision();

  if (status_ != SimulationStatus::Running) {
    RCLCPP_INFO(this->get_logger(), "Simulation ending condition met.\n");

    timer_->cancel();
  }
}

SimulationStatus SimulatorNode::checkCollision() const
{
  const Pose & pose = robot_.getPose();

  if (!world_.isWithinBounds(pose.x, pose.y)) {
    return SimulationStatus::OutOfBounds;
  }

  if (world_.isCollisionWithObstacle(pose.x, pose.y)) {
    return SimulationStatus::ObstacleCollision;
  }

  if (world_.isCollisionWithGoal(pose.x, pose.y)) {
    return SimulationStatus::GoalReached;
  }

  return SimulationStatus::Running;
}

void SimulatorNode::buildWorld()
{
  // Bounds
  const double maxX = this->declare_parameter<double>("world.bounds.max_x", kDefaultWorldMaxX);
  const double maxY = this->declare_parameter<double>("world.bounds.max_y", kDefaultWorldMaxY);

  // Goal
  const bool isGoalEnabled = this->declare_parameter<bool>("world.goal.enabled", false);

  std::optional<Goal> goal = std::nullopt;
  if (isGoalEnabled) {
    const double goalX = this->declare_parameter<double>("world.goal.x", 0.0);
    const double goalY = this->declare_parameter<double>("world.goal.y", 0.0);
    const double goalRadius = this->declare_parameter<double>("world.goal.radius", 0.0);

    goal = Goal{goalX, goalY, goalRadius};
  }

  // Obstacles
  const auto obstacleXs = this->declare_parameter<std::vector<double>>("world.obstacles.x",
    std::vector<double>{});
  const auto obstacleYs = this->declare_parameter<std::vector<double>>("world.obstacles.y",
    std::vector<double>{});
  const auto obstacleRadii = this->declare_parameter<std::vector<double>>("world.obstacles.radius",
    std::vector<double>{});

  if (obstacleXs.size() != obstacleYs.size() ||
    obstacleXs.size() != obstacleRadii.size())
  {
    throw std::runtime_error(
      "world.obstacles.x, world.obstacles.y, and "
      "world.obstacles.radius must have the same length.");
  }

  std::vector<Obstacle> obstacles;
  obstacles.reserve(obstacleXs.size());

  for (std::size_t i = 0; i < obstacleXs.size(); ++i) {
    obstacles.push_back(
      Obstacle{
      obstacleXs[i],
      obstacleYs[i],
      obstacleRadii[i]
      });
  }

  world_ = World(maxX, maxY, obstacles, goal);
}
