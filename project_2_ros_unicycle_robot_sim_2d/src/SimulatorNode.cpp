#include "project_2_ros_unicycle_robot_sim_2d/SimulatorNode.h"

#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

/* Public Member Functions */

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions & options)
: Node("simulator_node", options),
  robot_(Pose{1.0, 1.0, 0.0}),
  world_(10.0, 10.0),
  status_{SimulationStatus::Running}
{
  buildWorld();

  commandVelocitySubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&SimulatorNode::topicCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    10ms, std::bind(&SimulatorNode::updateLoop, this));

  status_ = checkCollision();
}

/* Private Member Functions */

void SimulatorNode::topicCallback(geometry_msgs::msg::Twist::UniquePtr message)
{
  if (message->linear.x == robot_.getVelocityCommand().linearVelocity &&
    message->angular.z == robot_.getVelocityCommand().angularVelocity)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\nUpdating velocity command with: \n"
                                    "\tLinear: %.2f m/s\n"
                                    "\tAngular: %.2f rad/s\n", message->linear.x,
                                                               message->angular.z);

  // Update target velocity command on robot model
  robot_.setVelocityCommand({
    .linearVelocity = message->linear.x,
    .angularVelocity = message->angular.z
  });
}

void SimulatorNode::updateLoop()
{
  if (status_ != SimulationStatus::Running) {
    return;
  }

  constexpr double dt = 0.01; // 10ms step time
  robot_.update(dt);

  status_ = checkCollision();

  if (status_ != SimulationStatus::Running) {
    RCLCPP_INFO(this->get_logger(), "Simulation ending condition met. Shutting down node.");
    rclcpp::shutdown();
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
  const double maxX = this->declare_parameter<double>("world.bounds.length", 10.0);
  const double maxY = this->declare_parameter<double>("world.bounds.width", 10.0);

  // Goal
  const bool isGoalEnabled = this->declare_parameter<bool>("world.goal.enabled", false);

  std::optional<Obstacle> goal = std::nullopt;
  if (isGoalEnabled) {
    const double goalX = this->declare_parameter<double>("world.goal.x", 0.0);
    const double goalY = this->declare_parameter<double>("world.goal.y", 0.0);
    const double goalRadius = this->declare_parameter<double>("world.goal.radius", 0.0);

    goal = Obstacle{goalX, goalY, goalRadius};
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
