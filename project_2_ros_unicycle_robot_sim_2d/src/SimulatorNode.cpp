#include "project_2_ros_unicycle_robot_sim_2d/SimulatorNode.h"

#include <iostream>

using namespace std::chrono_literals;

/* Public Member Functions */

SimulatorNode::SimulatorNode(const rclcpp::NodeOptions& options)
: Node("simulator_node", options),
  robot_{Pose{0.0, 0.0, 0.0}},
  world_{10, 10, {}, Obstacle{8.0, 8.0, 1.0}},
  status_{ SimulationStatus::Running }
{
    // Subscribe to command velocity topic
    commandVelocitySubscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SimulatorNode::topicCallback, this, std::placeholders::_1));

    // 100 Hz simulation timer loop (dt = 0.01 seconds)
    timer_ = this->create_wall_timer(10ms, std::bind(&SimulatorNode::updateLoop, this));

    // Evaluate initial spatial condition
    status_ = checkCollision();
}

/* Private Member Functions */

void SimulatorNode::topicCallback(geometry_msgs::msg::Twist::UniquePtr message)
{
    RCLCPP_INFO(this->get_logger(), "\nSubscribing: \n"
                                    "\tLinear: %.2f m/s\n"
                                    "\tAngular: %.2f rad/s\n", message->linear.x, message->angular.z);

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
    const Pose& pose = robot_.getPose();

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