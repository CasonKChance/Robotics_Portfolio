#include "project_2_ros_unicycle_robot_sim_2d/CommandVelocityPublisherNode.h"

using namespace std::chrono_literals;

/* Public Member Functions */

CommandVelocityPublisherNode::CommandVelocityPublisherNode(const rclcpp::NodeOptions & options)
: Node("command_velocity_publisher_node", options)
{
  // Initialize publisher for velocity commands
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Declare tunable ROS 2 parameters
  this->declare_parameter<double>("linear_velocity", 0.0);
  this->declare_parameter<double>("angular_velocity", 0.0);

  // Set up 100 Hz wall timer (10 ms period)
  timer_ = this->create_wall_timer(10ms,
    std::bind(&CommandVelocityPublisherNode::timerCallback, this));
}

/* Private Member Functions */

void CommandVelocityPublisherNode::timerCallback()
{
  auto message = geometry_msgs::msg::Twist();

  message.linear.x = this->get_parameter("linear_velocity").as_double();
  message.angular.z = normalizeAngle(this->get_parameter("angular_velocity").as_double());

  RCLCPP_INFO_ONCE(this->get_logger(), "\nPublishing...\n");

  publisher_->publish(message);
}

double CommandVelocityPublisherNode::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}
