#include "project_2_ros_unicycle_robot_sim_2d/CommandVelocityPublisher.h"

using namespace std::chrono_literals;

/* Public Member Functions */

CommandVelocityPublisher::CommandVelocityPublisher(const rclcpp::NodeOptions & options)
: Node("command_velocity_publisher", options)
{
  // Initialize publisher for velocity commands
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Declare tunable ROS 2 parameters
  this->declare_parameter<double>("linear_velocity", 0.0);
  this->declare_parameter<double>("angular_velocity", 0.0);

  // Set up 100 Hz wall timer (10 ms period)
  timer_ = this->create_wall_timer(10ms,
    std::bind(&CommandVelocityPublisher::timerCallback, this));
}

/* Private Member Functions */

void CommandVelocityPublisher::timerCallback()
{
  auto message = geometry_msgs::msg::Twist();

  message.linear.x = this->get_parameter("linear_velocity").as_double();
  message.angular.z = normalizeAngle(this->get_parameter("angular_velocity").as_double());

  RCLCPP_INFO(this->get_logger(), "\nPublishing: \n"
                                    "\tLinear: %.2f m/s\n"
                                    "\tAngular: %.2f rad/s\n", message.linear.x, message.angular.z);

  publisher_->publish(message);
}

double CommandVelocityPublisher::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}
