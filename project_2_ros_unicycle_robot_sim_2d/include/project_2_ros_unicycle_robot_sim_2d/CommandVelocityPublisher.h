#ifndef COMMAND_VELOCITY_PUBLISHER_H
#define COMMAND_VELOCITY_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

class CommandVelocityPublisher: public rclcpp::Node {
public:
  explicit CommandVelocityPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr publisher_;

  void timerCallback();

  double normalizeAngle(double angle) const;
};

#endif
