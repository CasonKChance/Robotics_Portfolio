#ifndef COMMAND_VELOCITY_PUBLISHER_H
#define COMMAND_VELOCITY_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

/**
 * @brief ROS 2 Node that periodically reads velocity parameters and publishes Twist messages.
 *
 * This node manages user-configurable linear and angular velocity parameters and broadcasts
 * them to the "cmd_vel" topic on a fixed timer schedule.
 */
class CommandVelocityPublisher: public rclcpp::Node {
public:
  /**
   * @brief Constructs a new Command Velocity Publisher node.
   * @param options Configuration options for Node initialization and parameter overrides.
   */
  explicit CommandVelocityPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr publisher_;

  /**
   * @brief Periodic timer callback that fetches current parameters, normalizes angular velocity,
   *        and publishes a Twist message to "cmd_vel".
   */
  void timerCallback();

  /**
   * @brief Wraps an angle in radians to the range [-π, π].
   * @param angle Raw input angle in radians.
   * @return Normalized angle bounded within [-π, π].
   */
  double normalizeAngle(double angle) const;
};

#endif
