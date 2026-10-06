#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <SDL2/SDL.h>

class RobotController: public rclcpp::Node
{
public:
  explicit RobotController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~RobotController();

private:
  SDL_Window * window_ {nullptr};
  bool upPressed_ {false};
  bool downPressed_ {false};
  bool leftPressed_ {false};
  bool rightPressed_ {false};

  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr commandVelocityPublisher_;
  rclcpp::TimerBase::SharedPtr keyboardTimer_;

  void processKeyboardInput();
  void publishCommandVelocity() const;
};

#endif
