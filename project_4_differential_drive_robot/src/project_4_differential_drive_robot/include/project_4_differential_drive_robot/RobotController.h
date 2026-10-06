#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <SDL2/SDL.h>

/**
 * @brief Provides a teleop controller to control the robot model using keyboard input.
 */
class RobotController: public rclcpp::Node
{
public:

  /**
   * @brief Constructs the controller and sets up an SDL window for keyboard input.
   * @param options Configuration options for Node initialization
   */
  explicit RobotController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the controller and releases SDL resources.
   */
  ~RobotController();

private:
  SDL_Window * window_ {nullptr}; // SDL window for keyboard input, must be focused.
  bool upPressed_ {false};
  bool downPressed_ {false};
  bool leftPressed_ {false};
  bool rightPressed_ {false};

  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr commandVelocityPublisher_;
  rclcpp::TimerBase::SharedPtr keyboardTimer_;

  /**
   * @brief Processes keyboard input and updates the state of the control buttons.
   */
  void processKeyboardInput();

  /**
   * @brief Publishes command velocities based on the current state of the control buttons.
   */
  void publishCommandVelocity() const;
};

#endif
