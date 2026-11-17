#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

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
  SDL_Renderer * renderer_ = nullptr;
  TTF_Font * font_ = nullptr;
  SDL_Texture * text_texture_ = nullptr;
  SDL_Rect text_rect_;
  bool up_pressed_ {false};
  bool down_pressed_ {false};
  bool left_pressed_ {false};
  bool right_pressed_ {false};

  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr command_velocity_publisher_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;

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
