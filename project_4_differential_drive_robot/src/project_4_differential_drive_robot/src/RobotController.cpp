#include <project_4_differential_drive_robot/RobotController.h>

#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

static const double kMaximumLinearSpeed = 1.0;
static const double kMaximumAngularSpeed = 2.0;

/* Public Member Functions */

RobotController::RobotController(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_controller", options)
{
    commandVelocityPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize SDL Window
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        throw std::runtime_error(
            std::string("Failed to initialize SDL: ") +
            SDL_GetError());
    }

    window_ = SDL_CreateWindow(
                "Differential Drive Teleop",
                SDL_WINDOWPOS_CENTERED,
                SDL_WINDOWPOS_CENTERED,
                400,
                150,
                SDL_WINDOW_SHOWN);

    if (!window_) {
        const std::string error = SDL_GetError();

        SDL_Quit();

        throw std::runtime_error(
            "Failed to create SDL window: " + error);
    }

    RCLCPP_INFO(this->get_logger(), "Teleop controller started. Focus the SDL window and use the arrow keys to control robot.");

    // Poll keyboard at 100Hz rate
    keyboardTimer_ = this->create_wall_timer(
        10ms, std::bind(&RobotController::processKeyboardInput, this));
}

RobotController::~RobotController()
{
  // Command robot to stop before shutting down controller
  if (commandVelocityPublisher_) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    commandVelocityPublisher_->publish(msg);
  }

  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }

  SDL_Quit();
}

/* Private Member Functions */

void RobotController::processKeyboardInput()
{
  SDL_Event event;

  // Consume all awaiting events
  while (SDL_PollEvent(&event)) {

    // Handle window close event
    if (event.type == SDL_QUIT) {
      upPressed_ = false;
      downPressed_ = false;
      leftPressed_ = false;
      rightPressed_ = false;

      publishCommandVelocity();

      RCLCPP_INFO(
        this->get_logger(),
        "Teleop window closed.");

      return;
    }

    // Handle window looses focus
    if (
      event.type == SDL_WINDOWEVENT &&
      event.window.event == SDL_WINDOWEVENT_FOCUS_LOST)
    {
      upPressed_ = false;
      downPressed_ = false;
      leftPressed_ = false;
      rightPressed_ = false;

      publishCommandVelocity();

      continue;
    }

    // Ignore all non-keyboard events
    if (
      event.type != SDL_KEYDOWN &&
      event.type != SDL_KEYUP)
    {
      continue;
    }

    // Ignore keyboard auto-repeat
    if (
      event.type == SDL_KEYDOWN &&
      event.key.repeat != 0)
    {
      continue;
    }

    const bool pressed = event.type == SDL_KEYDOWN;

    switch (event.key.keysym.scancode) {
      case SDL_SCANCODE_UP:
        upPressed_ = pressed;
        break;
      case SDL_SCANCODE_DOWN:
        downPressed_ = pressed;
        break;
      case SDL_SCANCODE_LEFT:
        leftPressed_ = pressed;
        break;
      case SDL_SCANCODE_RIGHT:
        rightPressed_ = pressed;
        break;
      default:
        continue;
    }

    publishCommandVelocity();
  }
}

void RobotController::publishCommandVelocity() const
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = (upPressed_ ? kMaximumLinearSpeed : 0.0) - (downPressed_ ? kMaximumLinearSpeed : 0.0);
    msg.angular.z = (leftPressed_ ? kMaximumAngularSpeed : 0.0) - (rightPressed_ ? kMaximumAngularSpeed : 0.0);

    commandVelocityPublisher_->publish(msg);
}