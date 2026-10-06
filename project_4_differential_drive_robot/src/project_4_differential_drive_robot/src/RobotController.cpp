#include <project_4_differential_drive_robot/RobotController.h>

#include <ament_index_cpp/get_package_share_path.hpp>

#include <stdexcept>
#include <string>
#include <filesystem>

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

  if (TTF_Init() == -1) {
    SDL_Quit();
    throw std::runtime_error(
            std::string("Failed to initialize SDL_ttf: ") +
            TTF_GetError());
  }

  window_ = SDL_CreateWindow(
                "Differential Drive Teleop",
                SDL_WINDOWPOS_CENTERED,
                SDL_WINDOWPOS_CENTERED,
                600,
                150,
                SDL_WINDOW_SHOWN);

  if (!window_) {
    const std::string error = SDL_GetError();

    TTF_Quit();
    SDL_Quit();

    throw std::runtime_error(
            "Failed to create SDL window: " + error);
  }

  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  if (!renderer_) {
    const std::string error = SDL_GetError();

    SDL_DestroyWindow(window_);
    TTF_Quit();
    SDL_Quit();

    throw std::runtime_error("Failed to create SDL renderer: " + error);
  }

  std::string font_path =
    (ament_index_cpp::get_package_share_path("project_4_differential_drive_robot") / "assets" /
    "arial.ttf").string();
  font_ = TTF_OpenFont(font_path.c_str(), 18);
  if (!font_) {
    const std::string error = TTF_GetError();

    std::cerr << "Failed to load font: " << error << std::endl;
  } else {
    SDL_Color textColor = {255, 255, 255, 255};
    SDL_Surface * textSurface = TTF_RenderText_Blended(font_,
      "Focus this window and use the arrow keys to control the robot", textColor);

    if (textSurface) {
      textTexture_ = SDL_CreateTextureFromSurface(renderer_, textSurface);

      textRect_.w = textSurface->w;
      textRect_.h = textSurface->h;
      textRect_.x = (600 - textRect_.w) / 2;
      textRect_.y = (150 - textRect_.h) / 2;

      SDL_FreeSurface(textSurface);

      SDL_SetRenderDrawColor(renderer_, 25, 25, 25, 255);
      SDL_RenderClear(renderer_);
      SDL_RenderCopy(renderer_, textTexture_, NULL, &textRect_);
      SDL_RenderPresent(renderer_);
    }
  }

  RCLCPP_INFO(this->get_logger(),
    "Teleop controller started. Focus the SDL window and use the arrow keys to control robot.");

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

  if (textTexture_) {
    SDL_DestroyTexture(textTexture_);
  }
  if (font_) {
    TTF_CloseFont(font_);
  }
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }

  TTF_Quit();
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

  if (renderer_) {
    SDL_SetRenderDrawColor(renderer_, 25, 25, 25, 255);
    SDL_RenderClear(renderer_);

    if (textTexture_) {
      SDL_RenderCopy(renderer_, textTexture_, NULL, &textRect_);
    }

    SDL_RenderPresent(renderer_);
  }
}

void RobotController::publishCommandVelocity() const
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = (upPressed_ ? kMaximumLinearSpeed : 0.0) -
    (downPressed_ ? kMaximumLinearSpeed : 0.0);
  msg.angular.z = (leftPressed_ ? kMaximumAngularSpeed : 0.0) -
    (rightPressed_ ? kMaximumAngularSpeed : 0.0);

  commandVelocityPublisher_->publish(msg);
}
