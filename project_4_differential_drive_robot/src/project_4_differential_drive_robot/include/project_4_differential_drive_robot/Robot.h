#ifndef ROBOT_H
#define ROBOT_H

#include "Pose.h"
#include "Twist.h"

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <numbers>
#include <memory>

/**
 * @brief Simulates a differential drive mobile robot.
 *
 * Tracks the current 2D pose [x, y, θ]ᵀ and updates state using discrete
 * Forward Euler numerical integration given a target linear and angular velocity.
 */
class Robot: public rclcpp::Node {
public:
  /**
   * @brief Constructs a Robot instance with a given initial pose.
   * @param initialPose Initial spatial configuration [x, y, θ]ᵀ in the World frame.
   * @param options Configuration options for Node initialization.
   */
  explicit Robot(
    const Pose & initial_pose,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  Pose pose_;                               // Current state [x, y, θ]ᵀ in the World frame.
  Twist velocityCommand_;                   // Current active velocity command [v, ω]ᵀ.
  Twist currentTwist_;                      // Actual velocity ([v, ω]ᵀ) of the robot, clamped from the command and physical limits.
  double leftWheelVelocity_ {0.0};          // Actual velocity of the left wheel (rads/s)
  double rightWheelVelocity_ {0.0};         // Actual velocity of the right wheel (rads/s)
  double wheelRadius_ {0.0};                // Wheel radius, described in URDF
  double wheelSeparation_ {0.0};            // Distance between the left and right wheels, described in URDF
  double wheelMomentOfInertia_ {0.0};       // Wheel moment of inertia, described in URDF
  double maximumWheelVelocity_ {0.0};       // Wheel joint maximum velocity, described in URDF
  double maximumWheelEffort_ {0.0};         // Wheel joint maximum effort, described in URDF
  bool robotDescriptionReady_ {false};      // Indicates whether the robot's description has been fully loaded and initialized.

  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr commandVelocitySubscription_;
  rclcpp::Subscription < std_msgs::msg::String > ::SharedPtr robotDescriptionSubscription_;
  rclcpp::TimerBase::SharedPtr updateTimer_;

  /**
   * @brief Advances the robot's state using Forward Euler integration.
   */
  void update(double dt);

  /**
   * @brief Topic callback for incoming Twist messages.
   * @param message Pointer to received geometry_msgs::msg::Twist command.
   */
  void commandVelocityTopicCallback(geometry_msgs::msg::Twist::UniquePtr message);

  /**
   * @brief Topic callback for incoming robot description messages.
   * @param message Pointer to received std_msgs::msg::String robot description.
   */
  void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

  /**
   * @brief Normalizes an angle into the range [-π, π] radians.
   * @param angle Angle in radians.
   * @return Equivalent angle wrapped within [-π, π].
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Updates the robot's actual velocity ([v, ω]ᵀ).
   */
  void updateTwist(double dt);
};

#endif
