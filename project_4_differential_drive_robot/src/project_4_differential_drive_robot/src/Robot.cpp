#include <project_4_differential_drive_robot/Robot.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cmath>
#include <algorithm>

using namespace std::placeholders;
using namespace std::chrono_literals;

/* Public Member Functions */

Robot::Robot(const Pose & initialPose, const rclcpp::NodeOptions & options)
: Node("robot", options),
  pose_{initialPose},
  velocityCommand_{Twist{0.0, 0.0}},
  currentTwist_{Twist{0.0, 0.0}}
{
  // Ensure initial heading is properly normalized to [-π, π]
  pose_.theta = normalizeAngle(pose_.theta);

  robotDescriptionSubscription_ =
    this->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(
      &Robot::robotDescriptionTopicCallback,
      this,
      _1));

  commandVelocitySubscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&Robot::commandVelocityTopicCallback, this, _1)
  );

  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  updateTimer_ = this->create_wall_timer(
    10ms, [this]() {this->update(0.01);});
}

/* Private Member Functions */

void Robot::update(double dt)
{
  if (dt <= 0.0) {
    return;
  }

  updateTwist(dt);

  double v = currentTwist_.linearVelocity;
  double omega = currentTwist_.angularVelocity;

  pose_.x += v * std::cos(pose_.theta) * dt;
  pose_.y += v * std::sin(pose_.theta) * dt;
  pose_.theta = normalizeAngle(pose_.theta + (omega * dt));

  broadcastTransform();
}

void Robot::commandVelocityTopicCallback(geometry_msgs::msg::Twist::UniquePtr message)
{
  if (message->linear.x == velocityCommand_.linearVelocity &&
    message->angular.z == velocityCommand_.angularVelocity)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Updating velocity command with: \n"
                                    "\tLinear: %.2f m/s\n"
                                    "\tAngular: %.2f rad/s", message->linear.x,
                                                               message->angular.z);

  velocityCommand_.linearVelocity = message->linear.x;
  velocityCommand_.angularVelocity = message->angular.z;
}

void Robot::robotDescriptionTopicCallback(
  std_msgs::msg::String::UniquePtr message)
{
  // Pull model
  urdf::Model model;

  if (!model.initString(message->data)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to parse robot description.");
    return;
  }

  const auto leftWheelJoint =
    model.getJoint("left_wheel_link_joint");

  const auto rightWheelJoint =
    model.getJoint("right_wheel_link_joint");

  const auto leftWheelLink =
    model.getLink("left_wheel_link");

  const auto rightWheelLink =
    model.getLink("right_wheel_link");

  if (!leftWheelJoint ||
    !rightWheelJoint ||
    !leftWheelLink ||
    !rightWheelLink)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Required differential-drive links/joints not found in URDF.");
    return;
  }

  // Validate model

  if (!leftWheelJoint->limits ||
    !rightWheelJoint->limits ||
    !leftWheelLink->inertial ||
    !rightWheelLink->inertial ||
    !leftWheelLink->collision ||
    !rightWheelLink->collision)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Wheel limits, inertials, or collision geometry missing.");
    return;
  }

  // Extract wheel geometry

  const auto * leftCylinder =
    dynamic_cast<const urdf::Cylinder *>(
    leftWheelLink->collision->geometry.get());

  const auto * rightCylinder =
    dynamic_cast<const urdf::Cylinder *>(
    rightWheelLink->collision->geometry.get());

  if (!leftCylinder || !rightCylinder) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Wheel collision geometry must be cylindrical.");
    return;
  }

  if (leftCylinder->radius != rightCylinder->radius) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Left and right wheel radii must be equal.");
    return;
  }

  wheelRadius_ = leftCylinder->radius;

  if (leftWheelLink->inertial->iyy != rightWheelLink->inertial->iyy) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Left and right wheel inertias must be equal.");
    return;
  }

  // Extract wheel attributes

  wheelMomentOfInertia_ = leftWheelLink->inertial->iyy;

  if (leftWheelJoint->limits->velocity != rightWheelJoint->limits->velocity) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Left and right wheel velocity limits must be equal.");
    return;
  }

  maximumWheelVelocity_ = leftWheelJoint->limits->velocity;

  if (leftWheelJoint->limits->effort != rightWheelJoint->limits->effort) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Left and right wheel effort limits must be equal.");
    return;
  }

  maximumWheelEffort_ = leftWheelJoint->limits->effort;

  const double leftWheelY = leftWheelJoint->parent_to_joint_origin_transform.position.y;
  const double rightWheelY = rightWheelJoint->parent_to_joint_origin_transform.position.y;

  wheelSeparation_ =
    std::abs(leftWheelY - rightWheelY);

  robotDescriptionReady_ = true;

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded differential-drive parameters:\n"
    "\tWheel radius: %.3f m\n"
    "\tWheel separation: %.3f m\n"
    "\tWheel inertia: %.6f kg*m^2\n"
    "\tMax wheel velocity: %.3f rad/s\n"
    "\tMax wheel effort: %.3f N*m",
    wheelRadius_,
    wheelSeparation_,
    wheelMomentOfInertia_,
    maximumWheelVelocity_,
    maximumWheelEffort_);
}

double Robot::normalizeAngle(double angle) const
{
  // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
  return std::atan2(std::sin(angle), std::cos(angle));
}

void Robot::updateTwist(double dt)
{
  if (!robotDescriptionReady_ || dt <= 0.0) {
    return;
  }

  /*
   * Differential-drive inverse kinematics.
   *
   * v = r/2 * (ωᵣ + ωₗ)
   * ω = r/L * (ωᵣ - ωₗ)
   *
   * Therefore:
   *
   * ωₗ = (v - ω * L/2) / r
   * ωᵣ = (v + ω * L/2) / r
   */
  double targetLeftWheelVelocity =
    (velocityCommand_.linearVelocity -
    (velocityCommand_.angularVelocity * wheelSeparation_ / 2.0)) /
    wheelRadius_;

  double targetRightWheelVelocity =
    (velocityCommand_.linearVelocity +
    (velocityCommand_.angularVelocity * wheelSeparation_ / 2.0)) /
    wheelRadius_;

  /*
   * Respect the URDF wheel velocity limit.
   *
   * Scale BOTH wheels by the same amount instead of independently
   * clamping them. This preserves the curvature of the commanded
   * motion.
   */
  const double largestTargetWheelVelocity =
    std::max(
    std::abs(targetLeftWheelVelocity),
    std::abs(targetRightWheelVelocity));

  if (largestTargetWheelVelocity > maximumWheelVelocity_) {
    const double scale =
      maximumWheelVelocity_ / largestTargetWheelVelocity;

    targetLeftWheelVelocity *= scale;
    targetRightWheelVelocity *= scale;
  }

  /*
   * Effort is torque:
   *
   *     Ꚍ = I * α
   *
   * so:
   *
   *     α = Ꚍ / I
   *
   */
  const double maximumAcceleration =
    maximumWheelEffort_ / wheelMomentOfInertia_;

  const double maximumVelocityChange = maximumAcceleration * dt;

  /*
   * Move each actual wheel velocity toward its target without allowing
   * it to overshoot the target.
   */
  const double leftError =
    targetLeftWheelVelocity - leftWheelVelocity_;

  const double rightError =
    targetRightWheelVelocity - rightWheelVelocity_;

  leftWheelVelocity_ += std::clamp(
    leftError,
    -maximumVelocityChange,
    maximumVelocityChange);

  rightWheelVelocity_ += std::clamp(
    rightError,
    -maximumVelocityChange,
    maximumVelocityChange);

  /*
   * Numerical safety. These should already be within these bounds
   * because the targets were limited above.
   */
  leftWheelVelocity_ = std::clamp(
    leftWheelVelocity_,
    -maximumWheelVelocity_,
    maximumWheelVelocity_);

  rightWheelVelocity_ = std::clamp(
    rightWheelVelocity_,
    -maximumWheelVelocity_,
    maximumWheelVelocity_);

  /*
   * Differential-drive forward kinematics.
   *
   * Wheel velocities are now the source of truth for the robot's
   * actual velocity.
   */
  currentTwist_.linearVelocity =
    (wheelRadius_ / 2.0) *
    (rightWheelVelocity_ + leftWheelVelocity_);

  currentTwist_.angularVelocity =
    (wheelRadius_ / wheelSeparation_) *
    (rightWheelVelocity_ - leftWheelVelocity_);
}

void Robot::broadcastTransform() const {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = pose_.x;
    t.transform.translation.y = pose_.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = sin(pose_.theta / 2.0);
    t.transform.rotation.w = cos(pose_.theta / 2.0);

    tfBroadcaster_->sendTransform(t);
}
