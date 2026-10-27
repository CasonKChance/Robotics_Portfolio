#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <RoboticArmKinematics/Kinematics.h>

#include "project_6_3r_robot_arm_interfaces/msg/joint_velocities.hpp"
#include "project_6_3r_robot_arm_interfaces/msg/robot_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <numbers>
#include <memory>
#include <array>

/**
 * @brief Stores the dynamic properties and constraints of a revolute joint.
 *
 * Contains the joint's maximum allowable effort and velocity, damping
 * coefficient, friction torque, and axis of rotation.
 */
struct JointDynamics
{
  double max_effort {0.0};                           // Maximum joint torque (N*m).
  double max_velocity {0.0};                         // Maximum joint angular velocity (rad/s).
  double damping {0.0};                              // Viscous damping coefficient (N*m*s/rad).
  double friction {0.0};                             // Joint friction torque (N*m).
  Eigen::Vector3d axis {Eigen::Vector3d::Zero(3)};   // Joint rotation axis.
};

/**
 * @brief Stores the physical and inertial properties of a robot link.
 *
 * Contains the link's mass, length, center of mass, and inertia tensor.
 * The center of mass and inertia tensor are expressed in the link frame.
 */
struct LinkDynamics
{
  double mass {0.0};                                          // Link mass (kg).
  double length {0.0};                                        // Link mass (kg).
  Eigen::Vector3d center_of_mass {Eigen::Vector3d::Zero()};   // Center of mass in the link frame
  Eigen::Matrix3d inertia {Eigen::Matrix3d::Zero()};          // Inertia tensor about center of mass, expressed in the link frame
};

/**
 * @class RobotArm
 * @brief ROS 2 node responsible for simulating and monitoring a 3R robotic arm.
 *
 * Maintains the joint positions and velocities of a three-revolute-joint
 * robotic manipulator, including its physical properties and end-effector
 * position.
 *
 * The node receives robot model information from a URDF description,
 * processes commanded joint velocities, updates the simulated joint
 * dynamics, and publishes joint states.
 */
class RobotArm: public rclcpp::Node
{
public:

  using JointVelocities = project_6_3r_robot_arm_interfaces::msg::JointVelocities;

  /**
   * @brief Constructs the RobotArm ROS 2 node.
   *
   * Initializes the node and its associated ROS 2 communication interfaces
   * and simulation state.
   *
   * @param options ROS 2 node configuration options.
   */
  explicit RobotArm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  Eigen::Vector3d end_effector_position_ {Eigen::Vector3d::Zero(3)};       // Current end-effector position (m).
  std::array < JointDynamics, 3 > joint_dynamics_;                         // Dynamic properties and constraints of the three revolute joints.
  std::array < LinkDynamics, 4 > link_dynamics_;                           // Physical and inertial properties of the four robot links.
  Eigen::Vector3d joint_angles_ {Eigen::Vector3d::Zero()};                 // Current joint angular positions (rad).
  Eigen::Vector3d joint_velocities_ {Eigen::Vector3d::Zero()};             // Current joint angular velocities (rad/s).
  Eigen::Vector3d commanded_joint_velocities_ {Eigen::Vector3d::Zero()};   // Commanded joint angular velocities (rad/s).
  Eigen::Vector3d joint_accelerations_ {Eigen::Vector3d::Zero()};          // Current joint angular accelerations (rad/s^2).
  Eigen::Vector3d velocity_kp_ {Eigen::Vector3d::Constant(5.0)};           // Velocity-controller proportional gain. Units: N*m / (rad/s)
  double y_offset_ {0.0};                                                  // End-effector geometry offset along the y-axis (m).
  double z_offset_ {0.0};                                                  // End-effector geometry offset along the z-axis (m).
  bool robot_description_ready_ {false};                                   // Indicates whether the robot description has been received and processed.
  bool ik_mode_ {false};                                                   // Indicates whether inverse-kinematics mode is enabled.

  rclcpp::Publisher < sensor_msgs::msg::JointState > ::SharedPtr joint_state_publisher_;
  rclcpp::Publisher < project_6_3r_robot_arm_interfaces::msg::RobotState > ::SharedPtr robot_state_publisher_;
  rclcpp::Subscription < JointVelocities > ::SharedPtr joint_velocities_subscription_;
  rclcpp::Subscription < std_msgs::msg::String > ::SharedPtr robot_description_subscription_;
  rclcpp::Subscription < sensor_msgs::msg::JointState > ::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Time last_update_time_;

  /**
   * @brief Advances the robot arm simulation by one time step.
   *
   * Updates the robot's simulated state using the specified elapsed time.
   *
   * @param dt Elapsed simulation time (s).
   */
  void update(double dt);

  /**
   * @brief Updates the joint velocities using the velocity controller.
   *
   * Uses the commanded and current joint velocities, together with
   * the modeled joint dynamics, to update the simulated joint velocities.
   *
   * @param dt Elapsed simulation time (s).
   */
  void updateJointVelocities(double dt);

  /**
   * @brief Processes a commanded joint velocity message.
   *
   * Receives commanded angular velocities for the robot's revolute joints
   * through the associated ROS 2 subscription.
   *
   * @param message Unique pointer to the received joint velocity message.
   */
  void jointVelocitiesTopicCallback(JointVelocities::UniquePtr message);

  /**
   * @brief Processes a received robot description.
   *
   * Receives the robot's URDF description and extracts the information
   * needed to initialize its geometric and dynamic properties.
   *
   * @param message Unique pointer to the received URDF description message.
   */
  void robotDescriptionTopicCallback(std_msgs::msg::String::UniquePtr message);

  /**
   * @brief Processes an externally received joint state message.
   *
   * Receives joint state information from the associated ROS 2
   * subscription for use by the robot arm node.
   *
   * @param message Unique pointer to the received joint state message.
   */
  void jointStateTopicCallback(sensor_msgs::msg::JointState::UniquePtr message);

  /**
   * @brief Publishes the robot arm's current joint states.
   *
   * Publishes joint position and velocity information using a
   * sensor_msgs::msg::JointState message.
   */
  void publishJointStates() const;

  /**
   * @brief Publishes the robot arm's current state.
   */
  void publishRobotState() const;

  /**
   * @brief Logs the current end-effector position.
   *
   * Outputs the end-effector position for monitoring and debugging.
   */
  void logEndEffectorPosition() const;

  /**
   * @brief Normalizes an angle into the range [-π, π] radians.
   * @param angle Angle in radians.
   * @return Equivalent angle wrapped within [-π, π].
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Computes approximate joint rotational inertias.
   *
   * Estimates the effective rotational inertia associated with each
   * revolute joint using the robot's modeled link properties.
   *
   * @return Three-dimensional vector containing the approximate joint
   *         inertias (kg*m^2).
   */
  Eigen::Vector3d computeApproximateJointInertias() const;
};

#endif
