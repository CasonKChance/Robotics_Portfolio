#ifndef ROBOT_ARM_CONTROLLER_H
#define ROBOT_ARM_CONTROLLER_H

#include <RoboticArmKinematics/Kinematics.h>

#include "project_6_3r_robot_arm_interfaces/msg/joint_velocities.hpp"
#include "project_6_3r_robot_arm_interfaces/msg/robot_state.hpp"
#include "project_6_3r_robot_arm_interfaces/action/go_to_position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <functional>
#include <memory>
#include <thread>
#include <vector>

/**
 * @brief Internal tracking struct for the robot's current kinematics, state, and physical limits.
 */
struct RobotState
{
  double end_effector_x {0.0};
  double end_effector_y {0.0};
  double end_effector_z {0.0};
  double arm1_length {0.0}; // m
  double arm2_length {0.0}; // m
  double arm3_length {0.0}; // m
  double joint_pedestal_arm1_angle {0.0}; // rad
  double joint_arm1_arm2_angle {0.0}; // rad
  double joint_arm2_arm3_angle {0.0}; // rad
  double joint_pedestal_arm1_velocity {0.0}; // rad/s
  double joint_arm1_arm2_velocity {0.0}; // rad/s
  double joint_arm2_arm3_velocity {0.0}; // rad/s
  double joint_pedestal_arm1_maximum_acceleration {0.0}; // rad/s^2
  double joint_arm1_arm2_maximum_acceleration {0.0}; // rad/s^2
  double joint_arm2_arm3_maximum_acceleration {0.0}; // rad/s^2
  double joint_pedestal_arm1_maximum_velocity {0.0}; // rad/s
  double joint_arm1_arm2_maximum_velocity {0.0}; // rad/s
  double joint_arm2_arm3_maximum_velocity {0.0}; // rad/s
  double x_offset {0.0}; // m - Offset of end effector in x direction from origin in zero configuration
  double y_offset {0.0}; // m - Offset of end effector in y direction from origin in zero configuration
  double z_offset {0.0}; // m - Offset of end effector in z direction from origin in zero configuration
};

/**
 * @brief Internal tracking enum for the controllers current state in a given control loop.
 */
enum class ControllerState
{
  Idle,
  RotatingToGoalPosition,
  WaitingForRotationToGoalPositionStop,
  GoalPositionReached,
  GoalCanceled
};

/**
 * @brief ROS 2 action-based controller for a three-joint robotic arm.
 *
 * Uses numerical inverse kinematics to determine target joint angles and
 * closed-loop joint velocity commands to move the end effector to a requested
 * Cartesian position.
 */
class RobotArmController: public rclcpp::Node {
public:
  using GoToPosition = project_6_3r_robot_arm_interfaces::action::GoToPosition;
  using JointVelocities = project_6_3r_robot_arm_interfaces::msg::JointVelocities;
  using GoalHandleGoToPosition = rclcpp_action::ServerGoalHandle < GoToPosition >;

  /**
   * @brief Constructs a RobotArmController instance.
   * @param options Configuration options for Node initialization.
   */
  explicit RobotArmController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  RobotState current_robot_state_;
  ControllerState controller_state_;
  double goal_position_tolerance_;
  std::vector<double> goal_angles_;

  rclcpp_action::Server < GoToPosition > ::SharedPtr go_to_position_action_server_;
  rclcpp::Publisher < JointVelocities > ::SharedPtr joint_velocities_publisher_;
  rclcpp::Subscription < project_6_3r_robot_arm_interfaces::msg::RobotState > ::SharedPtr robot_state_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr < GoalHandleGoToPosition > active_goal_handle_;

  /**
   * @brief Handles incoming action goal requests.
   *
   * Rejects goals if another goal is active, the requested Cartesian position
   * contains non-finite values, or the current robot state is invalid.
   *
   * @param uuid Unique ID of the goal request.
   * @param goal Shared pointer to the requested GoToPosition goal.
   * @return GoalResponse decision (accept/reject).
   */
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr < const GoToPosition::Goal > goal);

  /**
   * @brief Handles incoming cancellation requests for active action goals.
   *
   * Accepts cancellation only if the requested goal matches the currently
   * active goal. Joint motion is stopped by the control loop before the
   * cancellation is finalized.
   *
   * @param goalHandle Handle to the goal being canceled.
   * @return CancelResponse decision (accept/reject).
   */
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr < GoalHandleGoToPosition > goalHandle);

  /**
   * @brief Initializes execution of an accepted action goal.
   *
   * Stores the active goal handle and transitions the controller into
   * the RotatingToGoalPosition state.
   *
   * @param goalHandle Handle to the accepted goal.
   */
  void handleAccepted(const std::shared_ptr < GoalHandleGoToPosition > goalHandle);

  /**
   * @brief Executes one iteration of the closed-loop controller state machine.
   *
   * Calculates target joint angles using inverse kinematics, publishes
   * joint velocity commands, and monitors joint motion and end-effector
   * position to determine when the goal has been reached.
   *
   * Accounts for joint stopping distances, handles goal cancellation,
   * publishes action feedback, and corrects residual positioning error.
   */
  void controlLoop();

  /**
   * @brief Packages and publishes joint velocity commands to "joint_velocities".
   * @param jointVelocities Target velocities for the three joints (rad/s).
   * @throws std::invalid_argument If the provided vector does not contain exactly three velocities.
   */
  void publishCommandJointVelocities(const std::vector<double> & jointVelocities);

  /**
   * @brief Subscriber callback for robot state updates from topic "robot_state".
   *
   * Updates the internally tracked end-effector position, link lengths,
   * joint angles, joint velocities, motion limits, and positional offsets.
   *
   * @param message Unique pointer to the incoming RobotState message.
   */
  void robotStateTopicCallback(project_6_3r_robot_arm_interfaces::msg::RobotState::UniquePtr message);

  /**
   * @brief Calculates the shortest angular difference between current and target joint angles.
   * @param goalAngles Target joint angles for the three joints (rad).
   * @return Vector of remaining joint rotations in radians, bounded within [-π, π].
   * @throws std::invalid_argument If the provided vector does not contain exactly three angles.
   */
  std::vector<double> getRotationRemaining(const std::vector<double> & goalAngles);

  /**
   * @brief Calculates the required stopping distance given current velocity and maximum deceleration.
   * @param velocity Current joint angular velocity (rad/s).
   * @param acceleration Maximum joint deceleration magnitude (rad/s^2).
   * @return Angular stopping distance (rad), or infinity if the inputs are invalid.
   */
  double getStoppingDistance(const double velocity, const double acceleration) const;

  /**
   * @brief Solves numerical inverse kinematics for a target end-effector position.
   *
   * Uses the current joint angles as the initial guess and accounts for
   * the end-effector offset from the robot's zero configuration.
   * Normalizes the resulting angles and verifies the solution using
   * forward kinematics and the configured position tolerance.
   *
   * @param goal Target Cartesian end-effector position (m).
   * @return Vector of three target joint angles (rad), or an empty vector
   *         if no valid solution is found.
   */
  std::vector<double> solveGoalAngles(const Eigen::Vector3d & goal);

  /**
   * @brief Wraps an angle in radians to the range [-π, π].
   * @param angle Raw input angle in radians.
   * @return Normalized angle bounded within [-π, π].
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Returns the rotation axes of the three robotic arm joints.
   *
   * The pedestal joint rotates around the Z axis, while the remaining
   * two joints rotate around the Y axis.
   *
   * @return Vector of three joint rotation axes in joint order.
   */
  std::vector<Eigen::VectorXd> jointAxes();

  /**
   * @brief Retrieves the current lengths of the robotic arm links.
   * @return Vector containing the three link lengths (m) in joint order.
   */
  std::vector<double> linkLengths();

  /**
   * @brief Retrieves the current angular positions of the robotic arm joints.
   * @return Vector containing the three joint angles (rad) in joint order.
   */
  std::vector<double> jointAngles();

  /**
   * @brief Retrieves the current angular velocities of the robotic arm joints.
   * @return Vector containing the three joint velocities (rad/s) in joint order.
   */
  std::vector<double> jointVelocities();

  /**
   * @brief Retrieves the maximum angular accelerations of the robotic arm joints.
   * @return Vector containing the three maximum joint accelerations (rad/s^2) in joint order.
   */
  std::vector<double> maximumJointAccelerations();

  /**
   * @brief Retrieves the maximum angular velocities of the robotic arm joints.
   * @return Vector containing the three maximum joint velocities (rad/s) in joint order.
   */
  std::vector<double> maximumJointVelocities();

  /**
   * @brief Retrieves the current measured Cartesian position of the end effector.
   * @return Three-dimensional end-effector position vector (m).
   */
  Eigen::Vector3d measuredPosition();
  
  /**
   * @brief Retrieves the end-effector positional offset in the robot's zero configuration.
   * @return Three-dimensional positional offset vector (m).
   */
  Eigen::Vector3d measuredOffset();

  /**
   * @brief Validates the current robot state for use by the controller.
   *
   * Verifies that link lengths are positive and finite, and that joint
   * angles, velocities, acceleration limits, velocity limits,
   * end-effector position, and positional offsets contain finite values.
   *
   * @return True if all checked robot state values are valid; otherwise false.
   */
  bool validRobotState();
};

#endif