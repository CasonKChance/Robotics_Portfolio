#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/action/go_to_pose.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/msg/robot_state.hpp"
#include "Pose.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <functional>
#include <memory>
#include <thread>

/**
 * @brief Internal tracking struct for the robot's current kinematics, state, and physical limits.
 */
struct RobotState
{
  double x {0.0};
  double y {0.0};
  double theta {0.0};
  double angularVelocity {0.0}; // m/s
  double linearVelocity {0.0}; // m/s
  double maximumLinearVelocity {0.0}; // m/s
  double maximumAngularVelocity {0.0}; // rads/s
  double linearAcceleration {0.0}; // m/s^2
  double angularAcceleration {0.0}; // rads/s^2
};

/**
 * @brief Internal tracking enum for the controllers current state in a given control loop.
 */
enum class ControllerState
{
  Idle,
  RotatingToGoalPosition,
  WaitingForRotationToGoalPositionStop,
  DrivingToGoalPose,
  WaitingForDrivingToGoalPoseStop,
  RotatingToGoalPose,
  WaitingForRotationToGoalPoseStop,
  GoalPoseReached,
  GoalCanceled
};

/**
 * @brief ROS 2 Node providing closed-loop controller action management to drive a robot to a goal pose.
 *
 * Implements a `GoToPose` action server that executes a 3-stage maneuver: orienting toward the goal,
 * translating to the goal location, and rotating to the target final heading.
 */
class RobotControllerNode: public rclcpp::Node {
public:
  using GoToPose = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle < GoToPose >;

  /**
   * @brief Constructs a RobotControllerNode instance.
   * @param options Configuration options for Node initialization.
   */
  explicit RobotControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server < GoToPose > ::SharedPtr goToPoseActionServer_;
  rclcpp::Publisher < geometry_msgs::msg::Twist > ::SharedPtr commandVelocityPublisher_;
  rclcpp::Subscription < project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState >
  ::SharedPtr robotPoseSubscription_;
  rclcpp::TimerBase::SharedPtr controlTimer_;
  std::shared_ptr < GoalHandleGoToPose > activeGoalHandle_;
  RobotState currentRobotState_;
  ControllerState controllerState_;
  double goalPosePositionalTolerance_;
  double goalPoseHeadingTolerance_;

  /**
   * @brief Handles incoming action goal requests.
   * @param uuid Unique ID of the goal request.
   * @param goal Shared pointer to requested GoToPose goal.
   * @return GoalResponse decision (accept/reject).
   */
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr < const GoToPose::Goal > goal);

  /**
   * @brief Handles incoming cancellation requests for active action goals.
   * @param goalHandle Handle to the active goal being canceled.
   * @return CancelResponse decision.
   */
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr < GoalHandleGoToPose >
    goalHandle);

  /**
   * @brief Callback executed when a goal request is accepted to spin off execution thread.
   * @param goalHandle Handle to the accepted goal.
   */
  void handleAccepted(const std::shared_ptr < GoalHandleGoToPose > goalHandle);

  /**
   * @brief Executes one iteration of the closed-loop controller state machine.
   *
   * Evaluates the active goal and current robot state, then commands the robot
   * through the sequence of rotating toward the goal position, driving to the
   * goal position, and rotating to the final goal heading.
   */
  void controlLoop();

  /**
   * @brief Calculates Euclidean distance remaining between current state and target goal pose.
   * @param goalPose Target spatial pose configuration.
   * @return Distance remaining in meters.
   */
  double getDistanceRemaining(const Pose & goalPose) const;

  /**
   * @brief Calculates shortest angular difference between current heading and target heading.
   * @param goalPose Target spatial pose configuration.
   * @return Angular difference remaining in radians bounded within [-π, π].
   */
  double getRotationRemaining(const Pose & goalPose) const;

  /**
   * @brief Calculates angular error between current heading and vector pointing directly to target position.
   * @param goalPose Target spatial pose configuration.
   * @return Angular correction needed in radians bounded within [-π, π].
   */
  double getRotationToGoalPositionRemaining(const Pose & targetPose) const;

  /**
   * @brief Calculates required braking distance given current velocity and maximum deceleration.
   * @param velocity Current speed (linear or angular).
   * @param acceleration Active deceleration limit.
   * @return Distance needed to come to a full stop.
   */
  double getStoppingDistance(const double velocity, const double acceleration) const;

  /**
   * @brief Wraps an angle in radians to the range [-π, π].
   * @param angle Raw input angle in radians.
   * @return Normalized angle bounded within [-π, π].
   */
  double normalizeAngle(double angle) const;

  /**
   * @brief Helper function to package and publish Twist command messages to "cmd_vel".
   * @param linearVelocity Target linear velocity command (m/s).
   * @param angularVelocity Target angular velocity command (rad/s).
   */
  void publishCommandVelocity(double linearVelocity, double angularVelocity);

  /**
   * @brief Subscriber callback for robot state updates from topic "robot_state".
   * @param message Unique pointer to incoming RobotState message.
   */
  void robotPoseTopicCallback(
    project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState::UniquePtr message);
};

#endif
