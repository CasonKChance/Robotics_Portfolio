#ifndef ROBOT_OPERATOR_NODE_H
#define ROBOT_OPERATOR_NODE_H

#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/action/go_to_pose.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2/Pose.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <future>
#include <thread>

/**
 * @brief ROS 2 Node providing a user interface and action client interface to send navigation targets to a robot.
 *
 * Prompts user for target poses via standard console input, validates and normalizes the target coordinate inputs,
 * and sends action goal requests to a `GoToPose` action server.
 */
class RobotOperatorNode: public rclcpp::Node
{
public:
  using GoToPose = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle < GoToPose >;

  /**
   * @brief Constructs a RobotOperatorNode instance.
   * @param options Configuration options for Node initialization.
   */
  explicit RobotOperatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Stops and joins the user input thread before destroying the node.
   */
  ~RobotOperatorNode();

private:
  rclcpp_action::Client < GoToPose > ::SharedPtr goToPoseActionClient_;
  std::thread inputThread_;

  /**
   * @brief Sets up the action client
   */
  void setupActionClient();

  /**
   * @brief Continuous loop running on a separate thread to accept pose targets from standard console input.
   */
  void receiveGoalPose();

  /**
   * @brief Asynchronously dispatches a target Pose goal request to the GoToPose action server.
   * @param goalPose Target spatial pose configuration to send to the server.
   * @return Shared future holding the wrapped action result state upon goal completion.
   */
  std::shared_future < GoalHandleGoToPose::WrappedResult > sendGoalPose(const Pose & goalPose);

  /**
   * @brief Prompts user for input via standard console and blocks until valid numeric input is received.
   * @param prompt Descriptive string printed to standard output to prompt input.
   * @return Valid numeric double input parsed from standard input stream.
   */
  double getValidInput(const std::string & prompt) const;

  /**
   * @brief Wraps an angle in radians to the range [-π, π].
   * @param angle Raw input angle in radians.
   * @return Normalized angle bounded within [-π, π].
   */
  double normalizeAngle(double angle) const;
};

#endif
