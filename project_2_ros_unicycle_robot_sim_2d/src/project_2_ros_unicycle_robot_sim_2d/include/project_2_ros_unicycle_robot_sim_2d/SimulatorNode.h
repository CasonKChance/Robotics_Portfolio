#ifndef SIMULATOR_NODE_H
#define SIMULATOR_NODE_H

#include "Robot.h"
#include "World.h"
#include "project_2_ros_unicycle_robot_sim_2d/srv/send_world_data.hpp"
#include "project_2_ros_unicycle_robot_sim_2d/msg/robot_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

using project_2_ros_unicycle_robot_sim_2d::srv::SendWorldData;

/**
 * @brief Represents the current operational state or termination reason of the simulation.
 */
enum class SimulationStatus
{
  Running,             // Simulation is progressing normally
  GoalReached,         // Robot successfully reached the target goal region
  ObstacleCollision,   // Robot collided with an environmental obstacle
  OutOfBounds          // Robot moved outside the valid map boundaries
};

/**
 * @brief ROS 2 Node that manages the discrete 2D unicycle kinematic simulation loop.
 *
 * Subscribes to incoming Twist velocity commands on "cmd_vel", steps the kinematic model
 * at a fixed rate, and evaluates environmental safety checks.
 */
class SimulatorNode: public rclcpp::Node {
public:
  /**
   * @brief Constructs a Simulator instance holding a robot and world model.
   * @param options Configuration options for Node initialization.
   */
  explicit SimulatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Handles sending world data to visualization node via service response
   */
  void handleWorldDataService(
    const std::shared_ptr < rmw_request_id_t > request_header,
    const std::shared_ptr < SendWorldData::Request > request,
    const std::shared_ptr < SendWorldData::Response > response) const;

  /**
   * @brief Gets the current operational or termination status of the simulation.
   * @return Active SimulationStatus enum value.
   */
  SimulationStatus getStatus() const {return status_;}

private:
  Robot robot_;                                           // Robot state and kinematics model
  World world_;                                           // Simulation environment definition
  SimulationStatus status_ {SimulationStatus::Running};   // Active simulation status state machine

  rclcpp::Subscription < geometry_msgs::msg::Twist > ::SharedPtr commandVelocitySubscription_;
  rclcpp::Publisher < project_2_ros_unicycle_robot_sim_2d::msg::RobotPose >
  ::SharedPtr robotPosePublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Topic callback for incoming Twist messages.
   * @param message Pointer to received geometry_msgs::msg::Twist command.
   */
  void topicCallback(geometry_msgs::msg::Twist::UniquePtr message);

  /**
   * @brief Publishes the robot's current pose to topic 'robot_pose'.
   */
  void publishRobotPose() const;

  /**
   * @brief Discrete simulation loop handler executed periodically by wall timer.
   *        Advances time step, evaluates spatial conditions, and terminates on collision or goal.
   */
  void updateLoop();

  /**
   * @brief Evaluates current robot pose against world boundaries, obstacles, and goals.
   * @return The resulting SimulationStatus based on spatial overlap.
   */
  SimulationStatus checkCollision() const;

  /**
   * @brief Builds the world from config/world.yaml
   */
  void buildWorld();
};

#endif
