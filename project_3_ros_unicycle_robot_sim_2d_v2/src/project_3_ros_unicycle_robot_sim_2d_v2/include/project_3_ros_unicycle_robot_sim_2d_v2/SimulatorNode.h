#ifndef SIMULATOR_NODE_H
#define SIMULATOR_NODE_H

#include "Robot.h"
#include "World.h"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/srv/send_world_data.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/msg/robot_state.hpp"
#include "project_3_ros_unicycle_robot_sim_2d_v2_interfaces/msg/simulator_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

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
  using SendWorldData = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::srv::SendWorldData;

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
  rclcpp::Publisher < project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::RobotState >
  ::SharedPtr robotStatePublisher_;
  rclcpp::Publisher < project_3_ros_unicycle_robot_sim_2d_v2_interfaces::msg::SimulatorStatus >
  ::SharedPtr simulatorStatusPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Topic callback for incoming Twist messages.
   * @param message Pointer to received geometry_msgs::msg::Twist command.
   */
  void topicCallback(geometry_msgs::msg::Twist::UniquePtr message);

  /**
   * @brief Publishes the robot's current state to topic 'robot_state'.
   */
  void publishRobotState() const;

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

  /**
   * @brief Configures the robot's physical limits
   */
  void configureRobotLimits();
};

#endif
