#ifndef SIMULATOR_NODE_H
#define SIMULATOR_NODE_H

#include "Robot.h"
#include "World.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>

/**
 * @brief Represents the current operational state or termination reason of the simulation.
 */
enum class SimulationStatus {
    Running,           // Simulation is progressing normally
    GoalReached,       // Robot successfully reached the target goal region
    ObstacleCollision, // Robot collided with an environmental obstacle
    OutOfBounds        // Robot moved outside the valid map boundaries
};

/**
 * @brief Manages simulation time, advances a Robot through discrete kinematic updates,
 *        logs telemetry data, and evaluates environmental safety checks.
 */
class SimulatorNode: public rclcpp::Node {
public:
  /**
   * @brief Constructs a Simulator instance holding a robot and world.
   */
  explicit SimulatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Gets the current operational or termination status of the simulation.
   * @return Active SimulationStatus enum value.
   */
  SimulationStatus getStatus() const { return status_; }

private:
  Robot robot_;                                           // Robot model.
  World world_;                                           // Simulation world (obstacles, boundaries, etc.).
  SimulationStatus status_{ SimulationStatus::Running };  // Active simulation state tracker
  rclcpp::Subscription< geometry_msgs::msg::Twist > ::SharedPtr commandVelocitySubscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  void topicCallback(geometry_msgs::msg::Twist::UniquePtr message);

  void updateLoop();

  /**
   * @brief Evaluates current robot pose against world boundaries, obstacles, and goals.
   * @return The resulting SimulationStatus based on current pose overlap.
   */
  SimulationStatus checkCollision() const;
};

#endif