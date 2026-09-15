#include "project_2_ros_unicycle_robot_sim_2d/SimulatorNode.h"

#include <iostream>
#include <memory>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using project_2_ros_unicycle_robot_sim_2d::srv::SendWorldData;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto simulatorNode = std::make_shared<SimulatorNode>();
  auto worldDataServer = simulatorNode->create_service<SendWorldData>("send_world_data", std::bind(
    &SimulatorNode::handleWorldDataService,
    simulatorNode.get(),
    std::placeholders::_1,
    std::placeholders::_2,
    std::placeholders::_3));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(simulatorNode);

  while (rclcpp::ok() && simulatorNode->getStatus() == SimulationStatus::Running) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Log final simulation terminal state after spin loop exits
  switch(simulatorNode->getStatus()) {
    case SimulationStatus::GoalReached:
      std::cout << "Robot reached the goal!\n";
      break;
    case SimulationStatus::ObstacleCollision:
      std::cout << "Robot collided with an obstacle!\n";
      break;
    case SimulationStatus::OutOfBounds:
      std::cout << "Robot went out of bounds!\n";
      break;
    default:
      std::cout << "Simulation ended with unknown status.\n";
  }

  rclcpp::shutdown();

  return 0;
}
