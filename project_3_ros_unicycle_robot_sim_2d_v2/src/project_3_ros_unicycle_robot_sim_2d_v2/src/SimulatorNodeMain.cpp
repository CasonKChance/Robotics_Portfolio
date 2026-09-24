#include "project_3_ros_unicycle_robot_sim_2d_v2/SimulatorNode.h"

#include <iostream>
#include <memory>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::placeholders;

using SendWorldData = project_3_ros_unicycle_robot_sim_2d_v2_interfaces::srv::SendWorldData;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto simulatorNode = std::make_shared<SimulatorNode>();
  auto worldDataServer = simulatorNode->create_service<SendWorldData>("send_world_data", std::bind(
    &SimulatorNode::handleWorldDataService,
    simulatorNode.get(),
    _1,
    _2,
    _3));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(simulatorNode);

  while (rclcpp::ok() && simulatorNode->getStatus() == SimulatorState::Running) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Log final simulation terminal state after spin loop exits
  switch(simulatorNode->getStatus()) {
    case SimulatorState::GoalReached:
      RCLCPP_INFO(simulatorNode->get_logger(), "Robot reached the goal!");
      break;
    case SimulatorState::ObstacleCollision:
      RCLCPP_INFO(simulatorNode->get_logger(), "Robot collided with an obstacle!");
      break;
    case SimulatorState::OutOfBounds:
      RCLCPP_INFO(simulatorNode->get_logger(), "Robot went out of bounds!");
      break;
    default:
      RCLCPP_INFO(simulatorNode->get_logger(), "Simulation ended with unknown status.");
  }

  rclcpp::shutdown();

  return 0;
}
