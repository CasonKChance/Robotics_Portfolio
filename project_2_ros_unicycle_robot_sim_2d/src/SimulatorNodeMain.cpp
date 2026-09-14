#include <iostream>
#include <memory>

#include "project_2_ros_unicycle_robot_sim_2d/SimulatorNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simulatorNode = std::make_shared<SimulatorNode>();
    rclcpp::spin(simulatorNode);

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