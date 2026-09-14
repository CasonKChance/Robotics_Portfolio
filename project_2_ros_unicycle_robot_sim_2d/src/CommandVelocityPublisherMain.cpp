#include <memory>

#include "project_2_ros_unicycle_robot_sim_2d/CommandVelocityPublisher.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandVelocityPublisher>());
  rclcpp::shutdown();

  return 0;
}