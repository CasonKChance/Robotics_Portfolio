#include <memory>

#include "project_3_ros_unicycle_robot_sim_2d_v2/CommandVelocityPublisherNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandVelocityPublisherNode>());
  rclcpp::shutdown();

  return 0;
}
