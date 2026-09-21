#include <memory>

#include "project_3_ros_unicycle_robot_sim_2d_v2/RobotOperatorNode.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotOperatorNode>());
  rclcpp::shutdown();

  return 0;
}
