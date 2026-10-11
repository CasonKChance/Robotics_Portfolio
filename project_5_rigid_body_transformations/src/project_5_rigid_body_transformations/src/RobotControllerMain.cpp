#include <project_5_rigid_body_transformations/RobotController.h>

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();

  return 0;
}
