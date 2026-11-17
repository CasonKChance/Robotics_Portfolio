#include <project_7_treasure_hunting_robot/VelocityLogger.h>

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityLogger>());
  rclcpp::shutdown();

  return 0;
}
