#include <project_7_treasure_hunting_robot/CoordinatesProcessor.h>

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoordinatesProcessor>());
  rclcpp::shutdown();

  return 0;
}
