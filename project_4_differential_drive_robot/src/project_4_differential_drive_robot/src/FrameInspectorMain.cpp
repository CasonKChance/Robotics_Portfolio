#include <project_4_differential_drive_robot/FrameInspector.h>

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameInspector>());
  rclcpp::shutdown();

  return 0;
}
