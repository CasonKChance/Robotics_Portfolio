#include "project_4_differential_drive_robot/Robot.h"
#include "project_4_differential_drive_robot/Pose.h"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot>(Pose{0.0, 0.0, 0.0}));
  rclcpp::shutdown();

  return 0;
}
