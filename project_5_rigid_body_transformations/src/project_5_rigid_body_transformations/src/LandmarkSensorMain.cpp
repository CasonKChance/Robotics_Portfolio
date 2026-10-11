#include <project_5_rigid_body_transformations/LandmarkSensor.h>

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkSensor>());
  rclcpp::shutdown();

  return 0;
}
