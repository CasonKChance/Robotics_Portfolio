#include <project_6_2r_robot_arm/RobotArmAnalyzer.h>

#include <memory>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmAnalyzer>());
  rclcpp::shutdown();

  return 0;
}
