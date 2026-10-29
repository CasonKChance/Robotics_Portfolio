#include <project_6_3r_robot_arm/RobotArmController.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArmController>());
  rclcpp::shutdown();

  return 0;
}
