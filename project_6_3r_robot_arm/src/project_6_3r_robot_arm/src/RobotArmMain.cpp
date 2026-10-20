#include <project_6_3r_robot_arm/RobotArm.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotArm>());
  rclcpp::shutdown();

  return 0;
}