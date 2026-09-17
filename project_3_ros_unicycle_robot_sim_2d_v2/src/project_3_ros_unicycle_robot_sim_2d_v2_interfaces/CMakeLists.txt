cmake_minimum_required(VERSION 3.20)
project(project_3_ros_unicycle_robot_sim_2d_v2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/Goal.msg"
  "msg/RobotPose.msg"
  "srv/SendWorldData.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
