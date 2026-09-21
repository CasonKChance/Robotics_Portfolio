cmake_minimum_required(VERSION 3.20)
project(project_3_ros_unicycle_robot_sim_2d_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(project_3_ros_unicycle_robot_sim_2d_v2_interfaces REQUIRED)

# Declare C++ library for core simulation classes
add_library(${PROJECT_NAME}_lib
  src/Obstacle.cpp
  src/Goal.cpp
  src/Robot.cpp
  src/World.cpp
  src/SimulatorNode.cpp
  src/RobotControllerNode.cpp
  src/RobotOperatorNode.cpp
)

# Export include directories
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    ${geometry_msgs_TARGETS}
    ${project_3_ros_unicycle_robot_sim_2d_v2_interfaces_TARGETS}
)

add_executable(RobotControllerNode src/RobotControllerNodeMain.cpp)
target_link_libraries(RobotControllerNode
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(SimulatorNode src/SimulatorNodeMain.cpp)
target_link_libraries(SimulatorNode
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(RobotOperatorNode src/RobotOperatorNodeMain.cpp)
target_link_libraries(RobotOperatorNode
  PUBLIC
    ${PROJECT_NAME}_lib
)

install(
  PROGRAMS
    src/VisualizationNode.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME VisualizationNode
)

# Enable testing
option(BUILD_TESTING "Enable testing and build tests" ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add the GoogleTest target
  ament_add_gtest(TestSimulatorNode
    tests/SimulatorNodeTests.cpp
  )
  target_link_libraries(TestSimulatorNode
    ${PROJECT_NAME}_lib
  )
endif()

# Installation rules
install(
  TARGETS
    ${PROJECT_NAME}_lib
    RobotControllerNode
    SimulatorNode
    RobotOperatorNode
  EXPORT ${PROJECT_NAME}_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
