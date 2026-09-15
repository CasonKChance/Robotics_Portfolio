cmake_minimum_required(VERSION 3.20)
project(project_2_ros_unicycle_robot_sim_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/Goal.msg"
  "srv/SendWorldData.srv"
  DEPENDENCIES geometry_msgs
)

# Declare C++ library for core simulation classes
add_library(${PROJECT_NAME}_lib
  src/Obstacle.cpp
  src/Goal.cpp
  src/Robot.cpp
  src/World.cpp
  src/CommandVelocityPublisherNode.cpp
  src/SimulatorNode.cpp
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
    ${geometry_msgs_TARGETS}
)

add_executable(CommandVelocityPublisherNode src/CommandVelocityPublisherNodeMain.cpp)
target_link_libraries(CommandVelocityPublisherNode
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(SimulatorNode src/SimulatorNodeMain.cpp)
target_link_libraries(SimulatorNode
  PUBLIC
    ${PROJECT_NAME}_lib
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
  ament_add_gtest(TestCommandVelocityPublisherNode
    tests/CommandVelocityPublisherNodeTests.cpp
  )
  ament_add_gtest(TestSimulatorNode
    tests/SimulatorNodeTests.cpp
  )

  target_link_libraries(TestCommandVelocityPublisherNode
    ${PROJECT_NAME}_lib
  )
  target_link_libraries(TestSimulatorNode
    ${PROJECT_NAME}_lib
  )
endif()

# Installation rules
install(
  TARGETS
    ${PROJECT_NAME}_lib
    CommandVelocityPublisherNode
    SimulatorNode
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
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
