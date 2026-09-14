cmake_minimum_required(VERSION 3.20)
project(project_2_ros_unicycle_robot_sim_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare C++ library for core simulation classes
add_library(${PROJECT_NAME}_lib
  src/Obstacle.cpp
  src/Robot.cpp
  src/World.cpp
  src/CommandVelocityPublisher.cpp
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

add_executable(CommandVelocityPublisher src/CommandVelocityPublisher.cpp)
target_link_libraries(CommandVelocityPublisher
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
  ament_add_gtest(TestCommandVelocityPublisher
    tests/CommandVelocityPublisherTests.cpp
  )

  target_link_libraries(TestCommandVelocityPublisher
    ${PROJECT_NAME}_lib
  )
endif()

# Installation rules
install(
  TARGETS ${PROJECT_NAME}_lib CommandVelocityPublisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
