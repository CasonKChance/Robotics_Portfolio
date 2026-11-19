cmake_minimum_required(VERSION 3.20)
project(project_7_treasure_hunting_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SDL2 REQUIRED CONFIG COMPONENTS SDL2)
find_package(SDL2_ttf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(project_7_treasure_hunting_robot_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Declare C++ library for core classes
add_library(${PROJECT_NAME}_lib
  src/RobotController.cpp
  src/VelocityLogger.cpp
  src/CoordinatesProcessor.cpp
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
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    SDL2::SDL2
    SDL2_ttf::SDL2_ttf
    ament_index_cpp::ament_index_cpp
    ${project_7_treasure_hunting_robot_interfaces_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(RobotController src/RobotControllerMain.cpp)
target_link_libraries(RobotController
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(VelocityLogger src/VelocityLoggerMain.cpp)
target_link_libraries(VelocityLogger
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(CoordinatesProcessor src/CoordinatesProcessorMain.cpp)
target_link_libraries(CoordinatesProcessor
  PUBLIC
    ${PROJECT_NAME}_lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installation rules
install(
  TARGETS
    ${PROJECT_NAME}_lib
    RobotController
    VelocityLogger
    CoordinatesProcessor
  EXPORT ${PROJECT_NAME}_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/ImageDecoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ImageDecoder
)

install(DIRECTORY
  assets
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
