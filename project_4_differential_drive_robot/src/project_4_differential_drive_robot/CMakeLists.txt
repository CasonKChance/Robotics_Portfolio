cmake_minimum_required(VERSION 3.20)
project(project_4_differential_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(SDL2 REQUIRED CONFIG COMPONENTS SDL2)
find_package(SDL2_ttf REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Declare C++ library for core classes
add_library(${PROJECT_NAME}_lib
  src/Robot.cpp
  src/RobotController.cpp
  src/FrameInspector.cpp
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
    urdf::urdf
    ${geometry_msgs_TARGETS}
    tf2_ros::tf2_ros
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    SDL2::SDL2
    tf2::tf2
    SDL2_ttf::SDL2_ttf
    ament_index_cpp::ament_index_cpp
)

add_executable(Robot src/RobotMain.cpp)
target_link_libraries(Robot
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(RobotController src/RobotControllerMain.cpp)
target_link_libraries(RobotController
  PUBLIC
    ${PROJECT_NAME}_lib
)

add_executable(FrameInspector src/FrameInspectorMain.cpp)
target_link_libraries(FrameInspector
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
    Robot
    RobotController
    FrameInspector
  EXPORT ${PROJECT_NAME}_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
