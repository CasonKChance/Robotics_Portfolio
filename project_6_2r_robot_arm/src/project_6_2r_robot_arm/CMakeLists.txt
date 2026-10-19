cmake_minimum_required(VERSION 3.20)
project(project_6_2r_robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(
    ../../../RoboticArmKinematics
    ${CMAKE_CURRENT_BINARY_DIR}/RoboticArmKinematics
)

# Declare C++ library for core classes
# add_library(${PROJECT_NAME}_lib
# )

# Export include directories
# target_include_directories(${PROJECT_NAME}_lib
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )

# target_link_libraries(${PROJECT_NAME}_lib
#   PUBLIC
#     rclcpp::rclcpp
#     urdf::urdf
#     ${geometry_msgs_TARGETS}
#     ${std_msgs_TARGETS}
#     Kinematics
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installation rules
install(
  TARGETS
    Kinematics
#    ${PROJECT_NAME}_lib
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

ament_package()
