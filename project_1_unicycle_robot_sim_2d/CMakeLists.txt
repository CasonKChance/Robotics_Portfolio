cmake_minimum_required(VERSION 3.23)

project(unicycle_robot_sim_2d CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#./src
add_subdirectory(src)

# ./tests
option(BUILD_TESTING "Enable testing and build tests" ON)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()