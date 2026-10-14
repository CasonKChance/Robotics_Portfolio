cmake_minimum_required(VERSION 3.20)
project(
    PlanarRoboticArmKinematics
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include(CTest)

add_library(Kinematics SHARED src/Kinematics.cpp)
target_include_directories(Kinematics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(Kinematics
    PUBLIC
        Eigen3::Eigen
)

option(BUILD_TESTING "Enable testing and build tests" ON)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(KinematicsTests
        tests/TestKinematics.cpp
    )

    target_link_libraries(KinematicsTests
        PRIVATE
            Kinematics
            GTest::gtest
    )

    include(GoogleTest)
    gtest_discover_tests(KinematicsTests)
endif()