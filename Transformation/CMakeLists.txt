cmake_minimum_required(VERSION 3.20)
project(
    Transformation
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

include(CTest)

add_library(Rotation src/Rotation.cpp)
target_include_directories(Rotation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(Rotation
    PUBLIC
        Eigen3::Eigen
)

add_library(Transformation src/Transformation.cpp)
target_include_directories(Transformation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(Transformation
    PUBLIC
        Eigen3::Eigen
)

option(BUILD_TESTING "Enable testing and build tests" ON)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)

    add_executable(TransformationTests
        tests/TestTransformation.cpp
    )

    target_link_libraries(TransformationTests
        PRIVATE
            Transformation
            Rotation
            GTest::gtest
    )

    include(GoogleTest)
    gtest_discover_tests(TransformationTests)
endif()