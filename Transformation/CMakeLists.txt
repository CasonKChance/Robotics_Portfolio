cmake_minimum_required(3.20)
project(
    Transformation
    VERSION 0.1.0
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(Transformation src/Transformation.cpp)

add_library(Transformation::Transformation
    ALIAS Transformation
)

target_include_directories(Transformation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(Transformation
    PUBLIC
        Eigen3::Eigen
)