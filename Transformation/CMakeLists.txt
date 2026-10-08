cmake_minimum_required(3.20)
project(Transformation)
set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED NO_MODULE)

add_library(Transformation src/Transformation.cpp)
target_link_libraries(Transformation
    PUBLIC
        Eigen3::Eigen
        Transformation.h
)