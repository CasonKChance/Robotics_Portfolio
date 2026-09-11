add_subdirectory(simulator)

add_executable(robot_simulator Main.cpp)

target_link_libraries(robot_simulator
    PRIVATE 
        simulator
)

target_include_directories(robot_simulator
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_definitions(robot_simulator
    PRIVATE
        PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)