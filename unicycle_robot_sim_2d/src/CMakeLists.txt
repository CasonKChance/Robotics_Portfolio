add_subdirectory(simulator)

add_executable(robot_simulator Main.cpp)

target_link_libraries(robot_simulator
    PRIVATE 
        simulator
)