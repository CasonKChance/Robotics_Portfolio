add_library(simulator)

target_sources(simulator
    PRIVATE
        Robot.cpp
        Simulator.cpp
        
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_SOURCE_DIR}/include
        FILES
            ${CMAKE_SOURCE_DIR}/include/simulator/Pose.h
            ${CMAKE_SOURCE_DIR}/include/simulator/VelocityCommand.h
            ${CMAKE_SOURCE_DIR}/include/simulator/Robot.h
            ${CMAKE_SOURCE_DIR}/include/simulator/Simulator.h
)

add_executable(robot_simulator Main.cpp)

target_link_libraries(robot_simulator
    PRIVATE 
        simulator
)