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

target_compile_definitions(simulator PRIVATE
    PROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}"
)
