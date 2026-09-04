# TestRobot.cpp

add_executable(TestRobot TestRobot.cpp)

target_link_libraries(TestRobot
    PRIVATE
        simulator
)

add_test(NAME RunRobotTests COMMAND TestRobot)

# TestSimulator.cpp

add_executable(TestSimulator TestSimulator.cpp)

target_link_libraries(TestSimulator
    PRIVATE
        simulator
)

add_test(NAME RunSimulatorTests COMMAND TestSimulator)