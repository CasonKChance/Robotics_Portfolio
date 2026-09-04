# TestRobot.cpp

add_executable(TestRobot TestRobot.cpp)

target_link_libraries(TestRobot
    PRIVATE
        simulator
)

add_test(NAME RunRobotTests COMMAND TestRobot)