#include <simulator/Robot.h>
#include <simulator/World.h>
#include <simulator/Simulator.h>

#include <numbers>

int main() {
    Robot robot({0.0, 0.0, 0.0});
    World  world(10, 10, {Obstacle{2.5, 2.5, 0.5}}, Obstacle{5.0, 5.0, 0.5});

    Simulator simulator = Simulator(robot, world, 0.01);

    for (int i{ 0 }; i < 4; ++i) {
        robot.setVelocityCommand({
            .linearVelocity = 1.0,
            .angularVelocity = 0.0
        });

        simulator.runFor(10);

        robot.setVelocityCommand({
            .linearVelocity = 0.0,
            .angularVelocity = std::numbers::pi / 2
        });

        simulator.runFor(1);
    }
}