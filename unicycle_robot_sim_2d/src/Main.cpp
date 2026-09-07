#include <simulator/Robot.h>
#include <simulator/World.h>
#include <simulator/Simulator.h>

#include <numbers>
#include <iostream>

int main() {
    Robot robot({1.0, 1.0, 0.0});
    World  world(10, 10, {}, Obstacle{5.0, 5.0, 1.0});

    Simulator simulator = Simulator(robot, world, 0.01);

    while (simulator.getStatus() == SimulationStatus::Running) {
        robot.setVelocityCommand({
            .linearVelocity = 0.0,
            .angularVelocity = std::numbers::pi / 4.0
        });

        simulator.runFor(1.0);

        robot.setVelocityCommand({
            .linearVelocity = 1.0,
            .angularVelocity = 0.0
        });

        simulator.runFor(5);
    }

    switch(simulator.getStatus()) {
        case SimulationStatus::GoalReached:
            std::cout << "Robot reached the goal!\n";
            break;
        case SimulationStatus::ObstacleCollision:
            std::cout << "Robot collided with an obstacle!\n";
            break;
        case SimulationStatus::OutOfBounds:
            std::cout << "Robot went out of bounds!\n";
            break;
        default:
            std::cout << "Simulation ended with unknown status.\n";
    }
}