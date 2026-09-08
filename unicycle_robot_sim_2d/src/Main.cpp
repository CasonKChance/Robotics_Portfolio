#include <simulator/Robot.h>
#include <simulator/World.h>
#include <simulator/Simulator.h>

#include <numbers>
#include <iostream>
#include <fstream>
#include <filesystem>

std::string getTrajectoryInputPath() {
    std::filesystem::path logPath = std::filesystem::path(PROJECT_SOURCE_DIR) / "input" / "Trajectory.csv";
    return logPath.string();
}

int main() {
    Robot robot({1.0, 1.0, 0.0});
    World  world(10, 10, {{2.0, 4.0, 1.0}, {4.0, 2.0, 1.0}, {6.0, 6.0, 1.75}}, Obstacle{8.0, 8.0, 1.0});

    Simulator simulator = Simulator(robot, world, 0.01);

    std::string trajectoryInputFilePath{ getTrajectoryInputPath() };
    std::ifstream trajectoryInputFile(trajectoryInputFilePath, std::ios::in);
    if (!trajectoryInputFile.is_open()) {
        throw std::runtime_error("Error: Could not open robot data log file: " + trajectoryInputFilePath);
    }

    std::string line;
    std::getline(trajectoryInputFile, line); // Skip the header line

    while (simulator.getStatus() == SimulationStatus::Running && std::getline(trajectoryInputFile, line)) {
        std::stringstream ss(line);
        std::string linearStr, angularStr, timeStr;

        if (std::getline(ss, linearStr, ',') &&
            std::getline(ss, angularStr, ',') &&
            std::getline(ss, timeStr, ',')) {

            double linearVelocity = std::stod(linearStr);
            double angularVelocity = std::stod(angularStr);
            double time = std::stod(timeStr);

            robot.setVelocityCommand({
                .linearVelocity = linearVelocity,
                .angularVelocity = angularVelocity
            });

            simulator.runFor(time);
        }
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