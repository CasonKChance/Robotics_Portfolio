#include <simulator/Simulator.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <filesystem>

/* Public Member Functions */

Simulator::Simulator(Robot& robot, World& world, double timeStep) :
    robot_{ robot },
    world_{ world },
    logger_{ prepareLogPath() },
    timeStep_{ timeStep },
    currentTime_{ 0.0 }
{
    if (timeStep_ <= 0.0) {
        throw std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
    }

    // Initial collision check to ensure the robot starts in a valid state.
    checkCollision();
}

void Simulator::runFor(double duration) {
    if (duration <= 0.0) {
        return;
    }

    double remainingTime = duration;
    while (remainingTime >= timeStep_) {
        step(timeStep_);
        remainingTime -= timeStep_;

        checkCollision();
    }

    if (remainingTime > 0) {
        step(remainingTime);

        checkCollision();
    }
}

/* Private Member Functions */

void Simulator::step() {
    robot_.update(timeStep_);
    currentTime_ += timeStep_;

    logger_.logData(robot_.getPose(), robot_.getVelocityCommand(), currentTime_);
}

void Simulator::step(double timeStep) {
    if (timeStep <= 0.0) {
        throw std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
    }
    
    robot_.update(timeStep);
    currentTime_ += timeStep;

    logger_.logData(robot_.getPose(), robot_.getVelocityCommand(), currentTime_);
}

void Simulator::checkCollision() const {
    const Pose& pose = robot_.getPose();

    if (!world_.isWithinBounds(pose.x, pose.y)) {
        throw std::runtime_error("Robot is out of bounds at position (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ").");
    }

    if (world_.isCollisionWithObstacle(pose.x, pose.y)) {
        throw std::runtime_error("Robot collided with an obstacle at position (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ").");
    }

    if (world_.isCollisionWithGoal(pose.x, pose.y)) {
        throw std::runtime_error("Robot reached the goal at position (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ").");
    }
}

std::string Simulator::prepareLogPath() const {
    std::filesystem::path logPath = std::filesystem::path(PROJECT_BINARY_DIR) / "output" / "SimulatorDataLog.csv";
    std::filesystem::create_directories(logPath.parent_path());
    return logPath.string();
}
