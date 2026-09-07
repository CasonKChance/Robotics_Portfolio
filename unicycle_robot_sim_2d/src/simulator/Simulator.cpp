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
    currentTime_{ 0.0 },
    status_{ SimulationStatus::Running }
{
    if (timeStep_ <= 0.0) {
        throw std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
    }

    // Evaluate initial spatial condition without throwing an exception
    status_ = checkCollision();
}

void Simulator::runFor(double duration) {
    // Immediate return if duration is invalid or simulation already hit a terminal state
    if (duration <= 0.0 || status_ != SimulationStatus::Running) {
        return;
    }

    double remainingTime = duration;
    while (remainingTime >= timeStep_) {
        step(timeStep_);
        remainingTime -= timeStep_;

        status_ = checkCollision();
        if (status_ != SimulationStatus::Running) {
            break;
        }
    }

    if (remainingTime > 0 && status_ == SimulationStatus::Running) {
        step(remainingTime);

        status_ = checkCollision();
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

SimulationStatus Simulator::checkCollision() const {
    const Pose& pose = robot_.getPose();

    if (!world_.isWithinBounds(pose.x, pose.y)) {
        return SimulationStatus::OutOfBounds;
    }

    if (world_.isCollisionWithObstacle(pose.x, pose.y)) {
        return SimulationStatus::ObstacleCollision;
    }

    if (world_.isCollisionWithGoal(pose.x, pose.y)) {
        return SimulationStatus::GoalReached;
    }

    return SimulationStatus::Running;
}

std::string Simulator::prepareLogPath() const {
    std::filesystem::path logPath = std::filesystem::path(PROJECT_BINARY_DIR) / "output" / "SimulatorDataLog.csv";
    std::filesystem::create_directories(logPath.parent_path());
    return logPath.string();
}
