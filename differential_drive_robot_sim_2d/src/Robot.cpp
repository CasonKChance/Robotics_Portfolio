#include <simulator/Robot.h>
#include <simulator/Pose.h>
#include <simulator/VelocityCommand.h>

#include <cmath>
#include <iostream>

/* Public Member Functions */

Robot::Robot(const Pose& initialPose) :
    pose_{ initialPose },
    velocityCommand_{ VelocityCommand{ 0.0, 0.0 } }
{
    // Ensure initial heading is properly normalized to [-π, π]
    pose_.theta = normalizeAngle(pose_.theta);
}

void Robot::setVelocityCommand(const VelocityCommand& command) {
    velocityCommand_.linearVelocity = command.linearVelocity;
    velocityCommand_.angularVelocity = command.angularVelocity;
}

void Robot::update(double dt) {
    if (dt <= 0.0) {
        return;
    }

    double v = velocityCommand_.linearVelocity;
    double omega = velocityCommand_.angularVelocity;

    pose_.x += v * std::cos(pose_.theta) * dt;
    pose_.y += v * std::sin(pose_.theta) * dt;
    pose_.theta = normalizeAngle(pose_.theta + (omega * dt));
}

void Robot::printPose() const {
    std::cout << "x: " << pose_.x << "\n"
              << "y: " << pose_.y << "\n"
              << "theta: " << pose_.theta << "\n";
}

/* Private Member Functions */

double Robot::normalizeAngle(double angle) const {
    // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
    return std::atan2(std::sin(angle), std::cos(angle));
}
