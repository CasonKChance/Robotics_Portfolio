#include <simulator/Robot.h>

#include <cmath>
#include <iostream>
#include <algorithm>

/* Public Member Functions */

Robot::Robot(const Pose& initialPose) :
    pose_{ initialPose },
    velocityCommand_{ VelocityCommand{ 0.0, 0.0 } },
    actualVelocity_{ VelocityCommand{ 0.0, 0.0 } }
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

    updateActualVelocity(dt);

    double v = actualVelocity_.linearVelocity;
    double omega = actualVelocity_.angularVelocity;

    pose_.x += v * std::cos(pose_.theta) * dt;
    pose_.y += v * std::sin(pose_.theta) * dt;
    pose_.theta = normalizeAngle(pose_.theta + (omega * dt));
}

/* Private Member Functions */

double Robot::normalizeAngle(double angle) const {
    // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
    return std::atan2(std::sin(angle), std::cos(angle));
}

void Robot::updateActualVelocity(double dt) {
    double linear_error = velocityCommand_.linearVelocity - actualVelocity_.linearVelocity;
    if (std::abs(linear_error) > 0.001) {
        double sign = (linear_error > 0.0) ? 1.0 : -1.0;
        actualVelocity_.linearVelocity += sign * linearAcceleration_ * dt;
        
        if ((sign > 0.0 && actualVelocity_.linearVelocity > velocityCommand_.linearVelocity) ||
            (sign < 0.0 && actualVelocity_.linearVelocity < velocityCommand_.linearVelocity)) {
            actualVelocity_.linearVelocity = velocityCommand_.linearVelocity;
        }
    }

    double angular_error = velocityCommand_.angularVelocity - actualVelocity_.angularVelocity;
    if (std::abs(angular_error) > 0.001) {
        double sign = (angular_error > 0.0) ? 1.0 : -1.0;
        actualVelocity_.angularVelocity += sign * angularAcceleration_ * dt;
        
        if ((sign > 0.0 && actualVelocity_.angularVelocity > velocityCommand_.angularVelocity) ||
            (sign < 0.0 && actualVelocity_.angularVelocity < velocityCommand_.angularVelocity)) {
            actualVelocity_.angularVelocity = velocityCommand_.angularVelocity;
        }
    }
    
    actualVelocity_.linearVelocity  = std::clamp(actualVelocity_.linearVelocity, -maximumLinearVelocity_, maximumLinearVelocity_);
    actualVelocity_.angularVelocity = std::clamp(actualVelocity_.angularVelocity, -maximumAngularVelocity_, maximumAngularVelocity_);
}

