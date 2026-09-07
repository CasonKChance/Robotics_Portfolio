#include <simulator/Robot.h>

#include <cmath>
#include <iostream>
#include <algorithm>

static constexpr double kMaximumLinearVelocity = 2.0; // Maximum linear velocity (m/s)
static constexpr double kMaximumAngularVelocity = 1.0; // Maximum angular velocity (rad/s)

/* Public Member Functions */

Robot::Robot(const Pose& initialPose) :
    pose_{ initialPose },
    velocityCommand_{ VelocityCommand{ 0.0, 0.0 } }
{
    // Clamp linear velocity and angular velocity to their respective maximums
    if (velocityCommand_.linearVelocity < -kMaximumLinearVelocity || velocityCommand_.linearVelocity > kMaximumLinearVelocity) {
        velocityCommand_.linearVelocity = std::clamp(velocityCommand_.linearVelocity, -kMaximumLinearVelocity, kMaximumLinearVelocity);
    }
    if (velocityCommand_.angularVelocity < -kMaximumAngularVelocity || velocityCommand_.angularVelocity > kMaximumAngularVelocity) {
        velocityCommand_.angularVelocity = std::clamp(velocityCommand_.angularVelocity, -kMaximumAngularVelocity, kMaximumAngularVelocity);
    }
    
    // Ensure initial heading is properly normalized to [-π, π]
    pose_.theta = normalizeAngle(pose_.theta);
}

void Robot::setVelocityCommand(const VelocityCommand& command) {
    // Clamp linear velocity and angular velocity to their respective maximums
    velocityCommand_.linearVelocity = command.linearVelocity < -kMaximumLinearVelocity || command.linearVelocity > kMaximumLinearVelocity
                                    ? std::clamp(velocityCommand_.linearVelocity, -kMaximumLinearVelocity, kMaximumLinearVelocity)
                                    : command.linearVelocity;
    velocityCommand_.angularVelocity = command.angularVelocity < -kMaximumAngularVelocity || command.linearVelocity > kMaximumAngularVelocity
                                     ? std::clamp(velocityCommand_.angularVelocity, -kMaximumAngularVelocity, kMaximumAngularVelocity)
                                     : command.angularVelocity;
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

/* Private Member Functions */

double Robot::normalizeAngle(double angle) const {
    // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
    return std::atan2(std::sin(angle), std::cos(angle));
}
