#include <simulator/Simulator.h>

#include <cmath>
#include <stdexcept>
#include <string>

/* Public Member Functions */

Simulator::Simulator(Robot& robot, double timeStep) :
    robot_{ robot },
    timeStep_{ timeStep },
    currentTime_{ 0.0 }
{
    if (timeStep_ <= 0.0) {
        throw std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
    }
}

void Simulator::step() {
    robot_.update(timeStep_);
    currentTime_ += timeStep_;
}

void Simulator::step(double timeStep) {
    if (timeStep <= 0.0) {
        throw std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
    }
    robot_.update(timeStep);
    currentTime_ += timeStep;
}

void Simulator::runFor(double duration) {
    if (duration <= 0.0) {
        return;
    }

    double remainingTime = duration;
    while (remainingTime >= timeStep_) {
        step(timeStep_);
        remainingTime -= timeStep_;
    }

    if (remainingTime > 0) {
        step(remainingTime);
    }
}