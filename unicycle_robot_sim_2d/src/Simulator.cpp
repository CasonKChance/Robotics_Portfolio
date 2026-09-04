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
        std::invalid_argument("timeStep must be a positive value. Provided: " + std::to_string(timeStep));
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

    const int totalCompleteSteps = static_cast<int>(std::round(duration / timeStep_));
    for (int i{ 0 }; i < totalCompleteSteps; ++i) {
        step();
    }

    const double finalPartialTimeStep = std::fmod(duration, timeStep_);
    step(finalPartialTimeStep);
}