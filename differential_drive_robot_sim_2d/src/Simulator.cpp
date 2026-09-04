#include <simulator/Simulator.h>

#include <cmath>

/* Public Member Functions */

Simulator::Simulator(Robot& robot, double timeStep) :
    robot_{ robot },
    timeStep_{ timeStep > 0.0 ? timeStep : 0.01 },
    currentTime_{ 0.0 }
{
    // No-op
}

void Simulator::step() {
    robot_.update(timeStep_);
    currentTime_ += timeStep_;
}

void Simulator::runFor(double duration) {
    if (duration <= 0.0) {
        return;
    }

    const int totalSteps = static_cast<int>(std::round(duration / timeStep_));
    for (int i{ 0 }; i < totalSteps; ++i) {
        step();
    }
}

void Simulator::driveSquare() {
    
}