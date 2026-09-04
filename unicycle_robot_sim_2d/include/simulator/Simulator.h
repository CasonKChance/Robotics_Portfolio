#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Robot.h"

/**
 * @brief Manages simulation time and steps a target Robot through discrete kinematic updates.
 */
class Simulator {

    public:

        /**
        * @brief Constructs a Simulator bound to a specific Robot instance.
        * 
        * @param robot Reference to the robot being simulated (must remain valid for simulator lifetime).
        * @param timeStep Integration time step dt in seconds (must be > 0.0).
        */
        Simulator(
            Robot& robot,
            double timeStep
        );

        /**
        * @brief Advances the simulation by a single discrete time step (dt).
        */
        void step();

        /**
        * @brief Advances the simulation continuously for a target duration in seconds.
        * 
        * @param duration Total time span to simulate in seconds.
        */
        void runFor(double duration);

        /**
        * @brief Gets the total elapsed simulation time since initialization.
        * @return Current simulation clock time in seconds.
        */
        double getCurrentTime() const { return currentTime_; }

    private:

    Robot& robot_;              // Reference to the managed robot model.
    double timeStep_{ 0.01 };   // Fixed integration step size dt in seconds.
    double currentTime_{ 0.0 }; // Accumulated simulation clock time in seconds.
};

#endif