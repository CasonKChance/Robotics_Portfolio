#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Robot.h"
#include "SimulatorDataLogger.h"
#include "World.h"

/**
 * @brief Represents the current operational state or termination reason of the simulation.
 */
enum class SimulationStatus {
    Running,           // Simulation is progressing normally
    GoalReached,       // Robot successfully reached the target goal region
    ObstacleCollision, // Robot collided with an environmental obstacle
    OutOfBounds        // Robot moved outside the valid map boundaries
};

/**
 * @brief Manages simulation time, advances a Robot through discrete kinematic updates,
 *        logs telemetry data, and evaluates environmental safety checks.
 */
class Simulator {

    public:

        /**
         * @brief Constructs a Simulator instance bound to a target Robot and World.
         * 
         * @param robot Reference to the robot being simulated (must remain valid for simulator lifetime).
         * @param world Reference to the environment containing obstacles, boundaries, and goal regions.
         * @param timeStep Fixed integration step size dt in seconds (must be > 0.0).
         * 
         * @throws std::invalid_argument If timeStep is less than or equal to 0.0.
         * @throws std::runtime_error If the robot starts in an out-of-bounds or colliding state.
         */
        Simulator(
            Robot& robot,
            World& world,
            double timeStep
        );

        /**
         * @brief Advances the simulation continuously for a specified time duration.
         * 
         * Executes discrete integration steps, logging state at each step and checking
         * environmental status. Exits gracefully if a terminal state (goal, obstacle, 
         * out-of-bounds) is encountered or when duration completes.
         * 
         * @param duration Total time span to simulate in seconds (ignored if <= 0.0).
         */
        void runFor(double duration);

        /**
         * @brief Gets the total elapsed simulation time since initialization.
         * @return Current simulation clock time in seconds.
         */
        double getCurrentTime() const { return currentTime_; }

        /**
         * @brief Gets the current operational or termination status of the simulation.
         * @return Active SimulationStatus enum value.
         */
        SimulationStatus getStatus() const { return status_; }

    private:

        Robot& robot_;                                         // Reference to the managed robot model.
        World& world_;                                         // Reference to the simulation world (obstacles, boundaries, etc.).
        SimulatorDataLogger logger_;                           // Logger instance for recording simulation data.
        double timeStep_{ 0.01 };                              // Fixed integration step size dt in seconds.
        double currentTime_{ 0.0 };                            // Accumulated simulation clock time in seconds.
        SimulationStatus status_{ SimulationStatus::Running }; // Active simulation state tracker

        /**
         * @brief Advances the simulation by the default stored integration time step (timeStep_).
         */
        void step();

        /**
         * @brief Advances the simulation by a custom discrete time step (dt).
         * 
         * @param timeStep Integration step duration in seconds (must be > 0.0).
         * @throws std::invalid_argument If custom timeStep is less than or equal to 0.0.
         */
        void step(double timeStep);

        /**
         * @brief Evaluates current robot pose against world boundaries, obstacles, and goals.
         * @return The resulting SimulationStatus based on current pose overlap.
         */
        SimulationStatus checkCollision() const;

        /**
         * @brief Prepares the robot data log file path and ensures the output directory exists.   
         * @return Full path to the robot data log file as a string.
         */
        std::string prepareRobotLogPath() const;

        /**
         * @brief Prepares the world data log file path and ensures the output directory exists.   
         * @return Full path to the world data log file as a string.
         */
        std::string prepareWorldLogPath() const;

};

#endif