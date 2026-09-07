#ifndef SIMULATOR_DATA_LOGGER_H
#define SIMULATOR_DATA_LOGGER_H

#include <simulator/Pose.h>
#include <simulator/VelocityCommand.h>
#include <simulator/World.h>

#include <string>
#include <fstream>
#include <vector>
#include <optional>

/**
 * @brief Handles file-based telemetry output for simulator state data.
 * 
 * Automatically initializes CSV files with a standard header row upon construction
 * and manages file handle lifecycles cleanly via RAII.
 */
class SimulatorDataLogger {
    public:

        /**
         * @brief Constructs a new logger instance and opens the output file streams.
         * 
         * Creates or overwrites the target file, sets default floating-point precision,
         * and writes the initial CSV header.
         * 
         * @param robotDataFilePath Relative or absolute path to the destination log file for robot data.
         * @param worldDataFilePath Relative or absolute path to the destination log file for world data.
         * @throws std::runtime_error If the files cannot be opened for writing.
         */
        SimulatorDataLogger(const std::string& robotDataFilePath, const std::string& worldDataFilePath);

        /**
         * @brief Destructor. Ensures the underlying output file stream is safely closed.
         */
        ~SimulatorDataLogger();

        /**
         * @brief Writes a structured simulation state snapshot to the CSV file.
         * 
         * Formats the entry as: time, x, y, theta, linear_velocity, angular_velocity
         * 
         * @param pose Current 2D spatial pose [x, y, θ]ᵀ of the robot.
         * @param command Current active velocity command [v, ω]ᵀ applied to the robot.
         * @param currentTime Current total elapsed simulation time in seconds.
         * @throws std::runtime_error If called while the file stream is not open.
         */
        void logRobotData(const Pose& pose, const VelocityCommand& command, double currentTime);

        /**
         * @brief Logs the current world state, including obstacles and goal region.
         * 
         * Each obstacle is logged as: x, y, radius
         * The goal region (if present) is logged similarly.
         * 
         * @param obstacles Vector of active obstacles in the world.
         * @param goal Optional goal region; if std::nullopt, no goal is logged.
         */
        void logWorldData(const World& world);

    private:

        std::ofstream robotDataOutputFile_;      // Output stream writing telemetry data to disk.
        std::ofstream worldDataOutputFile_;      // Output stream writing world state data to disk.

};

#endif