#ifndef SIMULATOR_DATA_LOGGER_H
#define SIMULATOR_DATA_LOGGER_H

#include <simulator/Pose.h>
#include <simulator/VelocityCommand.h>

#include <string>
#include <fstream>

/**
 * @brief Handles file-based telemetry output for simulator state data.
 * 
 * Automatically initializes a CSV file with a standard header row upon construction
 * and manages file handle lifecycles cleanly via RAII.
 */
class SimulatorDataLogger {
    public:

        /**
         * @brief Constructs a new logger instance and opens the output file stream.
         * 
         * Creates or overwrites the target file, sets default floating-point precision,
         * and writes the initial CSV header.
         * 
         * @param filePath Relative or absolute path to the destination log file.
         * @throws std::runtime_error If the file cannot be opened for writing.
         */
        SimulatorDataLogger(const std::string& filePath);

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
        void logData(const Pose& pose, const VelocityCommand& command, double currentTime);

    private:

        std::ofstream outputFile_;      // Output stream writing telemetry data to disk.

};

#endif