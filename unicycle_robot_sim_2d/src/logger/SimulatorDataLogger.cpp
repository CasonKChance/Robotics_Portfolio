#include <logger/SimulatorDataLogger.h>
#include <simulator/Pose.h>
#include <simulator/VelocityCommand.h>

#include <fstream>
#include <string>
#include <stdexcept>

/* Public Member Functions */

SimulatorDataLogger::SimulatorDataLogger(const std::string& filePath) :
    outputFile_(filePath, std::ios::out)
{
    // Set default floating-point formatting precision for logged coordinates
    outputFile_.precision(6);

    if (!outputFile_.is_open()) {
        throw std::runtime_error("Error: Could not open log file: " + filePath);
    }

    // Write initial CSV column header
    outputFile_ << "time,x,y,theta,linear_velocity,angular_velocity\n";
}

SimulatorDataLogger::~SimulatorDataLogger() {
    if (outputFile_.is_open()) {
        outputFile_.close();
    }
}

void SimulatorDataLogger::logData(const Pose& pose, const VelocityCommand& command, double currentTime) {
    if (outputFile_.is_open()) {
        outputFile_ << currentTime << "," 
                    << pose.x << "," 
                    << pose.y << "," 
                    << pose.theta << "," 
                    << command.linearVelocity << "," 
                    << command.angularVelocity << "\n";
    } else {
        throw std::runtime_error("Error: Log file is not open for writing.");
    }
}