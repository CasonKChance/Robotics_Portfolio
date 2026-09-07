#include <simulator/SimulatorDataLogger.h>

#include <fstream>
#include <string>
#include <stdexcept>
#include <vector>
#include <optional>

static constexpr int kDefaultPrecision = 6;  // Default floating-point precision for logged coordinates

/* Public Member Functions */

SimulatorDataLogger::SimulatorDataLogger(const std::string& robotDataFilePath, const std::string& worldDataFilePath) :
    robotDataOutputFile_(robotDataFilePath, std::ios::out),
    worldDataOutputFile_(worldDataFilePath, std::ios::out)
{
    // Set default floating-point formatting precision for logged coordinates
    robotDataOutputFile_.precision(kDefaultPrecision);
    worldDataOutputFile_.precision(kDefaultPrecision);

    if (!robotDataOutputFile_.is_open()) {
        throw std::runtime_error("Error: Could not open robot data log file: " + robotDataFilePath);
    }
    if (!worldDataOutputFile_.is_open()) {
        throw std::runtime_error("Error: Could not open world data log file: " + worldDataFilePath);
    }

    // Write initial CSV column headers
    robotDataOutputFile_ << "time,x,y,theta,linear_velocity,angular_velocity\n";
    worldDataOutputFile_ << "x,y,radius,is_goal,is_world_bounds\n";
}

SimulatorDataLogger::~SimulatorDataLogger() {
    if (robotDataOutputFile_.is_open()) {
        robotDataOutputFile_.close();
    }
    if (worldDataOutputFile_.is_open()) {
        worldDataOutputFile_.close();
    }
}

void SimulatorDataLogger::logRobotData(const Pose& pose, const VelocityCommand& command, double currentTime) {
    if (robotDataOutputFile_.is_open()) {
        robotDataOutputFile_ << currentTime << "," 
                             << pose.x << "," 
                             << pose.y << "," 
                             << pose.theta << "," 
                             << command.linearVelocity << "," 
                             << command.angularVelocity << "\n";
    } else {
        throw std::runtime_error("Error: Robot data log file is not open for writing.");
    }
}

void SimulatorDataLogger::logWorldData(const World& world) {
    if (worldDataOutputFile_.is_open()) {
        // Log world bounds as a special entry
        worldDataOutputFile_ << world.getMaxX() << "," 
                             << world.getMaxY() << ",0,0,1\n";

        for (const auto& obstacle : world.getObstacles()) {
            worldDataOutputFile_ << obstacle.x << "," 
                                 << obstacle.y << "," 
                                 << obstacle.radius << ",0,0\n";
        }
        if (world.getGoal().has_value()) {
            worldDataOutputFile_ << world.getGoal().value().x << "," 
                                  << world.getGoal().value().y << "," 
                                  << world.getGoal().value().radius << ",1,0\n";
        }
    } else {
        throw std::runtime_error("Error: World data log file is not open for writing.");
    }
}