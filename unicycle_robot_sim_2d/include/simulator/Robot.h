#ifndef ROBOT_H
#define ROBOT_H

#include "Pose.h"
#include "VelocityCommand.h"

/**
 * @brief Simulates a unicycle-style mobile robot.
 * 
 * Tracks the current 2D pose [x, y, θ]ᵀ and updates state using discrete 
 * Forward Euler numerical integration given a linear and angular velocity command.
 */

class Robot {
    public:

        /**
        * @brief Constructs a Robot instance with a given initial pose.
        * @param initialPose Initial spatial configuration [x, y, θ]ᵀ in the World frame.
        */
        Robot(const Pose& initial_pose);

        /**
        * @brief Sets the target velocity command [v, ω]ᵀ.
        * @param command Linear (m/s) and angular (rad/s) velocity inputs.
        */
        void setVelocityCommand(const VelocityCommand& command);

        /**
        * @brief Advances the robot's state over a time step dt using Forward Euler integration.
        * @param dt Time step duration in seconds (must be positive).
        */
        void update(double dt);

        /**
        * @brief Gets the current pose of the robot in the World frame.
        * @return Immutable reference to the robot's current pose.
        */
        const Pose& getPose() const { return pose_; }

    private:

        Pose pose_;                       // Current state [x, y, θ]ᵀ in the World frame.
        VelocityCommand velocityCommand_; // Current active velocity command [v, ω]ᵀ.

        /**
        * @brief Normalizes an angle into the range [-π, π] radians.
        * @param angle Angle in radians.
        * @return Equivalent angle wrapped within [-π, π].
        */
        double normalizeAngle(double angle) const;
};

#endif