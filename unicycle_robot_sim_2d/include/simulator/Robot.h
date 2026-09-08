#ifndef ROBOT_H
#define ROBOT_H

#include "Pose.h"
#include "VelocityCommand.h"

#include <numbers>

/**
 * @brief Simulates a unicycle-style mobile robot.
 * 
 * Tracks the current 2D pose [x, y, θ]ᵀ and updates state using discrete 
 * Forward Euler numerical integration given a linear and angular velocity.
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

        /**
         * @brief Gets the current velocity command of the robot.
         * @return Immutable reference to the robot's current velocity command.
         */
        const VelocityCommand& getVelocityCommand() const { return velocityCommand_; }

        /**
         * @brief Gets the current actual velocity of the robot.
         * @return Imutable reference to the robot's current actual velocity.
         */
        const VelocityCommand& getActualVelocity() const { return actualVelocity_; }

        /**
         * @brief Gets the maximum linear velocity of the robot.
         * @return The maximum linear velocity of the robot.
         */
        double getMaximumLinearVelocity() const { return maximumLinearVelocity_; }

        /**
         * @brief Gets the maximum angular velocity of the robot.
         * @return The maximum angular velocity of the robot.
         */
        double getMaximumAngularVelocity() const { return maximumAngularVelocity_; }

    private:

        Pose pose_;                                                  // Current state [x, y, θ]ᵀ in the World frame.
        VelocityCommand velocityCommand_;                            // Current active velocity command [v, ω]ᵀ.
        VelocityCommand actualVelocity_;                             // Actual velocity ([v, ω]ᵀ) of the robot, clamped from the command.
        const double maximumLinearVelocity_{ 5.0 };                  // Maximum linear velocity of robot
        const double maximumAngularVelocity_{ std::numbers::pi };    // Maximum angular velocity of robot
        const double linearAcceleration_{ 2.5 };                     // Linear acceleration of robot
        const double angularAcceleration_{ std::numbers::pi / 2 };   // Angular acceleration of robot

        /**
         * @brief Normalizes an angle into the range [-π, π] radians.
         * @param angle Angle in radians.
         * @return Equivalent angle wrapped within [-π, π].
         */
        double normalizeAngle(double angle) const;

        /**
         * @brief Updates the robot's actual velocity ([v, ω]ᵀ) taking acceleration into account.
         */
        void updateActualVelocity(double dt);
};

#endif