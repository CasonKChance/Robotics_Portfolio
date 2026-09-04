#ifndef VELOCITY_COMMAND_H
#define VELOCITY_COMMAND_H

/**
 * @brief Represents a 2D velocity command vector [v, ω]ᵀ for a unicycle/differential drive robot.
 * 
 * - linearVelocity (v): Forward velocity along the robot's local x-axis (m/s).
 * - angularVelocity (ω): Rotational velocity counter-clockwise around the z-axis (rad/s).
 * 
 * Example (Forward Euler with dt = 1.0s):
 * Starting pose: [x, y, θ]ᵀ = [0.0, 0.0, 0.0]ᵀ
 * Command: [v, ω]ᵀ = [1.0, π]ᵀ
 * Resulting pose: [1.0, 0.0, π]ᵀ
 */

struct VelocityCommand {
    double linearVelocity{ 0.0 };
    double angularVelocity{ 0.0 };
};

#endif