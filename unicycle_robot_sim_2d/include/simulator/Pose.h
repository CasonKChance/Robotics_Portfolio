#ifndef POSE_H
#define POSE_H

/**
 * @brief Represents the 2D spatial pose vector [x, y, θ]ᵀ in the World Frame.
 * 
 * - x: Position along the world X-axis (meters).
 * - y: Position along the world Y-axis (meters).
 * - theta (θ): Heading relative to the world X-axis (radians), normalized to [-π, π].
 * 
 * Coordinate Convention (ISO 8855 / Standard ROS):
 * - World Frame: +X = Forward/East, +Y = Left/North.
 * - Rotation: Positive θ is Counter-Clockwise (CCW), Negative θ is Clockwise (CW).
 * 
 * Example:
 * A pose of [1.0, 2.0, -π/2]ᵀ represents a robot positioned 1.0m along the world X-axis,
 * 2.0m along the world Y-axis, rotated 90° clockwise (-π/2 rad) relative to the X-axis.
 */

struct Pose{
    double x{ 0.0 };
    double y{ 0.0 };
    double theta{ 0.0 };
};

#endif