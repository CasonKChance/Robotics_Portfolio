#ifndef TWIST_H
#define TWIST_H

/**
 * @brief Represents a 2D twist [v, ω]ᵀ for a differential drive robot.
 *
 * - linearVelocity (v): Forward velocity along the robot's local x-axis (m/s).
 * - angularVelocity (ω): Rotational velocity counter-clockwise around the z-axis (rad/s).
 */
struct Twist
{
  double linearVelocity {0.0};
  double angularVelocity {0.0};
};

#endif
