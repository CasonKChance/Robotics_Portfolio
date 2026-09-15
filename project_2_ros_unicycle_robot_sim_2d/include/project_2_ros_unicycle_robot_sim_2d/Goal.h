#ifndef GOAL_H
#define GOAL_H

/**
 * @brief Represents a circular goal region within the simulation space.
 */
struct Goal
{
  double x {0.0};       // Center X-coordinate in meters
  double y {0.0};       // Center Y-coordinate in meters
  double radius {0.0};  // Circular boundary radius in meters

  /**
   * @brief Constructs a circular goal with specified center coordinates and radius.
   * @param x Center position along the X-axis.
   * @param y Center position along the Y-axis.
   * @param radius Spatial clearance radius around the center point.
   */
  Goal(double x, double y, double radius);

  /**
   * @brief Checks if a 2D point overlaps with the circular boundary.
   * @param posX Query position along the X-axis.
   * @param posY Query position along the Y-axis.
   * @return True if the position lies strictly inside the goal radius; false otherwise.
   */
  bool isCollision(double posX, double posY) const;
};

#endif
