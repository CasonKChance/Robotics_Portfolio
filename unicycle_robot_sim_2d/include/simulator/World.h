#ifndef WORLD_H
#define WORLD_H

#include "Obstacle.h"

#include <optional>
#include <vector>

/**
 * @brief Manages the spatial environment, map boundaries, obstacles, and goal conditions.
 */
class World {
    public:

        /**
         * @brief Constructs a bounded world with optional obstacles and goal.
         * 
         * @param maxX Maximum X boundary.
         * @param maxY Maximum Y boundary.
         * @param obstacles Vector of obstacles (defaults to empty).
         * @param goal Optional goal region (defaults to std::nullopt / no goal).
         */
        World(int maxX, int maxY, std::vector<Obstacle> obstacles={}, std::optional<Obstacle> goal=std::nullopt);

        /**
         * @brief Evaluates whether a given position intersects with any environmental obstacle.
         * @param x X-coordinate to test.
         * @param y Y-coordinate to test.
         * @return True if colliding with any obstacle; false otherwise.
         */
        bool isCollisionWithObstacle(double x, double y) const;

        /**
         * @brief Checks if a given position lies within the goal region.
         * @param x X-coordinate to test.
         * @param y Y-coordinate to test.
         * @return True if inside the goal radius; false otherwise.
         */
        bool isCollisionWithGoal(double x, double y) const;

        /**
         * @brief Verifies whether a position is within the valid map boundaries.
         * @param x X-coordinate to test.
         * @param y Y-coordinate to test.
         * @return True if position lies within [0, maxX] and [0, maxY]; false otherwise.
         */
        bool isWithinBounds(double x, double y) const;

    private:

        int maxX_{ 10 };                                    // Map upper bound along the X-axis
        int maxY_{ 10 };                                    // Map upper bound along the Y-axis
        std::vector<Obstacle> obstacles_{};                 // List of active circular obstacles
        std::optional<Obstacle> goal_{ std::nullopt };      // Optional target goal region
};

#endif