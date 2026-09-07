#include <simulator/World.h>

#include <stdexcept>

/* Public Member Functions */

World::World(int maxX, int maxY, std::vector<Obstacle> obstacles, std::optional<Obstacle> goal) :
    maxX_{ maxX },
    maxY_{ maxY },
    obstacles_{ std::move(obstacles) },
    goal_{ std::move(goal) }
{
    // Ensure the goal and obstacles are within the bounds of the world
    if (goal_ && !isWithinBounds(goal_->x + goal_->radius, goal_->y + goal_->radius)) {
        throw std::invalid_argument("Goal position is out of bounds.");
    }

    for (const auto& obstacle : obstacles_) {
        if (!isWithinBounds(obstacle.x + obstacle.radius, obstacle.y + obstacle.radius)) {
            throw std::invalid_argument("Obstacle position is out of bounds.");
        }
    }
}

bool World::isCollisionWithObstacle(double x, double y) const {
    for (const auto& obstacle : obstacles_) {
        if (obstacle.isCollision(x, y)) {
            return true;
        }
    }

    return false;
}

bool World::isCollisionWithGoal(double x, double y) const {
    if (!goal_) {
        return false;
    }

    return goal_->isCollision(x, y);
}

bool World::isWithinBounds(double x, double y) const {
    return x >= 0 && x < maxX_ 
        && y >= 0 && y < maxY_;
}