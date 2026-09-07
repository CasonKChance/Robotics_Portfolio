#include <simulator/Obstacle.h>

Obstacle::Obstacle(double x, double y, double radius) : 
    x{x}, 
    y{y}, 
    radius{radius} 
{
    // No-op
}

bool Obstacle::isCollision(double posX, double posY) const {
    // Calculate squared Euclidean distance
    double dx{ x - posX };
    double dy{ y - posY };
    double distanceSquared{ dx * dx + dy * dy };

    return distanceSquared < radius * radius;
}
