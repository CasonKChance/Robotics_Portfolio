#include <simulator/World.h>

#include <iostream>
#include <cassert>

/**
 * @brief Tests collision detection logic against the goal region.
 * 
 * Verifies that coordinates within the goal's boundary radius are correctly 
 * flagged as collisions, while points outside the radius are recognized as clear.
 */
void testGoalCollisionChecking() {
    // Create a world with a goal at (5, 5) with radius 1.0
    World world(10, 10, {}, Obstacle{5.0, 5.0, 1.0});

    // Test positions inside the goal
    assert(world.isCollisionWithGoal(5.0, 5.0)); // Center of the goal
    assert(world.isCollisionWithGoal(5.5, 5.0)); // Inside the goal radius
    assert(world.isCollisionWithGoal(4.0, 5.0)); // Inside the goal radius

    // Test positions outside the goal
    assert(!world.isCollisionWithGoal(6.1, 5.0)); // Outside the goal radius
    assert(!world.isCollisionWithGoal(5.0, 6.1)); // Outside the goal radius
    assert(!world.isCollisionWithGoal(3.9, 5.0)); // Outside the goal radius

    std::cout << "[PASS] testGoalCollisionChecking\n";
}

/**
 * @brief Tests collision detection logic against circular obstacles.
 * 
 * Ensures point-obstacle collision checks accurately distinguish between 
 * interior/boundary collision points and safe exterior points.
 */
void testObstacleCollisionChecking() {
    // Create a world with an obstacle at (3, 3) with radius 1.0
    World world(10, 10, {Obstacle{3.0, 3.0, 1.0}});

    // Test positions inside the obstacle
    assert(world.isCollisionWithObstacle(3.0, 3.0)); // Center of the obstacle
    assert(world.isCollisionWithObstacle(3.5, 3.0)); // Inside the obstacle radius
    assert(world.isCollisionWithObstacle(2.0, 3.0)); // Inside the obstacle radius

    // Test positions outside the obstacle
    assert(!world.isCollisionWithObstacle(4.1, 3.0)); // Outside the obstacle radius
    assert(!world.isCollisionWithObstacle(3.0, 4.1)); // Outside the obstacle radius
    assert(!world.isCollisionWithObstacle(1.9, 3.0)); // Outside the obstacle radius

    std::cout << "[PASS] testObstacleCollisionChecking\n";
}

/**
 * @brief Tests world boundary limits (0 <= x < maxX, 0 <= y < maxY).
 * 
 * Verifies valid internal coordinates as well as edge cases including negative 
 * coordinates and exact boundary limit thresholds.
 */
void testBoundaryChecking() {
    World world(10, 10);

    // Test positions within bounds
    assert(world.isWithinBounds(0.0, 0.0));
    assert(world.isWithinBounds(5.0, 5.0));
    assert(world.isWithinBounds(9.9, 9.9));

    // Test positions outside bounds
    assert(!world.isWithinBounds(-1.0, 5.0)); // Negative X
    assert(!world.isWithinBounds(5.0, -1.0)); // Negative Y
    assert(!world.isWithinBounds(10.1, 5.0)); // X greater than maxX
    assert(!world.isWithinBounds(5.0, 10.1)); // Y greater than maxY

    std::cout << "[PASS] testBoundaryChecking\n";
}

int main() {
    testGoalCollisionChecking();
    testObstacleCollisionChecking();
    testBoundaryChecking();

    std::cout << "\nAll unit tests passed successfully!\n";
    return 0;
}