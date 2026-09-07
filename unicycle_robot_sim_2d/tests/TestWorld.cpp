#include <simulator/World.h>

#include <iostream>
#include <cassert>

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

void testBoundaryChecking() {
    World world(10, 10);

    // Test positions within bounds
    assert(world.isWithinBounds(0.0, 0.0));
    assert(world.isWithinBounds(5.0, 5.0));
    assert(world.isWithinBounds(9.9, 9.9));

    // Test positions outside bounds
    assert(!world.isWithinBounds(-1.0, 5.0)); // Negative X
    assert(!world.isWithinBounds(5.0, -1.0)); // Negative Y
    assert(!world.isWithinBounds(10.0, 5.0)); // X equal to maxX
    assert(!world.isWithinBounds(5.0, 10.0)); // Y equal to maxY

    std::cout << "[PASS] testBoundaryChecking\n";
}

int main() {
    testGoalCollisionChecking();
    testObstacleCollisionChecking();
    testBoundaryChecking();

    std::cout << "\nAll unit tests passed successfully!\n";
    return 0;
}