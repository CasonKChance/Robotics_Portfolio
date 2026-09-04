#include <simulator/Robot.h>
#include <simulator/Pose.h>
#include <simulator/Simulator.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>

// Helper to safely compare floating-point values
constexpr double EPSILON = 1e-6;

double normalizeAngle(double angle) {
    // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool isPositionClose(double a, double b, double tol = EPSILON) {
    return std::abs(a - b) < tol;
}

bool isAngleClose(double a, double b, double tol = EPSILON) {
    return std::abs(normalizeAngle(a - b)) < tol;
}

bool isPoseClose(const Pose& p1, const Pose& p2, double tol = EPSILON) {
    return isPositionClose(p1.x, p2.x, tol) && 
           isPositionClose(p1.y, p2.y, tol) && 
           (isAngleClose(p1.theta, p2.theta, tol));
}
/**
 * @brief Tests that a stationary robot remains at the origin when given zero velocity.
 */
void testStationary() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    robot.setVelocityCommand({
        .linearVelocity = 0.0,
        .angularVelocity = 0.0
    });

    for (int i = 0; i < 10; ++i) {
        simulator.runFor(1.0);
    }

    assert(isPoseClose(robot.getPose(), Pose{0.0, 0.0, 0.0}));
    std::cout << "[PASS] testStationary\n";
}

/**
 * @brief Tests linear movement along the X-axis (forward).
 */
void testMoveForward() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    robot.setVelocityCommand({
        .linearVelocity = 1.0,
        .angularVelocity = 0.0
    });

    for (int i = 0; i < 10; ++i) {
        simulator.runFor(1.0);
    }

    assert(isPoseClose(robot.getPose(), Pose{10.0, 0.0, 0.0}));
    std::cout << "[PASS] testMoveForward\n";
}

/**
 * @brief Tests in-place rotation and angle wrapping [-π, π].
 */
void testRotateInPlace() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    // Rotate at π/2 rad/s for 10 seconds -> 5π radians total.
    // 5π wraps around to π (or -π) in normalized range.
    robot.setVelocityCommand({
        .linearVelocity = 0.0,
        .angularVelocity = (std::numbers::pi / 2.0)
    });

    for (int i = 0; i < 10; ++i) {
        simulator.runFor(1.0);
    }

    // Checking against normalized result (±π)
    assert(isPoseClose(robot.getPose(), Pose{0.0, 0.0, std::numbers::pi}));
    std::cout << "[PASS] testRotateInPlace\n";
}

/**
 * @brief Tests circular motion discretization drift.
 * Note: Discrete Forward Euler over large dt=1.0s creates accumulated radial error.
 * Using smaller dt yields closer convergence.
 */
void testCircularMotion() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    robot.setVelocityCommand({
        .linearVelocity = 1.0,
        .angularVelocity = 1.0,
    });

    // Simulate for one full circular trajectory (2π radians) with small dt=0.01s
    const double dt = 0.01;
    int steps = static_cast<int>((2 * std::numbers::pi) / dt);
    for (int i = 0; i < steps; ++i) {
        simulator.runFor(dt);
    }

    // With dt=0.01s, numerical drift is small (~0.03m tolerance needed)
    assert(isPoseClose(robot.getPose(), Pose{0.0, 0.0, 0.0}, 0.05));
    std::cout << "[PASS] testCircularMotion\n";
}

/**
 * @brief Tests execution of a 4-leg square trajectory returning to starting pose.
 */
void testMoveInASquare() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    for (int i = 0; i < 4; ++i) {
        // Move forward 10m
        robot.setVelocityCommand({
            .linearVelocity = 1.0,
            .angularVelocity = 0.0
        });
        simulator.runFor(10.0);

        // Turn right 90 deg (-π/2 rad)
        robot.setVelocityCommand({
            .linearVelocity = 0.0,
            .angularVelocity = -(std::numbers::pi / 2.0)
        });
        simulator.runFor(1.0);
    }

    assert(isPoseClose(robot.getPose(), Pose{0.0, 0.0, 0.0}));
    std::cout << "[PASS] testMoveInASquare\n";
}

/**
 * @brief Tests that the simulator correctly accumulates time and steps the robot.
 */
void testTime() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    Simulator simulator(robot, 0.01);

    assert(isClose(simulator.getCurrentTime(), 0.0));

    simulator.runFor(1.0);
    assert(isClose(simulator.getCurrentTime(), 1.0));

    simulator.runFor(2.5);
    assert(isClose(simulator.getCurrentTime(), 3.5));

    simulator.runFor(0.5);
    assert(isClose(simulator.getCurrentTime(), 4.0));

    std::cout << "[PASS] testTime\n";
}

int main() {
    testStationary();
    testMoveForward();
    testRotateInPlace();
    testCircularMotion();
    testMoveInASquare();
    testTime();

    std::cout << "\nAll unit tests passed successfully!\n";
    return 0;
}