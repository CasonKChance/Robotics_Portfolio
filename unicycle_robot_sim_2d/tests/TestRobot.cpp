#include <simulator/Robot.h>
#include <simulator/Pose.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>
#include <string>

// Helper to print a Pose nicely
std::string to_string(const Pose& p) {
    return "Pose[x: " + std::to_string(p.x) + 
           ", y: " + std::to_string(p.y) + 
           ", theta: " + std::to_string(p.theta) + "]";
}

// Custom assertion macro that supports dynamic string streaming
#ifndef NDEBUG
#define assert_msg(condition, stream_expression) \
    do { \
        if (!(condition)) { \
            std::cerr << "Assertion failed: (" #condition ")\n" \
                      << "Message: " << stream_expression \
                      << "\nFile: " << __FILE__ << ", Line: " << __LINE__ << "\n"; \
            std::terminate(); \
        } \
    } while (false)
#else
#define assert_msg(condition, stream_expression) do { } while (false)
#endif

// Helper to safely compare floating-point values
constexpr double EPSILON = 1e-6;

double normalizeAngle(double angle) {
    // std::atan2(sin(θ), cos(θ)) maps any angle onto [-π, π] continuously
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool isClose(double a, double b, double tol = EPSILON) {
    return std::abs(a - b) < tol;
}

bool isAngleClose(double a, double b, double tol = EPSILON) {
    return std::abs(normalizeAngle(a - b)) < tol;
}

bool isPoseClose(const Pose& p1, const Pose& p2, double tol = EPSILON) {
    return isClose(p1.x, p2.x, tol) && 
           isClose(p1.y, p2.y, tol) && 
           (isAngleClose(p1.theta, p2.theta, tol));
}

/**
 * @brief Tests that a stationary robot remains at the origin when given zero velocity.
 */
void testStationary() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    robot.setVelocityCommand({
        .linearVelocity = 0.0,
        .angularVelocity = 0.0
    });

    for (int i = 0; i < 10; ++i) {
        robot.update(1);
    }

    Pose expected{0.0, 0.0, 0.0};
    assert_msg(isPoseClose(robot.getPose(), expected), 
                "TestStationary - Expected: " << to_string(expected) << ". Actual: " << to_string(robot.getPose()));
    std::cout << "[PASS] testStationary\n";
}

/**
 * @brief Tests linear movement along the X-axis (forward).
 */
void testMoveForward() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    robot.setVelocityCommand({
        .linearVelocity = 1.0,
        .angularVelocity = 0.0
    });

    for (int i = 0; i < 10; ++i) {
        robot.update(1);
    }

    Pose expected{10.0, 0.0, 0.0};
    assert_msg(isPoseClose(robot.getPose(), expected), 
                "TestMoveForward - Expected: " << to_string(expected) << ". Actual: " << to_string(robot.getPose()));
    std::cout << "[PASS] testMoveForward\n";
}

/**
 * @brief Tests in-place rotation and angle wrapping [-π, π].
 */
void testRotateInPlace() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    // Rotate at π/2 rad/s for 10 seconds -> 5π radians total.
    // 5π wraps around to π (or -π) in normalized range.
    robot.setVelocityCommand({
        .linearVelocity = 0.0,
        .angularVelocity = (std::numbers::pi / 2.0)
    });

    for (int i = 0; i < 10; ++i) {
        robot.update(1.0);
    }

    // Checking against normalized result (±π)
    Pose expected{0.0, 0.0, std::numbers::pi};
    assert_msg(isPoseClose(robot.getPose(), expected), 
                "TestRotateInPlace - Expected: " << to_string(expected) << ". Actual: " << to_string(robot.getPose()));
    std::cout << "[PASS] testRotateInPlace\n";
}

/**
 * @brief Tests circular motion discretization drift.
 * Note: Discrete Forward Euler over large dt=1.0s creates accumulated radial error.
 * Using smaller dt yields closer convergence.
 */
void testCircularMotion() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    robot.setVelocityCommand({
        .linearVelocity = 1.0,
        .angularVelocity = 1.0,
    });

    // Use smaller time steps for integration accuracy over 2π seconds (full circle)
    double dt = 0.01;
    int steps = static_cast<int>((2 * std::numbers::pi) / dt);

    for (int i = 0; i < steps; ++i) {
        robot.update(dt);
    }

    // With dt=0.01s, numerical drift is small (~0.03m tolerance needed)
    Pose expected{0.0, 0.0, 0.0};
    assert_msg(isPoseClose(robot.getPose(), expected, 0.03), 
                "TestCircularMotion - Expected: " << to_string(expected) << ". Actual: " << to_string(robot.getPose()));
    std::cout << "[PASS] testCircularMotion\n";
}

/**
 * @brief Tests execution of a 4-leg square trajectory returning to starting pose.
 */
void testMoveInASquare() {
    Robot robot(Pose{0.0, 0.0, 0.0});

    for (int i = 0; i < 4; ++i) {
        // Move forward 10m
        robot.setVelocityCommand({
            .linearVelocity = 1.0,
            .angularVelocity = 0.0
        });
        robot.update(10.0);

        // Turn right 90 deg (-π/2 rad)
        robot.setVelocityCommand({
            .linearVelocity = 0.0,
            .angularVelocity = -(std::numbers::pi / 2.0)
        });
        robot.update(1.0);
    }

    Pose expected{0.0, 0.0, 0.0};
    assert_msg(isPoseClose(robot.getPose(), expected), 
                "TestMoveInASquare - Expected: " << to_string(expected) << ". Actual: " << to_string(robot.getPose()));
    std::cout << "[PASS] testMoveInASquare\n";
}

int main() {
    testStationary();
    testMoveForward();
    testRotateInPlace();
    testCircularMotion();
    testMoveInASquare();

    std::cout << "\nAll unit tests passed successfully!\n";
    return 0;
}