#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <gtest/gtest.h>

#include <numbers>
#include <cmath>

TEST(PlanarRoboticArmKinematicsTestSuite, ForwardKinematicsTest) {
    // Test case 1: Arm fully extended along the x-axis
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};

    Point2D endEffectorPosition1 = Kinematics::forwardKinematics(jointAngles1, linkLengths1);

    double expectedX1 = 2.0;
    double expectedY1 = 0.0;

    EXPECT_NEAR(endEffectorPosition1.x, expectedX1, 1e-6);
    EXPECT_NEAR(endEffectorPosition1.y, expectedY1, 1e-6);

    // Test case 2: Arm fully extended along the y-axis
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {1.0, 1.0};

    Point2D endEffectorPosition2 = Kinematics::forwardKinematics(jointAngles2, linkLengths2);

    double expectedX2 = 0.0;
    double expectedY2 = 2.0;

    EXPECT_NEAR(endEffectorPosition2.x, expectedX2, 1e-6);
    EXPECT_NEAR(endEffectorPosition2.y, expectedY2, 1e-6);

    // Test case 3: Arm at a 45-degree angle
    std::vector<double> jointAngles3 = {std::numbers::pi / 4, std::numbers::pi / 4};
    std::vector<double> linkLengths3 = {1.0, 1.0};

    Point2D endEffectorPosition3 = Kinematics::forwardKinematics(jointAngles3, linkLengths3);

    double expectedX3 = std::cos(std::numbers::pi / 4) + std::cos(std::numbers::pi / 2);
    double expectedY3 = std::sin(std::numbers::pi / 4) + std::sin(std::numbers::pi / 2);

    EXPECT_NEAR(endEffectorPosition3.x, expectedX3, 1e-6);
    EXPECT_NEAR(endEffectorPosition3.y, expectedY3, 1e-6);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}