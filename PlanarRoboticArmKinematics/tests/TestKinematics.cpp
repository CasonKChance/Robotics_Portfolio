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

TEST(PlanarRoboticArmKinematicsTestSuite, IsReachableTest) {
    // Line lengths are equal, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengths = {1.0, 1.0};
    
    // Test case 1: Point within reach
    Point2D point1{1.0, 1.0};
    EXPECT_TRUE(Kinematics::isReachable(point1, linkLengths));

    // Test case 2: Point at maximum reach
    Point2D point2{2.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point2, linkLengths));

    // Test case 3: Point at minimum reach (0,0)
    Point2D point3{0.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point3, linkLengths));

    // Test case 4: Point outside reach
    Point2D point4{3.0, 0.0};
    EXPECT_FALSE(Kinematics::isReachable(point4, linkLengths));

    // Link lengths are unequal, so minimum reach is the difference of the lengths and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal = {1.0, 2.0};

    // Test case 5: Point within reach
    Point2D point5{1.0, 1.0};
    EXPECT_TRUE(Kinematics::isReachable(point5, linkLengthsUnequal));

    // Test case 6: Point at maximum reach
    Point2D point6{3.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point6, linkLengthsUnequal));

    // Test case 7: Point at minimum reach (1,0)
    Point2D point7{1.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point7, linkLengthsUnequal));

    // Test case 8: Point outside reach
    Point2D point8{4.0, 0.0};
    EXPECT_FALSE(Kinematics::isReachable(point8, linkLengthsUnequal));

    // Test case 9: Point outside reach (less than minimum reach)
    Point2D point9{0.0, 0.0};
    EXPECT_FALSE(Kinematics::isReachable(point9, linkLengthsUnequal));

    // Link lengths are unequal but the longest link is less than the sum of the other links, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal2 = {2.0, 1.5, 1.5};

    // Test case 10: Point within reach
    Point2D point10{1.0, 1.0};
    EXPECT_TRUE(Kinematics::isReachable(point10, linkLengthsUnequal2));

    // Test case 11: Point at maximum reach
    Point2D point11{5.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point11, linkLengthsUnequal2));

    // Test case 12: Point at minimum reach (0,0)
    Point2D point12{0.0, 0.0};
    EXPECT_TRUE(Kinematics::isReachable(point12, linkLengthsUnequal2));

    // Test case 13: Point outside reach
    Point2D point13{6.0, 0.0};
    EXPECT_FALSE(Kinematics::isReachable(point13, linkLengthsUnequal2));
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}