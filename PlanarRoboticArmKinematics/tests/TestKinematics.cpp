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

TEST(PlanarRoboticArmKinematicsTestSuite, JacobianTest) {
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended along X, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    Eigen::Vector3d axisOfRotation1(0.0, 0.0, 1.0);

    auto J1 = Kinematics::jacobian(jointAngles1, linkLengths1, axisOfRotation1);

    Eigen::Matrix<double, 6, 2> expectedJ1;
    expectedJ1 << 0.0,  0.0,
                  0.0,  0.0,
                  1.0,  1.0,
                  0.0,  0.0,
                  0.0, -1.0,
                  0.0,  0.0;

    EXPECT_TRUE(J1.isApprox(expectedJ1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    Eigen::Vector3d axisOfRotation2(0.0, 0.0, 1.0);

    auto J2 = Kinematics::jacobian(jointAngles2, linkLengths2, axisOfRotation2);

    Eigen::Matrix<double, 6, 2> expectedJ2;
    expectedJ2 << 0.0, 0.0,
                  0.0, 0.0,
                  1.0, 1.0,
                  0.0, 2.0,
                  0.0, 0.0,
                  0.0, 0.0;

    EXPECT_TRUE(J2.isApprox(expectedJ2, tolerance));

    // Test case 3: Three-link arm with mixed joint angles, rotating about Y
    std::vector<double> jointAngles3 = {
        0.0, std::numbers::pi / 2, -std::numbers::pi / 2
    };
    std::vector<double> linkLengths3 = {1.0, 2.0, 1.0};
    Eigen::Vector3d axisOfRotation3(0.0, 1.0, 0.0);

    auto J3 = Kinematics::jacobian(jointAngles3, linkLengths3, axisOfRotation3);

    Eigen::Matrix<double, 6, 3> expectedJ3;
    expectedJ3 << 0.0, 0.0,  0.0,
                  1.0, 1.0,  1.0,
                  0.0, 0.0,  0.0,
                  0.0, 0.0,  2.0,
                  0.0, 0.0,  0.0,
                  0.0, 1.0,  1.0;

    EXPECT_TRUE(J3.isApprox(expectedJ3, tolerance));

    // Test case 4: Two-link arm fully extended along Y, rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    Eigen::Vector3d axisOfRotation4(1.0, 0.0, 0.0);

    auto J4 = Kinematics::jacobian(jointAngles4, linkLengths4, axisOfRotation4);

    Eigen::Matrix<double, 6, 2> expectedJ4;
    expectedJ4 << 1.0,  1.0,
                  0.0,  0.0,
                  0.0,  0.0,
                  0.0,  0.0,
                  0.0,  0.0,
                  0.0, -2.0;

    EXPECT_TRUE(J4.isApprox(expectedJ4, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, EndEffectorTwistTest) {
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities1(2);
    jointVelocities1 << 1.0, 2.0;
    Eigen::Vector3d axisOfRotation1(0.0, 0.0, 1.0);

    Eigen::VectorXd twist1 = Kinematics::endEffectorTwist(
        jointAngles1, linkLengths1, jointVelocities1, axisOfRotation1
    );

    Eigen::Matrix<double, 6, 1> expectedTwist1;
    expectedTwist1 << 0.0, 0.0, 3.0, 0.0, -2.0, 0.0;

    EXPECT_TRUE(twist1.isApprox(expectedTwist1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities2(2);
    jointVelocities2 << 1.0, -0.5;
    Eigen::Vector3d axisOfRotation2(0.0, 0.0, 1.0);

    Eigen::VectorXd twist2 = Kinematics::endEffectorTwist(
        jointAngles2, linkLengths2, jointVelocities2, axisOfRotation2
    );

    Eigen::Matrix<double, 6, 1> expectedTwist2;
    expectedTwist2 << 0.0, 0.0, 0.5, -1.0, 0.0, 0.0;

    EXPECT_TRUE(twist2.isApprox(expectedTwist2, tolerance));

    // Test case 3: Three-link arm with mixed angles, rotating about Y
    std::vector<double> jointAngles3 = {
        0.0, std::numbers::pi / 2, -std::numbers::pi / 2
    };
    std::vector<double> linkLengths3 = {1.0, 2.0, 1.0};
    Eigen::VectorXd jointVelocities3(3);
    jointVelocities3 << 1.0, 2.0, -1.0;
    Eigen::Vector3d axisOfRotation3(0.0, 1.0, 0.0);

    Eigen::VectorXd twist3 = Kinematics::endEffectorTwist(
        jointAngles3, linkLengths3, jointVelocities3, axisOfRotation3
    );

    Eigen::Matrix<double, 6, 1> expectedTwist3;
    expectedTwist3 << 0.0, 2.0, 0.0, -2.0, 0.0, 1.0;

    EXPECT_TRUE(twist3.isApprox(expectedTwist3, tolerance));

    // Test case 4: Two-link arm rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities4(2);
    jointVelocities4 << 0.5, 1.5;
    Eigen::Vector3d axisOfRotation4(1.0, 0.0, 0.0);

    Eigen::VectorXd twist4 = Kinematics::endEffectorTwist(
        jointAngles4, linkLengths4, jointVelocities4, axisOfRotation4
    );

    Eigen::Matrix<double, 6, 1> expectedTwist4;
    expectedTwist4 << 2.0, 0.0, 0.0, 0.0, 0.0, -3.0;

    EXPECT_TRUE(twist4.isApprox(expectedTwist4, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, EndEffectorVelocityTest) {
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities1(2);
    jointVelocities1 << 1.0, 2.0;
    Eigen::Vector3d axisOfRotation1(0.0, 0.0, 1.0);

    Eigen::Vector3d velocity1 = Kinematics::endEffectorVelocity(
        jointAngles1, linkLengths1, jointVelocities1, axisOfRotation1
    );

    Eigen::Vector3d expectedVelocity1(0.0, 4.0, 0.0);

    EXPECT_TRUE(velocity1.isApprox(expectedVelocity1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities2(2);
    jointVelocities2 << 1.0, -0.5;
    Eigen::Vector3d axisOfRotation2(0.0, 0.0, 1.0);

    Eigen::Vector3d velocity2 = Kinematics::endEffectorVelocity(
        jointAngles2, linkLengths2, jointVelocities2, axisOfRotation2
    );

    Eigen::Vector3d expectedVelocity2(-2.5, 0.0, 0.0);

    EXPECT_TRUE(velocity2.isApprox(expectedVelocity2, tolerance));

    // Test case 3: Three-link arm with mixed angles, rotating about Y
    std::vector<double> jointAngles3 = {
        0.0, std::numbers::pi / 2, -std::numbers::pi / 2
    };
    std::vector<double> linkLengths3 = {1.0, 2.0, 1.0};
    Eigen::VectorXd jointVelocities3(3);
    jointVelocities3 << 1.0, 2.0, -1.0;
    Eigen::Vector3d axisOfRotation3(0.0, 1.0, 0.0);

    Eigen::Vector3d velocity3 = Kinematics::endEffectorVelocity(
        jointAngles3, linkLengths3, jointVelocities3, axisOfRotation3
    );

    Eigen::Vector3d expectedVelocity3(-6.0, 0.0, -3.0);

    EXPECT_TRUE(velocity3.isApprox(expectedVelocity3, tolerance));

    // Test case 4: Two-link arm rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities4(2);
    jointVelocities4 << 0.5, 1.5;
    Eigen::Vector3d axisOfRotation4(1.0, 0.0, 0.0);

    Eigen::Vector3d velocity4 = Kinematics::endEffectorVelocity(
        jointAngles4, linkLengths4, jointVelocities4, axisOfRotation4
    );

    Eigen::Vector3d expectedVelocity4(0.0, 0.0, 3.0);

    EXPECT_TRUE(velocity4.isApprox(expectedVelocity4, tolerance));

    // Test case 5: Test calculated velocity against Forward Kinematics derived velocity
    std::vector<double> jointAngles5 = {0.0, 0.0};
    std::vector<double> linkLengths5 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities5(2);
    jointVelocities5 << 1.0, 2.0;
    Eigen::Vector3d axisOfRotation5(0.0, 0.0, 1.0);

    Eigen::Vector3d velocity5 = Kinematics::endEffectorVelocity(
        jointAngles5, linkLengths5, jointVelocities5, axisOfRotation5
    );

    Point2D p1 = Kinematics::forwardKinematics(jointAngles5, linkLengths5);

    double dt = 1e-7;
    std::vector<double> jointAngles5AfterTime = {
        jointAngles5[0] + jointVelocities5[0] * dt, 
        jointAngles5[1] + jointVelocities5[1] * dt
    };
    Point2D p2 = Kinematics::forwardKinematics(jointAngles5AfterTime, linkLengths5);

    Eigen::Vector3d omegaAxis = axisOfRotation5.normalized();

    Eigen::Vector3d axisU;
    if (omegaAxis.x() != 0.0) {
        axisU << 0.0, 1.0, 0.0;
    } else {
        axisU << 1.0, 0.0, 0.0;
    }

    Eigen::Vector3d axisV = omegaAxis.cross(axisU);

    Eigen::Vector3d expectedVelocity5 = axisU * ((p2.x - p1.x)/dt) + axisV * ((p2.y - p1.y)/dt);

    EXPECT_TRUE(velocity5.isApprox(expectedVelocity5, tolerance));
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}