#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <gtest/gtest.h>

#include <numbers>
#include <cmath>

TEST(PlanarRoboticArmKinematicsTestSuite, ForwardKinematicsTest) {
    // Test case 1: Arm fully extended along the x-axis
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};

    Eigen::Vector2d endEffectorPosition1 = Kinematics::forwardKinematics(jointAngles1, linkLengths1);

    double expectedX1 = 2.0;
    double expectedY1 = 0.0;

    EXPECT_NEAR(endEffectorPosition1.x(), expectedX1, 1e-6);
    EXPECT_NEAR(endEffectorPosition1.y(), expectedY1, 1e-6);

    // Test case 2: Arm fully extended along the y-axis
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {1.0, 1.0};

    Eigen::Vector2d endEffectorPosition2 = Kinematics::forwardKinematics(jointAngles2, linkLengths2);

    double expectedX2 = 0.0;
    double expectedY2 = 2.0;

    EXPECT_NEAR(endEffectorPosition2.x(), expectedX2, 1e-6);
    EXPECT_NEAR(endEffectorPosition2.y(), expectedY2, 1e-6);

    // Test case 3: Arm at a 45-degree angle
    std::vector<double> jointAngles3 = {std::numbers::pi / 4, std::numbers::pi / 4};
    std::vector<double> linkLengths3 = {1.0, 1.0};

    Eigen::Vector2d endEffectorPosition3 = Kinematics::forwardKinematics(jointAngles3, linkLengths3);

    double expectedX3 = std::cos(std::numbers::pi / 4) + std::cos(std::numbers::pi / 2);
    double expectedY3 = std::sin(std::numbers::pi / 4) + std::sin(std::numbers::pi / 2);

    EXPECT_NEAR(endEffectorPosition3.x(), expectedX3, 1e-6);
    EXPECT_NEAR(endEffectorPosition3.y(), expectedY3, 1e-6);
}

TEST(PlanarRoboticArmKinematicsTestSuite, IsReachableTest) {
    // Line lengths are equal, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengths = {1.0, 1.0};
    
    // Test case 1: Point within reach
    Eigen::Vector2d point1(1.0, 1.0);
    EXPECT_TRUE(Kinematics::isReachable(point1, linkLengths));

    // Test case 2: Point at maximum reach
    Eigen::Vector2d point2(2.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point2, linkLengths));

    // Test case 3: Point at minimum reach (0,0)
    Eigen::Vector2d point3(0.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point3, linkLengths));

    // Test case 4: Point outside reach
    Eigen::Vector2d point4(3.0, 0.0);
    EXPECT_FALSE(Kinematics::isReachable(point4, linkLengths));

    // Link lengths are unequal, so minimum reach is the difference of the lengths and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal = {1.0, 2.0};

    // Test case 5: Point within reach
    Eigen::Vector2d point5(1.0, 1.0);
    EXPECT_TRUE(Kinematics::isReachable(point5, linkLengthsUnequal));

    // Test case 6: Point at maximum reach
    Eigen::Vector2d point6(3.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point6, linkLengthsUnequal));

    // Test case 7: Point at minimum reach (1,0)
    Eigen::Vector2d point7(1.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point7, linkLengthsUnequal));

    // Test case 8: Point outside reach
    Eigen::Vector2d point8(4.0, 0.0);
    EXPECT_FALSE(Kinematics::isReachable(point8, linkLengthsUnequal));

    // Test case 9: Point outside reach (less than minimum reach)
    Eigen::Vector2d point9(0.0, 0.0);
    EXPECT_FALSE(Kinematics::isReachable(point9, linkLengthsUnequal));

    // Link lengths are unequal but the longest link is less than the sum of the other links, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal2 = {2.0, 1.5, 1.5};

    // Test case 10: Point within reach
    Eigen::Vector2d point10(1.0, 1.0);
    EXPECT_TRUE(Kinematics::isReachable(point10, linkLengthsUnequal2));

    // Test case 11: Point at maximum reach
    Eigen::Vector2d point11(5.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point11, linkLengthsUnequal2));

    // Test case 12: Point at minimum reach (0,0)
    Eigen::Vector2d point12(0.0, 0.0);
    EXPECT_TRUE(Kinematics::isReachable(point12, linkLengthsUnequal2));

    // Test case 13: Point outside reach
    Eigen::Vector2d point13(6.0, 0.0);
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

    Eigen::Vector2d p1 = Kinematics::forwardKinematics(jointAngles5, linkLengths5);

    double dt = 1e-7;
    std::vector<double> jointAngles5AfterTime = {
        jointAngles5[0] + jointVelocities5[0] * dt, 
        jointAngles5[1] + jointVelocities5[1] * dt
    };
    Eigen::Vector2d p2 = Kinematics::forwardKinematics(jointAngles5AfterTime, linkLengths5);

    Eigen::Vector3d omegaAxis = axisOfRotation5.normalized();

    Eigen::Vector3d axisU;
    if (omegaAxis.x() != 0.0) {
        axisU << 0.0, 1.0, 0.0;
    } else {
        axisU << 1.0, 0.0, 0.0;
    }

    Eigen::Vector3d axisV = omegaAxis.cross(axisU);

    Eigen::Vector3d expectedVelocity5 = axisU * ((p2.x() - p1.x())/dt) + axisV * ((p2.y() - p1.y())/dt);

    EXPECT_TRUE(velocity5.isApprox(expectedVelocity5, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, AnalyticInverseKinematicsTest) {
    const double tolerance = 1e-6;

    // Test case 1: Single-link arm along the x-axis
    Eigen::Vector2d targetPosition1(2.0, 0.0);
    std::vector<double> linkLengths1 = {2.0};

    auto solutions1 = Kinematics::analyticInverseKinematics(targetPosition1, linkLengths1);

    ASSERT_EQ(solutions1.size(), 1);
    ASSERT_EQ(solutions1[0].size(), 1);
    EXPECT_NEAR(solutions1[0][0], 0.0, tolerance);

    // Test case 2: Single-link arm along the y-axis
    Eigen::Vector2d targetPosition2(0.0, 2.0);
    std::vector<double> linkLengths2 = {2.0};

    auto solutions2 = Kinematics::analyticInverseKinematics(targetPosition2, linkLengths2);

    ASSERT_EQ(solutions2.size(), 1);
    ASSERT_EQ(solutions2[0].size(), 1);
    EXPECT_NEAR(solutions2[0][0], std::numbers::pi / 2, tolerance);

    // Test case 3: Two-link arm with two solutions (elbow up and elbow down)
    Eigen::Vector2d targetPosition3(1.0, 1.0);
    std::vector<double> linkLengths3 = {1.0, 1.0};

    auto solutions3 = Kinematics::analyticInverseKinematics(targetPosition3, linkLengths3);

    ASSERT_EQ(solutions3.size(), 2);
    ASSERT_EQ(solutions3[0].size(), 2);
    ASSERT_EQ(solutions3[1].size(), 2);

    // First solution
    EXPECT_NEAR(solutions3[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions3[0][1], std::numbers::pi / 2, tolerance);

    // Second solution
    EXPECT_NEAR(solutions3[1][0], std::numbers::pi / 2, tolerance);
    EXPECT_NEAR(solutions3[1][1], -std::numbers::pi / 2, tolerance);

    // Test case 4: Two-link arm fully extended along the x-axis
    Eigen::Vector2d targetPosition4(2.0, 0.0);
    std::vector<double> linkLengths4 = {1.0, 1.0};

    auto solutions4 = Kinematics::analyticInverseKinematics(targetPosition4, linkLengths4);

    ASSERT_EQ(solutions4.size(), 1);
    ASSERT_EQ(solutions4[0].size(), 2);
    EXPECT_NEAR(solutions4[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions4[0][1], 0.0, tolerance);

    // Test case 5: Two-link arm fully extended along the y-axis
    Eigen::Vector2d targetPosition5(0.0, 2.0);
    std::vector<double> linkLengths5 = {1.0, 1.0};

    auto solutions5 = Kinematics::analyticInverseKinematics(targetPosition5, linkLengths5);

    ASSERT_EQ(solutions5.size(), 1);
    ASSERT_EQ(solutions5[0].size(), 2);
    EXPECT_NEAR(solutions5[0][0], std::numbers::pi / 2, tolerance);
    EXPECT_NEAR(solutions5[0][1], 0.0, tolerance);

    // Test case 6: Two-link arm with equal link lengths reaching the origin
    Eigen::Vector2d targetPosition6(0.0, 0.0);
    std::vector<double> linkLengths6 = {1.0, 1.0};

    auto solutions6 = Kinematics::analyticInverseKinematics(targetPosition6, linkLengths6);

    ASSERT_EQ(solutions6.size(), 1);
    ASSERT_EQ(solutions6[0].size(), 2);
    EXPECT_NEAR(solutions6[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions6[0][1], std::numbers::pi, tolerance);

    // Test case 7: Two-link arm at minimum reach with first link longer
    Eigen::Vector2d targetPosition7(1.0, 0.0);
    std::vector<double> linkLengths7 = {2.0, 1.0};

    auto solutions7 = Kinematics::analyticInverseKinematics(targetPosition7, linkLengths7);

    ASSERT_EQ(solutions7.size(), 1);
    ASSERT_EQ(solutions7[0].size(), 2);
    EXPECT_NEAR(solutions7[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions7[0][1], std::numbers::pi, tolerance);

    // Test case 8: Two-link arm at minimum reach with second link longer
    Eigen::Vector2d targetPosition8(1.0, 0.0);
    std::vector<double> linkLengths8 = {1.0, 2.0};

    auto solutions8 = Kinematics::analyticInverseKinematics(targetPosition8, linkLengths8);

    ASSERT_EQ(solutions8.size(), 1);
    ASSERT_EQ(solutions8[0].size(), 2);
    EXPECT_NEAR(std::abs(solutions8[0][0]), std::numbers::pi, tolerance);
    EXPECT_NEAR(solutions8[0][1], std::numbers::pi, tolerance);

    // Test case 9: Two-link arm with unreachable target beyond maximum reach
    Eigen::Vector2d targetPosition9(3.0, 0.0);
    std::vector<double> linkLengths9 = {1.0, 1.0};

    auto solutions9 = Kinematics::analyticInverseKinematics(targetPosition9, linkLengths9);

    EXPECT_TRUE(solutions9.empty());

    // Test case 10: Two-link arm with unreachable target inside minimum reach
    Eigen::Vector2d targetPosition10(0.0, 0.0);
    std::vector<double> linkLengths10 = {2.0, 1.0};

    auto solutions10 = Kinematics::analyticInverseKinematics(targetPosition10, linkLengths10);

    EXPECT_TRUE(solutions10.empty());

    // Test case 11: Single-link arm with unreachable target
    Eigen::Vector2d targetPosition11(1.0, 1.0);
    std::vector<double> linkLengths11 = {1.0};

    auto solutions11 = Kinematics::analyticInverseKinematics(targetPosition11, linkLengths11);

    EXPECT_TRUE(solutions11.empty());

    // Test case 12: Two-link arm with unequal lengths and two solutions
    Eigen::Vector2d targetPosition12(2.0, 1.0);
    std::vector<double> linkLengths12 = {2.0, 1.0};

    auto solutions12 = Kinematics::analyticInverseKinematics(targetPosition12, linkLengths12);

    ASSERT_EQ(solutions12.size(), 2);

    for (const auto& solution : solutions12) {
        ASSERT_EQ(solution.size(), 2);

        Eigen::Vector2d calculatedPosition =
            Kinematics::forwardKinematics(solution, linkLengths12);

        EXPECT_TRUE(calculatedPosition.isApprox(targetPosition12, tolerance));
    }

    // Test case 13: More than two links should throw an exception
    Eigen::Vector2d targetPosition13(1.0, 1.0);
    std::vector<double> linkLengths13 = {1.0, 1.0, 1.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition13, linkLengths13),
        std::invalid_argument
    );

    // Test case 14: Empty link lengths should throw an exception
    Eigen::Vector2d targetPosition14(1.0, 1.0);
    std::vector<double> linkLengths14 = {};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition14, linkLengths14),
        std::invalid_argument
    );

    // Test case 15: Zero link length should throw an exception
    Eigen::Vector2d targetPosition15(1.0, 1.0);
    std::vector<double> linkLengths15 = {1.0, 0.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition15, linkLengths15),
        std::invalid_argument
    );

    // Test case 16: Negative link length should throw an exception
    Eigen::Vector2d targetPosition16(1.0, 1.0);
    std::vector<double> linkLengths16 = {1.0, -1.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition16, linkLengths16),
        std::invalid_argument
    );
}


int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}