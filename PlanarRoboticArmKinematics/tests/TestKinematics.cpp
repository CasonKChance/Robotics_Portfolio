#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <gtest/gtest.h>

#include <numbers>
#include <cmath>

TEST(PlanarRoboticArmKinematicsTestSuite, ForwardKinematicsTest) {
    const double pi = std::numbers::pi;
    const double tolerance = 1e-6;

    std::vector<double> linkLengths = {1.0, 1.0, 1.0};
    std::vector<Eigen::VectorXd> jointAxes = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY()
    };

    // Test case 1: Arm fully extended along the x-axis
    std::vector<double> jointAngles1 = {0.0, 0.0, 0.0};

    Eigen::Vector3d endEffectorPosition1 = Kinematics::forwardKinematics(jointAngles1, linkLengths, jointAxes);

    double expectedX1 = 3.0;
    double expectedY1 = 0.0;
    double expectedZ1 = 0.0;

    EXPECT_NEAR(endEffectorPosition1.x(), expectedX1, tolerance);
    EXPECT_NEAR(endEffectorPosition1.y(), expectedY1, tolerance);
    EXPECT_NEAR(endEffectorPosition1.z(), expectedZ1, tolerance);

    // Test case 2: Arm fully extended along the y-axis
    std::vector<double> jointAngles2 = {pi/2, 0.0, 0.0};

    Eigen::Vector3d endEffectorPosition2 = Kinematics::forwardKinematics(jointAngles2, linkLengths, jointAxes);

    double expectedX2 = 0.0;
    double expectedY2 = 3.0;
    double expectedZ2 = 0.0;

    EXPECT_NEAR(endEffectorPosition2.x(), expectedX2, tolerance);
    EXPECT_NEAR(endEffectorPosition2.y(), expectedY2, tolerance);
    EXPECT_NEAR(endEffectorPosition2.z(), expectedZ2, tolerance);

    // Test case 3: Arm fully extended along the z-axis
    std::vector<double> jointAngles3 = {0.0, -pi/2, 0.0};

    Eigen::Vector3d endEffectorPosition3 = Kinematics::forwardKinematics(jointAngles3, linkLengths, jointAxes);

    double expectedX3 = 1.0;
    double expectedY3 = 0.0;
    double expectedZ3 = 2.0;

    EXPECT_NEAR(endEffectorPosition3.x(), expectedX3, tolerance);
    EXPECT_NEAR(endEffectorPosition3.y(), expectedY3, tolerance);
    EXPECT_NEAR(endEffectorPosition3.z(), expectedZ3, tolerance);

    // Test case 4: Each joint rotated by 45 degrees
    std::vector<double> jointAngles4 = {pi/4, pi/4, pi/4};

    Eigen::Vector3d endEffectorPosition4 = Kinematics::forwardKinematics(jointAngles4, linkLengths, jointAxes);

    double expectedX4 = std::cos(pi / 4) * (1.0 + std::cos(pi / 4) + std::cos(pi / 2));
    double expectedY4 = std::sin(pi / 4) * (1.0 + std::cos(pi / 4) + std::cos(pi / 2));
    double expectedZ4 = -std::sin(pi / 4) - std::sin(pi / 2);

    EXPECT_NEAR(endEffectorPosition4.x(), expectedX4, tolerance);
    EXPECT_NEAR(endEffectorPosition4.y(), expectedY4, tolerance);
    EXPECT_NEAR(endEffectorPosition4.z(), expectedZ4, tolerance);
}

TEST(PlanarRoboticArmKinematicsTestSuite, IsWithinReachBoundsTest) {
    // Line lengths are equal, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengths = {1.0, 1.0, 1.0};
    
    // Test case 1: Point within reach
    Eigen::Vector3d point1(1.0, 1.0, 1.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point1, linkLengths));

    // Test case 2: Point at maximum reach
    Eigen::Vector3d point2(3.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point2, linkLengths));

    // Test case 3: Point at minimum reach (0,0)
    Eigen::Vector3d point3(0.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point3, linkLengths));

    // Test case 4: Point outside reach
    Eigen::Vector3d point4(4.0, 0.0, 0.0);
    EXPECT_FALSE(Kinematics::isWithinReachBounds(point4, linkLengths));

    // Link lengths are unequal, so minimum reach is the difference of the longest link length and the sum of the rest of the links lengths and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal = {3.0, 1.0, 1.0};

    // Test case 5: Point within reach
    Eigen::Vector3d point5(1.0, 1.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point5, linkLengthsUnequal));

    // Test case 6: Point at maximum reach
    Eigen::Vector3d point6(5.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point6, linkLengthsUnequal));

    // Test case 7: Point at minimum reach (1,0)
    Eigen::Vector3d point7(1.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point7, linkLengthsUnequal));

    // Test case 8: Point outside reach
    Eigen::Vector3d point8(6.0, 0.0, 0.0);
    EXPECT_FALSE(Kinematics::isWithinReachBounds(point8, linkLengthsUnequal));

    // Test case 9: Point outside reach (less than minimum reach)
    Eigen::Vector3d point9(0.0, 0.0, 0.0);
    EXPECT_FALSE(Kinematics::isWithinReachBounds(point9, linkLengthsUnequal));

    // Link lengths are unequal but the longest link is less than the sum of the other links, so minimum reach is 0 and maximum reach is the sum of the lengths.
    std::vector<double> linkLengthsUnequal2 = {2.0, 1.5, 1.5};

    // Test case 10: Point within reach
    Eigen::Vector3d point10(1.0, 1.0, 1.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point10, linkLengthsUnequal2));

    // Test case 11: Point at maximum reach
    Eigen::Vector3d point11(5.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point11, linkLengthsUnequal2));

    // Test case 12: Point at minimum reach (0,0)
    Eigen::Vector3d point12(0.0, 0.0, 0.0);
    EXPECT_TRUE(Kinematics::isWithinReachBounds(point12, linkLengthsUnequal2));

    // Test case 13: Point outside reach
    Eigen::Vector3d point13(6.0, 0.0, 0.0);
    EXPECT_FALSE(Kinematics::isWithinReachBounds(point13, linkLengthsUnequal2));
}

TEST(PlanarRoboticArmKinematicsTestSuite, SpatialJacobianTest) {
    const double pi = std::numbers::pi;
    const double tolerance = 1e-6;
    const std::vector<Eigen::VectorXd> jointAxes = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY()
    };

    // Test case 1: Three-link arm fully extended along X axisz
    std::vector<double> jointAngles1 = {0.0, 0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0, 1.0};

    auto J1 = Kinematics::spatialJacobian(jointAngles1, linkLengths1, jointAxes);

    Eigen::Matrix<double, 6, 3> expectedJ1;
    expectedJ1 << 0.0, 0.0, 0.0,
                  0.0, 1.0, 1.0,
                  1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0,
                  0.0, 1.0, 2.0;

    EXPECT_TRUE(J1.isApprox(expectedJ1, tolerance));

    // Test case 2: Three-link arm fully extended along Y axis
    std::vector<double> jointAngles2 = {pi/2, 0.0, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0, 1.0};

    auto J2 = Kinematics::spatialJacobian(jointAngles2, linkLengths2, jointAxes);

    Eigen::Matrix<double, 6, 3> expectedJ2;
    expectedJ2 << 0.0, -1.0, -1.0,
                  0.0,  0.0,  0.0,
                  1.0,  0.0,  0.0,
                  0.0,  0.0,  0.0,
                  0.0,  0.0,  0.0,
                  0.0,  2.0,  3.0;

    EXPECT_TRUE(J2.isApprox(expectedJ2, tolerance));

    // Test case 3: Three-link arm straight at 90 degrees
    std::vector<double> jointAngles3 = {0.0, pi/2, 0.0};
    std::vector<double> linkLengths3 = {2.0, 1.0, 1.0};

    auto J3 = Kinematics::spatialJacobian(jointAngles3, linkLengths3, jointAxes);

    Eigen::Matrix<double, 6, 3> expectedJ3;
    expectedJ3 << 0.0,  0.0,  0.0,
                  0.0,  1.0,  1.0,
                  1.0,  0.0,  0.0,
                  0.0,  0.0,  1.0,
                  0.0,  0.0,  0.0,
                  0.0,  2.0,  2.0;

    EXPECT_TRUE(J3.isApprox(expectedJ3, tolerance));

    // Test case 4: Three-link arm with mixed joint angles
    std::vector<double> jointAngles4 = {0.0, pi/2, -pi/2};
    std::vector<double> linkLengths4 = {1.0, 2.0, 1.0};

    auto J4 = Kinematics::spatialJacobian(jointAngles4, linkLengths4, jointAxes);

    Eigen::Matrix<double, 6, 3> expectedJ4;
    expectedJ4 << 0.0,  0.0,  0.0,
                  0.0,  1.0,  1.0,
                  1.0,  0.0,  0.0,
                  0.0,  0.0,  2.0,
                  0.0,  0.0,  0.0,
                  0.0,  1.0,  1.0;

    EXPECT_TRUE(J4.isApprox(expectedJ4, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, LinearVelocityJacobianTest) {
    const double pi = std::numbers::pi;
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended along X, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    const std::vector<Eigen::VectorXd> jointAxes1 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto J1 = Kinematics::linearVelocityJacobian(jointAngles1, linkLengths1, jointAxes1);

    Eigen::Matrix<double, 3, 2> expectedJ1;
    expectedJ1 << 0.0, 0.0,
                  2.0, 1.0,
                  0.0, 0.0;

    EXPECT_TRUE(J1.isApprox(expectedJ1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    const std::vector<Eigen::VectorXd> jointAxes2 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto J2 = Kinematics::linearVelocityJacobian(jointAngles2, linkLengths2, jointAxes2);

    Eigen::Matrix<double, 3, 2> expectedJ2;
    expectedJ2 << -3.0, -1.0,
                   0.0,  0.0,
                   0.0,  0.0;

    EXPECT_TRUE(J2.isApprox(expectedJ2, tolerance));

    // Test case 3: Three-link arm with mixed joint angles, rotating about Y
    std::vector<double> jointAngles3 = {
        0.0, std::numbers::pi / 2, -std::numbers::pi / 2
    };
    std::vector<double> linkLengths3 = {1.0, 2.0, 1.0};
    const std::vector<Eigen::VectorXd> jointAxes3 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY()
    };

    auto J3 = Kinematics::linearVelocityJacobian(jointAngles3, linkLengths3, jointAxes3);

    Eigen::Matrix<double, 3, 3> expectedJ3;
    expectedJ3 << 0.0, -2.0,  0.0,
                2.0,  0.0,  0.0,
                0.0, -1.0, -1.0;

    EXPECT_TRUE(J3.isApprox(expectedJ3, tolerance));

    // Test case 4: Two-link arm fully extended along X, rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    const std::vector<Eigen::VectorXd> jointAxes4 = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitX()
    };

    auto J4 = Kinematics::linearVelocityJacobian(jointAngles4, linkLengths4, jointAxes4);

    Eigen::Matrix<double, 3, 2> expectedJ4;
    expectedJ4 << 0.0, 0.0,
                0.0, 0.0,
                0.0, 0.0;

    EXPECT_TRUE(J4.isApprox(expectedJ4, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, EndEffectorTwistTest) {
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities1(2);
    jointVelocities1 << 1.0, 2.0;
    const std::vector<Eigen::VectorXd> jointAxes1 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::VectorXd twist1 = Kinematics::endEffectorTwist(
        jointAngles1, linkLengths1, jointVelocities1, jointAxes1
    );

    Eigen::Matrix<double, 6, 1> expectedTwist1;
    expectedTwist1 << 0.0, 0.0, 3.0, 0.0, -2.0, 0.0;

    EXPECT_TRUE(twist1.isApprox(expectedTwist1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities2(2);
    jointVelocities2 << 1.0, -0.5;
    const std::vector<Eigen::VectorXd> jointAxes2 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::VectorXd twist2 = Kinematics::endEffectorTwist(
        jointAngles2, linkLengths2, jointVelocities2, jointAxes2
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
    const std::vector<Eigen::VectorXd> jointAxes3 = {
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY()
    };

    Eigen::VectorXd twist3 = Kinematics::endEffectorTwist(
        jointAngles3, linkLengths3, jointVelocities3, jointAxes3
    );

    Eigen::Matrix<double, 6, 1> expectedTwist3;
    expectedTwist3 << 0.0, 2.0, 0.0, -2.0, 0.0, 1.0;

    EXPECT_TRUE(twist3.isApprox(expectedTwist3, tolerance));

    // Test case 4: Two-link arm rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities4(2);
    jointVelocities4 << 0.5, 1.5;
    const std::vector<Eigen::VectorXd> jointAxes4 = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitX()
    };

    Eigen::VectorXd twist4 = Kinematics::endEffectorTwist(
        jointAngles4, linkLengths4, jointVelocities4, jointAxes4
    );

    Eigen::Matrix<double, 6, 1> expectedTwist4;
    expectedTwist4 << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    EXPECT_TRUE(twist4.isApprox(expectedTwist4, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, endEffectorLinearVelocityTest) {
    const double tolerance = 1e-6;

    // Test case 1: Two-link arm fully extended, rotating about Z
    std::vector<double> jointAngles1 = {0.0, 0.0};
    std::vector<double> linkLengths1 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities1(2);
    jointVelocities1 << 1.0, 2.0;
    const std::vector<Eigen::VectorXd> jointAxes1 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::Vector3d velocity1 = Kinematics::endEffectorLinearVelocity(
        jointAngles1, linkLengths1, jointVelocities1, jointAxes1
    );

    Eigen::Vector3d expectedVelocity1(0.0, 4.0, 0.0);

    EXPECT_TRUE(velocity1.isApprox(expectedVelocity1, tolerance));

    // Test case 2: Two-link arm straight at 90 degrees, rotating about Z
    std::vector<double> jointAngles2 = {std::numbers::pi / 2, 0.0};
    std::vector<double> linkLengths2 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities2(2);
    jointVelocities2 << 1.0, -0.5;
    const std::vector<Eigen::VectorXd> jointAxes2 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::Vector3d velocity2 = Kinematics::endEffectorLinearVelocity(
        jointAngles2, linkLengths2, jointVelocities2, jointAxes2
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
    const std::vector<Eigen::VectorXd> jointAxes3 = {
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY(),
        Eigen::Vector3d::UnitY()
    };

    Eigen::Vector3d velocity3 = Kinematics::endEffectorLinearVelocity(
        jointAngles3, linkLengths3, jointVelocities3, jointAxes3
    );

    Eigen::Vector3d expectedVelocity3(-6.0, 0.0, -3.0);

    EXPECT_TRUE(velocity3.isApprox(expectedVelocity3, tolerance));

    // Test case 4: Two-link arm rotating about X
    std::vector<double> jointAngles4 = {0.0, 0.0};
    std::vector<double> linkLengths4 = {2.0, 1.0};
    Eigen::VectorXd jointVelocities4(2);
    jointVelocities4 << 0.5, 1.5;
    const std::vector<Eigen::VectorXd> jointAxes4 = {
        Eigen::Vector3d::UnitX(),
        Eigen::Vector3d::UnitX()
    };

    Eigen::Vector3d velocity4 = Kinematics::endEffectorLinearVelocity(
        jointAngles4, linkLengths4, jointVelocities4, jointAxes4
    );

    Eigen::Vector3d expectedVelocity4(0.0, 0.0, 0.0);

    EXPECT_TRUE(velocity4.isApprox(expectedVelocity4, tolerance));

    // Test case 5: Test calculated velocity against Forward Kinematics derived velocity
    std::vector<double> jointAngles5 = {0.0, 0.0};
    std::vector<double> linkLengths5 = {1.0, 1.0};
    Eigen::VectorXd jointVelocities5(2);
    jointVelocities5 << 1.0, 2.0;
    const std::vector<Eigen::VectorXd> jointAxes5 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    Eigen::Vector3d velocity5 = Kinematics::endEffectorLinearVelocity(
        jointAngles5, linkLengths5, jointVelocities5, jointAxes5
    );

    Eigen::Vector3d p1 = Kinematics::forwardKinematics(jointAngles5, linkLengths5, jointAxes5);

    double dt = 1e-7;
    std::vector<double> jointAngles5AfterTime = {
        jointAngles5[0] + jointVelocities5[0] * dt, 
        jointAngles5[1] + jointVelocities5[1] * dt
    };
    Eigen::Vector3d p2 = Kinematics::forwardKinematics(jointAngles5AfterTime, linkLengths5, jointAxes5);

    Eigen::Vector3d expectedVelocity5 = (p2 - p1) / dt;

    EXPECT_TRUE(velocity5.isApprox(expectedVelocity5, tolerance));
}

TEST(PlanarRoboticArmKinematicsTestSuite, AnalyticInverseKinematicsTest) {
    const double tolerance = 1e-6;

    // Test case 1: Single-link arm along the x-axis
    Eigen::Vector3d targetPosition1(2.0, 0.0, 0.0);
    std::vector<double> linkLengths1 = {2.0};

    auto solutions1 = Kinematics::analyticInverseKinematics(targetPosition1, linkLengths1);

    ASSERT_EQ(solutions1.size(), 1);
    ASSERT_EQ(solutions1[0].size(), 1);
    EXPECT_NEAR(solutions1[0][0], 0.0, tolerance);

    // Test case 2: Single-link arm along the y-axis
    Eigen::Vector3d targetPosition2(0.0, 2.0, 0.0);
    std::vector<double> linkLengths2 = {2.0};

    auto solutions2 = Kinematics::analyticInverseKinematics(targetPosition2, linkLengths2);

    ASSERT_EQ(solutions2.size(), 1);
    ASSERT_EQ(solutions2[0].size(), 1);
    EXPECT_NEAR(solutions2[0][0], std::numbers::pi / 2, tolerance);

    // Test case 3: Two-link arm with two solutions (elbow up and elbow down)
    Eigen::Vector3d targetPosition3(1.0, 1.0, 0.0);
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
    Eigen::Vector3d targetPosition4(2.0, 0.0, 0.0);
    std::vector<double> linkLengths4 = {1.0, 1.0};

    auto solutions4 = Kinematics::analyticInverseKinematics(targetPosition4, linkLengths4);

    ASSERT_EQ(solutions4.size(), 1);
    ASSERT_EQ(solutions4[0].size(), 2);
    EXPECT_NEAR(solutions4[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions4[0][1], 0.0, tolerance);

    // Test case 5: Two-link arm fully extended along the y-axis
    Eigen::Vector3d targetPosition5(0.0, 2.0, 0.0);
    std::vector<double> linkLengths5 = {1.0, 1.0};

    auto solutions5 = Kinematics::analyticInverseKinematics(targetPosition5, linkLengths5);

    ASSERT_EQ(solutions5.size(), 1);
    ASSERT_EQ(solutions5[0].size(), 2);
    EXPECT_NEAR(solutions5[0][0], std::numbers::pi / 2, tolerance);
    EXPECT_NEAR(solutions5[0][1], 0.0, tolerance);

    // Test case 6: Two-link arm with equal link lengths reaching the origin
    Eigen::Vector3d targetPosition6(0.0, 0.0, 0.0);
    std::vector<double> linkLengths6 = {1.0, 1.0};

    auto solutions6 = Kinematics::analyticInverseKinematics(targetPosition6, linkLengths6);

    ASSERT_EQ(solutions6.size(), 1);
    ASSERT_EQ(solutions6[0].size(), 2);
    EXPECT_NEAR(solutions6[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions6[0][1], std::numbers::pi, tolerance);

    // Test case 7: Two-link arm at minimum reach with first link longer
    Eigen::Vector3d targetPosition7(1.0, 0.0, 0.0);
    std::vector<double> linkLengths7 = {2.0, 1.0};

    auto solutions7 = Kinematics::analyticInverseKinematics(targetPosition7, linkLengths7);

    ASSERT_EQ(solutions7.size(), 1);
    ASSERT_EQ(solutions7[0].size(), 2);
    EXPECT_NEAR(solutions7[0][0], 0.0, tolerance);
    EXPECT_NEAR(solutions7[0][1], std::numbers::pi, tolerance);

    // Test case 8: Two-link arm at minimum reach with second link longer
    Eigen::Vector3d targetPosition8(1.0, 0.0, 0.0);
    std::vector<double> linkLengths8 = {1.0, 2.0};

    auto solutions8 = Kinematics::analyticInverseKinematics(targetPosition8, linkLengths8);

    ASSERT_EQ(solutions8.size(), 1);
    ASSERT_EQ(solutions8[0].size(), 2);
    EXPECT_NEAR(std::abs(solutions8[0][0]), std::numbers::pi, tolerance);
    EXPECT_NEAR(solutions8[0][1], std::numbers::pi, tolerance);

    // Test case 9: Two-link arm with unreachable target beyond maximum reach
    Eigen::Vector3d targetPosition9(3.0, 0.0, 0.0);
    std::vector<double> linkLengths9 = {1.0, 1.0};

    auto solutions9 = Kinematics::analyticInverseKinematics(targetPosition9, linkLengths9);

    EXPECT_TRUE(solutions9.empty());

    // Test case 10: Two-link arm with unreachable target inside minimum reach
    Eigen::Vector3d targetPosition10(0.0, 0.0, 0.0);
    std::vector<double> linkLengths10 = {2.0, 1.0};

    auto solutions10 = Kinematics::analyticInverseKinematics(targetPosition10, linkLengths10);

    EXPECT_TRUE(solutions10.empty());

    // Test case 11: Single-link arm with unreachable target
    Eigen::Vector3d targetPosition11(1.0, 1.0, 0.0);
    std::vector<double> linkLengths11 = {1.0};

    auto solutions11 = Kinematics::analyticInverseKinematics(targetPosition11, linkLengths11);

    EXPECT_TRUE(solutions11.empty());

    // Test case 12: Two-link arm with unequal lengths and two solutions
    Eigen::Vector3d targetPosition12(2.0, 1.0, 0.0);
    std::vector<double> linkLengths12 = {2.0, 1.0};
    std::vector<Eigen::VectorXd> jointAxes12 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solutions12 = Kinematics::analyticInverseKinematics(targetPosition12, linkLengths12);

    ASSERT_EQ(solutions12.size(), 2);

    for (const auto& solution : solutions12) {
        ASSERT_EQ(solution.size(), 2);

        Eigen::Vector3d calculatedPosition = Kinematics::forwardKinematics(solution, linkLengths12,jointAxes12);

        EXPECT_TRUE(calculatedPosition.isApprox(targetPosition12, tolerance));
    }

    // Test case 13: More than two links should throw an exception
    Eigen::Vector3d targetPosition13(1.0, 1.0, 0.0);
    std::vector<double> linkLengths13 = {1.0, 1.0, 1.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition13, linkLengths13),
        std::invalid_argument
    );

    // Test case 14: Empty link lengths should throw an exception
    Eigen::Vector3d targetPosition14(1.0, 1.0, 0.0);
    std::vector<double> linkLengths14 = {};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition14, linkLengths14),
        std::invalid_argument
    );

    // Test case 15: Zero link length should throw an exception
    Eigen::Vector3d targetPosition15(1.0, 1.0, 0.0);
    std::vector<double> linkLengths15 = {1.0, 0.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition15, linkLengths15),
        std::invalid_argument
    );

    // Test case 16: Negative link length should throw an exception
    Eigen::Vector3d targetPosition16(1.0, 1.0, 0.0);
    std::vector<double> linkLengths16 = {1.0, -1.0};

    EXPECT_THROW(
        Kinematics::analyticInverseKinematics(targetPosition16, linkLengths16),
        std::invalid_argument
    );
}

TEST(PlanarRoboticArmKinematicsTestSuite, NumericalInverseKinematicsTest) {
    const double tolerance = 1e-6;
    const Eigen::Vector3d errorTolerance(1e-8, 1e-8, 1e-8);
    const int maximumIterations = 100;

    // Test case 1: Single-link arm along the x-axis
    Eigen::Vector3d targetPosition1(2.0, 0.0, 0.0);
    std::vector<double> linkLengths1 = {2.0};
    std::vector<double> initialGuess1 = {0.5};
    std::vector<Eigen::VectorXd> jointAxes1 = {
        Eigen::Vector3d::UnitZ()
    };

    auto solution1 = Kinematics::numericalInverseKinematics(
        targetPosition1, linkLengths1, jointAxes1,
        initialGuess1, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution1.size(), 1);
    EXPECT_NEAR(solution1[0], 0.0, tolerance);

    // Test case 2: Single-link arm along the y-axis
    Eigen::Vector3d targetPosition2(0.0, 2.0, 0.0);
    std::vector<double> linkLengths2 = {2.0};
    std::vector<double> initialGuess2 = {0.0};
    std::vector<Eigen::VectorXd> jointAxes2 = {
        Eigen::Vector3d::UnitZ()
    };

    auto solution2 = Kinematics::numericalInverseKinematics(
        targetPosition2, linkLengths2, jointAxes2,
        initialGuess2, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution2.size(), 1);
    EXPECT_NEAR(solution2[0], std::numbers::pi / 2, tolerance);

    // Test case 3: Two-link arm reaching a target within its workspace
    Eigen::Vector3d targetPosition3(1.0, 1.0, 0.0);
    std::vector<double> linkLengths3 = {1.0, 1.0};
    std::vector<double> initialGuess3 = {0.2, 1.2};
    std::vector<Eigen::VectorXd> jointAxes3 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution3 = Kinematics::numericalInverseKinematics(
        targetPosition3, linkLengths3, jointAxes3,
        initialGuess3, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution3.size(), 2);

    Eigen::Vector3d calculatedPosition3 =
        Kinematics::forwardKinematics(solution3, linkLengths3, jointAxes3);

    EXPECT_TRUE(calculatedPosition3.isApprox(targetPosition3, tolerance));

    // Test case 4: Two-link arm with unequal link lengths
    Eigen::Vector3d targetPosition4(2.0, 1.0, 0.0);
    std::vector<double> linkLengths4 = {2.0, 1.0};
    std::vector<double> initialGuess4 = {0.1, 1.3};
    std::vector<Eigen::VectorXd> jointAxes4 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution4 = Kinematics::numericalInverseKinematics(
        targetPosition4, linkLengths4, jointAxes4,
        initialGuess4, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution4.size(), 2);

    Eigen::Vector3d calculatedPosition4 =
        Kinematics::forwardKinematics(solution4, linkLengths4, jointAxes4);

    EXPECT_TRUE(calculatedPosition4.isApprox(targetPosition4, tolerance));

    // Test case 5: Three-link arm reaching a target within its workspace
    std::vector<double> linkLengths5 = {1.0, 1.0, 1.0};
    std::vector<double> expectedAngles5 = {0.3, 0.5, -0.4};
    std::vector<Eigen::VectorXd> jointAxes5 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };
    Eigen::Vector3d targetPosition5 =
        Kinematics::forwardKinematics(expectedAngles5, linkLengths5, jointAxes5);
    std::vector<double> initialGuess5 = {0.4, 0.4, -0.3};

    auto solution5 = Kinematics::numericalInverseKinematics(
        targetPosition5, linkLengths5, jointAxes5,
        initialGuess5, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution5.size(), 3);

    Eigen::Vector3d calculatedPosition5 =
        Kinematics::forwardKinematics(solution5, linkLengths5, jointAxes5);

    EXPECT_TRUE(calculatedPosition5.isApprox(targetPosition5, tolerance));

    // Test case 6: Four-link arm reaching a target within its workspace
    std::vector<double> linkLengths6 = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> expectedAngles6 = {0.2, 0.4, -0.3, 0.5};
    std::vector<Eigen::VectorXd> jointAxes6 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };
    Eigen::Vector3d targetPosition6 =
        Kinematics::forwardKinematics(expectedAngles6, linkLengths6, jointAxes6);
    std::vector<double> initialGuess6 = {0.3, 0.3, -0.2, 0.4};

    auto solution6 = Kinematics::numericalInverseKinematics(
        targetPosition6, linkLengths6, jointAxes6,
        initialGuess6, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution6.size(), 4);

    Eigen::Vector3d calculatedPosition6 =
        Kinematics::forwardKinematics(solution6, linkLengths6, jointAxes6);

    EXPECT_TRUE(calculatedPosition6.isApprox(targetPosition6, tolerance));

    // Test case 7: Initial guess already satisfies the target
    std::vector<double> linkLengths7 = {1.0, 1.0};
    std::vector<double> initialGuess7 = {0.4, 0.8};
    std::vector<Eigen::VectorXd> jointAxes7 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };
    Eigen::Vector3d targetPosition7 =
        Kinematics::forwardKinematics(initialGuess7, linkLengths7, jointAxes7);

    auto solution7 = Kinematics::numericalInverseKinematics(
        targetPosition7, linkLengths7, jointAxes7,
        initialGuess7, errorTolerance, maximumIterations
    );

    ASSERT_EQ(solution7.size(), 2);
    EXPECT_NEAR(solution7[0], initialGuess7[0], tolerance);
    EXPECT_NEAR(solution7[1], initialGuess7[1], tolerance);

    // Test case 8: Two-link arm with unreachable target beyond maximum reach
    Eigen::Vector3d targetPosition8(3.0, 0.0, 0.0);
    std::vector<double> linkLengths8 = {1.0, 1.0};
    std::vector<double> initialGuess8 = {0.5, 0.5};
    std::vector<Eigen::VectorXd> jointAxes8 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution8 = Kinematics::numericalInverseKinematics(
        targetPosition8, linkLengths8, jointAxes8,
        initialGuess8, errorTolerance, maximumIterations
    );

    EXPECT_TRUE(solution8.empty());

    // Test case 9: Two-link arm with unreachable target inside minimum reach
    Eigen::Vector3d targetPosition9(0.0, 0.0, 0.0);
    std::vector<double> linkLengths9 = {2.0, 1.0};
    std::vector<double> initialGuess9 = {0.5, 0.5};
    std::vector<Eigen::VectorXd> jointAxes9 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution9 = Kinematics::numericalInverseKinematics(
        targetPosition9, linkLengths9, jointAxes9,
        initialGuess9, errorTolerance, maximumIterations
    );

    EXPECT_TRUE(solution9.empty());

    // Test case 10: Single-link arm with unreachable target
    Eigen::Vector3d targetPosition10(1.0, 1.0, 0.0);
    std::vector<double> linkLengths10 = {1.0};
    std::vector<double> initialGuess10 = {0.0};
    std::vector<Eigen::VectorXd> jointAxes10 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution10 = Kinematics::numericalInverseKinematics(
        targetPosition10, linkLengths10, jointAxes10,
        initialGuess10, errorTolerance, maximumIterations
    );

    EXPECT_TRUE(solution10.empty());

    // Test case 11: Zero maximum iterations should return the initial guess
    Eigen::Vector3d targetPosition11(1.0, 1.0, 0.0);
    std::vector<double> linkLengths11 = {1.0, 1.0};
    std::vector<double> initialGuess11 = {0.2, 0.5};
    std::vector<Eigen::VectorXd> jointAxes11 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    auto solution11 = Kinematics::numericalInverseKinematics(
        targetPosition11, linkLengths11, jointAxes11,
        initialGuess11, errorTolerance, 0
    );

    ASSERT_EQ(solution11.size(), 2);
    EXPECT_NEAR(solution11[0], initialGuess11[0], tolerance);
    EXPECT_NEAR(solution11[1], initialGuess11[1], tolerance);

    // Test case 12: Empty link lengths should throw an exception
    Eigen::Vector3d targetPosition12(1.0, 1.0, 0.0);
    std::vector<double> linkLengths12 = {};
    std::vector<double> initialGuess12 = {};
    std::vector<Eigen::VectorXd> jointAxes12 = {};

    EXPECT_THROW(
        Kinematics::numericalInverseKinematics(
            targetPosition12, linkLengths12, jointAxes12,
            initialGuess12, errorTolerance, maximumIterations
        ),
        std::invalid_argument
    );

    // Test case 13: Zero link length should throw an exception
    Eigen::Vector3d targetPosition13(1.0, 1.0, 0.0);
    std::vector<double> linkLengths13 = {1.0, 0.0};
    std::vector<double> initialGuess13 = {0.0, 0.0};
    std::vector<Eigen::VectorXd> jointAxes13 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    EXPECT_THROW(
        Kinematics::numericalInverseKinematics(
            targetPosition13, linkLengths13, jointAxes13,
            initialGuess13, errorTolerance, maximumIterations
        ),
        std::invalid_argument
    );

    // Test case 14: Negative link length should throw an exception
    Eigen::Vector3d targetPosition14(1.0, 1.0, 0.0);
    std::vector<double> linkLengths14 = {1.0, -1.0};
    std::vector<double> initialGuess14 = {0.0, 0.0};
    std::vector<Eigen::VectorXd> jointAxes14 = {
        Eigen::Vector3d::UnitZ(),
        Eigen::Vector3d::UnitZ()
    };

    EXPECT_THROW(
        Kinematics::numericalInverseKinematics(
            targetPosition14, linkLengths14, jointAxes14,
            initialGuess14, errorTolerance, maximumIterations
        ),
        std::invalid_argument
    );
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}