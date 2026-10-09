#include <Transformation/Transformation.h>

#include <gtest/gtest.h>

#include <numbers>

TEST(RotationMatrixTestSuite, RotationMatrixXTest) {
    Eigen::Matrix3d R = Transformation::rotationMatrixX(0.0);

    EXPECT_TRUE(Transformation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Transformation::rotationMatrixX(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 1, 0, 0,
                 0, 0, -1,
                 0, 1, 0;

    EXPECT_TRUE(Transformation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Transformation::rotationMatrixX(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << 1, 0, 0,
                 0, -1, 0,
                 0, 0, -1;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Transformation::rotationMatrixX(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 1, 0, 0,
                 0, 0, 1,
                 0, -1, 0;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotationMatrixYTest) {
    Eigen::Matrix3d R = Transformation::rotationMatrixY(0.0);

    EXPECT_TRUE(Transformation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Transformation::rotationMatrixY(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 0, 0, 1,
                 0, 1, 0,
                 -1, 0, 0;

    EXPECT_TRUE(Transformation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Transformation::rotationMatrixY(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << -1, 0, 0,
                 0, 1, 0,
                 0, 0, -1;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Transformation::rotationMatrixY(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 0, 0, -1,
                 0, 1, 0,
                 1, 0, 0;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotationMatrixZTest) {
    Eigen::Matrix3d R = Transformation::rotationMatrixZ(0.0);

    EXPECT_TRUE(Transformation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Transformation::rotationMatrixZ(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 0, -1, 0,
                 1, 0, 0,
                 0, 0, 1;

    EXPECT_TRUE(Transformation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Transformation::rotationMatrixZ(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << -1, 0, 0,
                 0, -1, 0,
                 0, 0, 1;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Transformation::rotationMatrixZ(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 0, 1, 0,
                 -1, 0, 0,
                 0, 0, 1;
                 
    EXPECT_TRUE(Transformation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, IsRotationMatrixTest) {
    // Identity matrix (valid)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(Transformation::isRotationMatrix(I));

    // Rotation about multiple axes (valid)
    Eigen::Matrix3d R =
        Transformation::rotationMatrixX(std::numbers::pi / 4) *
        Transformation::rotationMatrixY(std::numbers::pi / 3) *
        Transformation::rotationMatrixZ(std::numbers::pi / 6);

    EXPECT_TRUE(Transformation::isRotationMatrix(R));

    // Zero matrix (invalid)
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    EXPECT_FALSE(Transformation::isRotationMatrix(zero));

    // Scaling matrix (invalid - not orthogonal)
    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
    scale(0, 0) = 2.0;

    EXPECT_FALSE(Transformation::isRotationMatrix(scale));

    // Reflection matrix (invalid - determinant = -1)
    Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
    reflection(0, 0) = -1.0;

    EXPECT_FALSE(Transformation::isRotationMatrix(reflection));

    // Shear matrix (invalid - not orthogonal)
    Eigen::Matrix3d shear = Eigen::Matrix3d::Identity();
    shear(0, 1) = 0.5;

    EXPECT_FALSE(Transformation::isRotationMatrix(shear));

    // Matrix with determinant = 1 but not orthogonal
    Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
    invalid(0, 0) = 2.0;
    invalid(1, 1) = 0.5;

    EXPECT_FALSE(Transformation::isRotationMatrix(invalid));

    // Slight floating-point error (valid)
    Eigen::Matrix3d approximate = Eigen::Matrix3d::Identity();
    approximate(0, 0) += 1e-8;

    EXPECT_TRUE(Transformation::isRotationMatrix(approximate));

    // Significant deviation from orthogonality (invalid)
    Eigen::Matrix3d perturbed = Eigen::Matrix3d::Identity();
    perturbed(0, 0) += 1e-3;

    EXPECT_FALSE(Transformation::isRotationMatrix(perturbed));
}

TEST(RotationMatrixTestSuite, ComposeRotationsTest) {
    Eigen::Matrix3d R_ab = Transformation::rotationMatrixY(std::numbers::pi / 6);
    Eigen::Matrix3d R_bc = Transformation::rotationMatrixZ(std::numbers::pi / 9);

    Eigen::Matrix3d R_ac = Transformation::composeRotations(R_ab, R_bc);

    Eigen::Matrix3d R_ac_expected;
    R_ac_expected << std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::sin(std::numbers::pi / 6),
                     std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 9), 0,
                     -std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 6);

    EXPECT_TRUE(R_ac.isApprox(R_ac_expected, 1e-6));

    Eigen::Matrix3d R_de = Transformation::rotationMatrixX(std::numbers::pi / 6);
    Eigen::Matrix3d R_ef = Transformation::rotationMatrixZ(std::numbers::pi / 9);

    Eigen::Matrix3d R_df = Transformation::composeRotations(R_de, R_ef);

    Eigen::Matrix3d R_df_expected;
    R_df_expected << std::cos(std::numbers::pi / 9), -std::sin(std::numbers::pi / 9), 0,
                     std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::sin(std::numbers::pi / 6),
                     std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::cos(std::numbers::pi / 6);

    EXPECT_TRUE(R_df.isApprox(R_df_expected, 1e-6));

    Eigen::Matrix3d R_gh = Transformation::rotationMatrixZ(std::numbers::pi / 6);
    Eigen::Matrix3d R_hi = Transformation::rotationMatrixX(std::numbers::pi / 9);

    Eigen::Matrix3d R_gi = Transformation::composeRotations(R_gh, R_hi);

    Eigen::Matrix3d R_gi_expected;
    R_gi_expected << std::cos(std::numbers::pi / 6), -std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9),
                     std::sin(std::numbers::pi / 6), std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9),
                     0, std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 9);

    EXPECT_TRUE(R_gi.isApprox(R_gi_expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotateVectorTest) {
    Eigen::Vector3d v(1.0, 0.0, 0.0);

    Eigen::Matrix3d R = Transformation::rotationMatrixZ(std::numbers::pi / 2);
    Eigen::Vector3d v_rotated = Transformation::rotateVector(R, v);

    Eigen::Vector3d v_expected(0.0, 1.0, 0.0);
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));

    R = Transformation::rotationMatrixY(std::numbers::pi / 2);
    v_rotated = Transformation::rotateVector(R, v);

    v_expected << 0.0, 0.0, -1.0;
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));

    R = Transformation::rotationMatrixX(std::numbers::pi / 2);
    v_rotated = Transformation::rotateVector(R, v);

    v_expected << 1.0, 0.0, 0.0;
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}