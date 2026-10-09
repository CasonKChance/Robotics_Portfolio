#include <Transformation/Rotation.h>
#include <Transformation/Transformation.h>

#include <gtest/gtest.h>

#include <numbers>

TEST(RotationMatrixTestSuite, RotationMatrixXTest) {
    Eigen::Matrix3d R = Rotation::rotationMatrixX(0.0);

    EXPECT_TRUE(Rotation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Rotation::rotationMatrixX(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 1, 0, 0,
                 0, 0, -1,
                 0, 1, 0;

    EXPECT_TRUE(Rotation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Rotation::rotationMatrixX(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << 1, 0, 0,
                 0, -1, 0,
                 0, 0, -1;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Rotation::rotationMatrixX(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 1, 0, 0,
                 0, 0, 1,
                 0, -1, 0;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotationMatrixYTest) {
    Eigen::Matrix3d R = Rotation::rotationMatrixY(0.0);

    EXPECT_TRUE(Rotation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Rotation::rotationMatrixY(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 0, 0, 1,
                 0, 1, 0,
                 -1, 0, 0;

    EXPECT_TRUE(Rotation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Rotation::rotationMatrixY(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << -1, 0, 0,
                 0, 1, 0,
                 0, 0, -1;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Rotation::rotationMatrixY(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 0, 0, -1,
                 0, 1, 0,
                 1, 0, 0;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotationMatrixZTest) {
    Eigen::Matrix3d R = Rotation::rotationMatrixZ(0.0);

    EXPECT_TRUE(Rotation::isRotationMatrix(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    Eigen::Matrix3d R2 = Rotation::rotationMatrixZ(std::numbers::pi/2);
    Eigen::Matrix3d R2Expected;
    R2Expected << 0, -1, 0,
                 1, 0, 0,
                 0, 0, 1;

    EXPECT_TRUE(Rotation::isRotationMatrix(R2));
    EXPECT_TRUE(R2.isApprox(R2Expected, 1e-6));

    Eigen::Matrix3d R3 = Rotation::rotationMatrixZ(std::numbers::pi);
    Eigen::Matrix3d R3Expected;
    R3Expected << -1, 0, 0,
                 0, -1, 0,
                 0, 0, 1;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R3));
    EXPECT_TRUE(R3.isApprox(R3Expected, 1e-6));

    Eigen::Matrix3d R4 = Rotation::rotationMatrixZ(-std::numbers::pi/2);
    Eigen::Matrix3d R4Expected;
    R4Expected << 0, 1, 0,
                 -1, 0, 0,
                 0, 0, 1;
                 
    EXPECT_TRUE(Rotation::isRotationMatrix(R4));
    EXPECT_TRUE(R4.isApprox(R4Expected, 1e-6));
}

TEST(RotationMatrixTestSuite, IsRotationMatrixTest) {
    // Identity matrix (valid)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(Rotation::isRotationMatrix(I));

    // Rotation about multiple axes (valid)
    Eigen::Matrix3d R =
        Rotation::rotationMatrixX(std::numbers::pi / 4) *
        Rotation::rotationMatrixY(std::numbers::pi / 3) *
        Rotation::rotationMatrixZ(std::numbers::pi / 6);

    EXPECT_TRUE(Rotation::isRotationMatrix(R));

    // Zero matrix (invalid)
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    EXPECT_FALSE(Rotation::isRotationMatrix(zero));

    // Scaling matrix (invalid - not orthogonal)
    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
    scale(0, 0) = 2.0;

    EXPECT_FALSE(Rotation::isRotationMatrix(scale));

    // Reflection matrix (invalid - determinant = -1)
    Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
    reflection(0, 0) = -1.0;

    EXPECT_FALSE(Rotation::isRotationMatrix(reflection));

    // Shear matrix (invalid - not orthogonal)
    Eigen::Matrix3d shear = Eigen::Matrix3d::Identity();
    shear(0, 1) = 0.5;

    EXPECT_FALSE(Rotation::isRotationMatrix(shear));

    // Matrix with determinant = 1 but not orthogonal
    Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
    invalid(0, 0) = 2.0;
    invalid(1, 1) = 0.5;

    EXPECT_FALSE(Rotation::isRotationMatrix(invalid));

    // Slight floating-point error (valid)
    Eigen::Matrix3d approximate = Eigen::Matrix3d::Identity();
    approximate(0, 0) += 1e-8;

    EXPECT_TRUE(Rotation::isRotationMatrix(approximate));

    // Significant deviation from orthogonality (invalid)
    Eigen::Matrix3d perturbed = Eigen::Matrix3d::Identity();
    perturbed(0, 0) += 1e-3;

    EXPECT_FALSE(Rotation::isRotationMatrix(perturbed));
}

TEST(RotationMatrixTestSuite, ComposeRotationsTest) {
    Eigen::Matrix3d R_ab = Rotation::rotationMatrixY(std::numbers::pi / 6);
    Eigen::Matrix3d R_bc = Rotation::rotationMatrixZ(std::numbers::pi / 9);

    Eigen::Matrix3d R_ac = Rotation::composeRotations(R_ab, R_bc);

    Eigen::Matrix3d R_ac_expected;
    R_ac_expected << std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::sin(std::numbers::pi / 6),
                     std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 9), 0,
                     -std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 6);

    EXPECT_TRUE(R_ac.isApprox(R_ac_expected, 1e-6));

    Eigen::Matrix3d R_de = Rotation::rotationMatrixX(std::numbers::pi / 6);
    Eigen::Matrix3d R_ef = Rotation::rotationMatrixZ(std::numbers::pi / 9);

    Eigen::Matrix3d R_df = Rotation::composeRotations(R_de, R_ef);

    Eigen::Matrix3d R_df_expected;
    R_df_expected << std::cos(std::numbers::pi / 9), -std::sin(std::numbers::pi / 9), 0,
                     std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::sin(std::numbers::pi / 6),
                     std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::cos(std::numbers::pi / 6);

    EXPECT_TRUE(R_df.isApprox(R_df_expected, 1e-6));

    Eigen::Matrix3d R_gh = Rotation::rotationMatrixZ(std::numbers::pi / 6);
    Eigen::Matrix3d R_hi = Rotation::rotationMatrixX(std::numbers::pi / 9);

    Eigen::Matrix3d R_gi = Rotation::composeRotations(R_gh, R_hi);

    Eigen::Matrix3d R_gi_expected;
    R_gi_expected << std::cos(std::numbers::pi / 6), -std::sin(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), std::sin(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9),
                     std::sin(std::numbers::pi / 6), std::cos(std::numbers::pi / 6) * std::cos(std::numbers::pi / 9), -std::cos(std::numbers::pi / 6) * std::sin(std::numbers::pi / 9),
                     0, std::sin(std::numbers::pi / 9), std::cos(std::numbers::pi / 9);

    EXPECT_TRUE(R_gi.isApprox(R_gi_expected, 1e-6));
}

TEST(RotationMatrixTestSuite, RotateVectorTest) {
    Eigen::Vector3d v(1.0, 0.0, 0.0);

    Eigen::Matrix3d R = Rotation::rotationMatrixZ(std::numbers::pi / 2);
    Eigen::Vector3d v_rotated = Rotation::rotateVector(R, v);

    Eigen::Vector3d v_expected(0.0, 1.0, 0.0);
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));

    R = Rotation::rotationMatrixY(std::numbers::pi / 2);
    v_rotated = Rotation::rotateVector(R, v);

    v_expected << 0.0, 0.0, -1.0;
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));

    R = Rotation::rotationMatrixX(std::numbers::pi / 2);
    v_rotated = Rotation::rotateVector(R, v);

    v_expected << 1.0, 0.0, 0.0;
    EXPECT_TRUE(v_rotated.isApprox(v_expected, 1e-6));
}

TEST(TransformMatrixTestSuite, MakeTransformTest) {
    Eigen::Matrix3d R = Rotation::rotationMatrixZ(std::numbers::pi / 2);
    Eigen::Vector3d translation(1.0, 2.0, 3.0);

    Eigen::Matrix4d T = Transformation::makeTransform(R, translation);

    Eigen::Matrix4d T_expected = Eigen::Matrix4d::Identity();
    T_expected.block<3, 3>(0, 0) = R;
    T_expected.block<3, 1>(0, 3) = translation;

    EXPECT_TRUE(T.isApprox(T_expected, 1e-6));
}

TEST(TransformMatrixTestSuite, TransformPointTest) {
    Eigen::Matrix3d R = Rotation::rotationMatrixZ(std::numbers::pi / 2);
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Matrix4d T = Transformation::makeTransform(R, translation);

    Eigen::Vector3d point(1.0, 0.0, 0.0);
    Eigen::Vector3d point_transformed = Transformation::transformPoint(T, point);

    Eigen::Vector3d point_expected = R * point + translation;
    EXPECT_TRUE(point_transformed.isApprox(point_expected, 1e-6));
}

TEST(TransformMatrixTestSuite, ComposeTransformsTest) {
    Eigen::Matrix3d R1 = Rotation::rotationMatrixZ(std::numbers::pi / 4);
    Eigen::Vector3d translation1(1.0, 0.0, 0.0);
    Eigen::Matrix4d T1 = Transformation::makeTransform(R1, translation1);

    Eigen::Matrix3d R2 = Rotation::rotationMatrixX(std::numbers::pi / 4);
    Eigen::Vector3d translation2(0.0, 1.0, 0.0);
    Eigen::Matrix4d T2 = Transformation::makeTransform(R2, translation2);

    Eigen::Matrix4d T_composed = Transformation::composeTransforms(T1, T2);
    Eigen::Matrix4d T_expected = T1 * T2;

    EXPECT_TRUE(T_composed.isApprox(T_expected, 1e-6));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}