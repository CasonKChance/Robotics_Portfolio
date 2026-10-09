#include <Transformation/Transformation.h>

#include <cmath>

namespace Transformation {
    Eigen::Matrix3d rotationMatrixX(double theta) {
        Eigen::Matrix3d R;

        R << 1, 0, 0,
             0, std::cos(theta), -std::sin(theta),
             0, std::sin(theta), std::cos(theta);

        return R;
    }

    Eigen::Matrix3d rotationMatrixY(double theta) {
        Eigen::Matrix3d R;

        R << std::cos(theta), 0, std::sin(theta),
             0, 1, 0,
             -std::sin(theta), 0, std::cos(theta);

        return R;
    }

    Eigen::Matrix3d rotationMatrixZ(double theta) {
        Eigen::Matrix3d R;

        R << std::cos(theta), -std::sin(theta), 0,
             std::sin(theta), std::cos(theta), 0,
             0, 0, 1;

        return R;
    }

    bool isRotationMatrix(const Eigen::Matrix3d& R) {
        Eigen::Matrix3d R_T = R.transpose();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        double det_R = R.determinant();

        return (R*R_T).isApprox(I, 1e-6) && std::abs(det_R - 1.0) < 1e-6;
    }

    Eigen::Matrix3d composeRotations(const Eigen::Matrix3d& R_ab, const Eigen::Matrix3d& R_bc) {
        return R_ab * R_bc;
    }

    Eigen::Vector3d rotateVector(const Eigen::Matrix3d& R, const Eigen::Vector3d& v) {
        return R * v;
    }
}