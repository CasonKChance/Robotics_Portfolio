#include <Transformation/Transformation.h>

namespace Transformation {
    Eigen::Matrix3d rotationMatrixX(double theta) {
        Eigen::Matrix3d R;

        R << 1, 0, 0,
             0, cos(theta), -sin(theta),
             0, sin(theta), cos(theta);

        return R;
    }

    Eigen::Matrix3d rotationMatrixY(double theta) {
        Eigen::Matrix3d R;

        R << cos(theta), 0, sin(theta),
             0, 1, 0,
             -sin(theta), 0, cos(theta);

        return R;
    }

    Eigen::Matrix3d rotationMatrixZ(double theta) {
        Eigen::Matrix3d R;

        R << cos(theta), -sin(theta), 0,
             sin(theta), cos(theta), 0,
             0, 0, 1;

        return R;
    }

    bool isRotationMatrix(const Eigen::Matrix3d& R) {
        Eigen::Matrix3d R_T = R.transpose();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        double det_R = R.determinant();

        return (R*R_T).isApprox(I) && (det_R - 1) < 1e-6 && (det_R - 1) > -1e-6;
    }
}