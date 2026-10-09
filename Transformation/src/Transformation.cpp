#include <Transformation/Transformation.h>

#include <cmath>

namespace Transformation {
    Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& translation) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = translation;

        return T;
    }

    Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d& point) {
        return (T * point.homogeneous()).head<3>();
    }

    Eigen::Matrix4d composeTransforms(const Eigen::Matrix4d& T_ab, const Eigen::Matrix4d& T_bc) {
        return T_ab * T_bc;
    }

    Eigen::Matrix4d inverseTransform(const Eigen::Matrix4d& T) {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d translation = T.block<3, 1>(0, 3);

        Eigen::Matrix4d T_inverse = Eigen::Matrix4d::Identity();
        T_inverse.block<3, 3>(0, 0) = R.transpose();
        T_inverse.block<3, 1>(0, 3) = -R.transpose() * translation;

        return T_inverse;
    }
}