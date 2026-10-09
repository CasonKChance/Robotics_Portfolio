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
}