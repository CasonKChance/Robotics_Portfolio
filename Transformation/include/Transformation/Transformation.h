#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Eigen/Dense>

namespace Transformation {
    /**
     * @brief Creates a 4x4 transformation matrix from a 3x3 rotation matrix and a 3D translation vector.
     * @param R The 3x3 rotation matrix.
     * @param translation The 3D translation vector.
     * @return A 4x4 transformation matrix.
     */
    Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& translation);

    /**
     * @brief Transforms a 3D point using a 4x4 transformation matrix.
     * @param T The 4x4 transformation matrix.
     * @param point The 3D point to transform.
     * @return The transformed 3D point.
     */
    Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T, const Eigen::Vector3d& point);

    /**
     * @brief Composes two 4x4 transformation matrices.
     * @param T_ab The first transformation matrix (the orientation of frame B expressed in frame A).
     * @param T_bc The second transformation matrix (the orientation of frame C expressed in frame B).
     * @return The composed transformation matrix (frame C expressed in frame A).
     */
    Eigen::Matrix4d composeTransforms(const Eigen::Matrix4d& T_ab, const Eigen::Matrix4d& T_bc);
}

#endif