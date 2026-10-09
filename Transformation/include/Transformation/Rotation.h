#ifndef ROTATION_H
#define ROTATION_H

#include <Eigen/Dense>

namespace Rotation {
    /**
     * @brief Provides a rotation matrix about x by the given theta
     * @param theta the angle to rotate about x by
     * @return the rotation matrix about x by the given theta
     */
    Eigen::Matrix3d rotationMatrixX(double theta);

    /**
     * @brief Provides a rotation matrix about y by the given theta
     * @param theta the angle to rotate about y by
     * @return the rotation matrix about y by the given theta
     */
    Eigen::Matrix3d rotationMatrixY(double theta);

    /**
     * @brief Provides a rotation matrix about z by the given theta
     * @param theta the angle to rotate about z by
     * @return the rotation matrix about z by the given theta
     */
    Eigen::Matrix3d rotationMatrixZ(double theta);

    /**
     * @brief Determines whether a given matrix is a valid rotation matrix
     * @param R the matrix to verify
     * @return true if the matrix is a valid rotation matrix, false otherwise
     */
    bool isRotationMatrix(const Eigen::Matrix3d& R);

    /**
     * @brief Composes two rotation matrices R_ab and R_bc to produce the resulting rotation matrix R_ac
     * @param R_ab the orientation of frame b expressed in frame a
     * @param R_bc the orientation of frame c expressed in frame b
     * @return the resulting rotation matrix R_ac
     */
    Eigen::Matrix3d composeRotations(const Eigen::Matrix3d& R_ab, const Eigen::Matrix3d& R_bc);

    /**
     * @brief Rotates a vector by a given rotation matrix
     * @param R the rotation matrix
     * @param v the vector to rotate
     * @return the rotated vector
     */
    Eigen::Vector3d rotateVector(const Eigen::Matrix3d& R, const Eigen::Vector3d& v);
}

#endif