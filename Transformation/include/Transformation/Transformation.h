#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <Eigen/Dense>

namespace Transformations {
    /**
     * @brief Provides a rotation maxtrix about x by the given theta
     * @param theta the angle to rotate about x by
     */
    Eigen::Matrix3d rotationMatrixX(double theta);

    /**
     * @brief Provides a rotation maxtrix about y by the given theta
     * @param theta the angle to rotate about y by
     */
    Eigen::Matrix3d rotationMatrixY(double theta);

    /**
     * @brief Provides a rotation maxtrix about z by the given theta
     * @param theta the angle to rotate about z by
     */
    Eigen::Matrix3d rotationMatrixZ(double theta);

    /**
     * @brief Determines whether a given matrix is a valid rotation matrix
     * @param R the matrix to verify
     */
    bool isRotationMatrix(const Eigen::Matrix3d& R);
}

#endif