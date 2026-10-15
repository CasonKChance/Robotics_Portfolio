#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Point2D.h"

#include <Eigen/Dense>

#include <vector>

namespace Kinematics{
    /**
     * @brief Computes the position of the end effector of a planar robotic arm using forward kinematics.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @return The position of the end effector in the base frame.
     */
    Point2D forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths);

    /**
     * @brief Determines if a provided point is within the planar robotic arm's task space.
     * @param point The point to test.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @return True if the point is reachable, false otherwise.
     */
    bool isReachable(const Point2D& point, const std::vector<double>& linkLengths);

    /**
     * @brief Calculates the jacobian of a planar robot arm
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param axisOfRotation 3-vector of unit length describing which axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X)
     * @return the jacobian matrix.
     */
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation);

    /**
     * @brief Calculates the end-effector twist
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param jointVelocities The angular velocities of the joints in radians.
     * @param axisOfRotation 3-vector of unit length describing which axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X).
     * @return the end-effector twist
     */
    Eigen::VectorXd endEffectorTwist(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation);
}

#endif