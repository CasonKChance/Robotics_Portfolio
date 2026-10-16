#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Dense>

#include <vector>

namespace Kinematics{
    /**
     * @brief Computes the position of the end effector of a planar robotic arm using forward kinematics.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @return The position of the end effector in the base frame.
     */
    Eigen::Vector2d forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths);

    /**
     * @brief Determines if a provided point is within the planar robotic arm's task space.
     * @param point The point to test.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @return True if the point is reachable, false otherwise.
     */
    bool isReachable(const Eigen::Vector2d& point, const std::vector<double>& linkLengths);

    /**
     * @brief Calculates the spatial jacobian of a planar robot arm.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param axisOfRotation 3-vector of unit length describing which cardinal axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X).
     * @return The spatial jacobian matrix.
     */
    Eigen::Matrix<double, 6, Eigen::Dynamic> spatialJacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation);

    /**
     * @brief Calculates the linear velocity jacobian of a planar robot arm.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param axisOfRotation 3-vector of unit length describing which cardinal axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X).
     * @return The linear velocity jacobian matrix.
     */
    Eigen::Matrix<double, 3, Eigen::Dynamic> linearVelocityJacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation);

    /**
     * @brief Calculates the end-effector twist.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param jointVelocities The angular velocities of the joints in radians.
     * @param axisOfRotation 3-vector of unit length describing which cardinal axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X).
     * @return The end-effector twist.
     */
    Eigen::VectorXd endEffectorTwist(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation);

    /**
     * @brief Calculates the end-effector velocity.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @param jointVelocities The angular velocities of the joints in radians.
     * @param axisOfRotation 3-vector of unit length describing which cardinal axis the joints are rotating about (e.g. [1, 0, 0]ᵀ when rotating about the X).
     * @return The end-effector velocity.
     */    
    Eigen::Vector3d endEffectorVelocity(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation);

    /**
     * @brief Uses analytic inverse kinematics to calculate the possible joint angle solutions to place the end-effector at a target position.
     *        Only works for robotic arms with 1 or 2 links, robots with arbitrary n links where n > 2 must use the numerical inverse kinematics.
     * @param targetPosition The target position for the end-effector.
     * @return The set of solutions with each solution containing the necessary joint angles. Can return 0, 1, or 2 solutions.
     */
    std::vector<std::vector<double>> analyticInverseKinematics(const Eigen::Vector2d& targetPosition, const std::vector<double>& linkLengths);

    /**
     * @brief Uses numerical inverse kinematics to converge on a joint angles solution to place the end-effector at a target position.
     * @param targetPosition The target position for the end-effector.
     * @return The closest solution to the initial guess.
     */
    std::vector<double> numericalInverseKinematics(const Eigen::Vector2d& targetPosition, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation, const std::vector<double>& initialGuess, const Eigen::Vector2d& errorTolerance, const int maximumIterations);
}

#endif