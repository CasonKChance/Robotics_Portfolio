#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <cmath>
#include <stdexcept>

namespace Kinematics {
    Eigen::Vector3d forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const std::vector<Eigen::Vector3d>& jointAxes)
    {
        if (jointAngles.size() != linkLengths.size()
            || jointAngles.size() != jointAxes.size()) {
            throw std::invalid_argument("The number of joints must be equal to the number of links and the number of joint axes.");
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < jointAngles.size(); ++i) {
            if (linkLengths[i] <= 0) {
                throw::std::invalid_argument("Links must have positive length.");
            }

            if (!jointAxes[i].allFinite()
                || jointAxes[i].norm() < 1e-9) {
                throw std::invalid_argument("Joint axes must be finite and nonzero.");
            }

            Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

            Eigen::Matrix3d rotation =
                Eigen::AngleAxisd(
                    jointAngles[i],
                    jointAxes[i].normalized()
                ).toRotationMatrix();

            transformation.block<3, 3>(0, 0) = rotation;

            transformation.block<3, 1>(0, 3) = rotation * Eigen::Vector3d(linkLengths[i], 0, 0);

            T *= transformation;
        }

        return T.block<3, 1>(0, 3);
    }

    bool isWithinReachBounds(const Eigen::Vector3d& point, const std::vector<double>& linkLengths)
    {
        if (linkLengths.empty()) {
            throw std::invalid_argument("The number of links must be greater than zero.");
        }

        if (!point.allFinite()) {
            throw std::invalid_argument(
                "Target position must be finite."
            );
        }

        double totalLength = 0.0;
        double longestLink = 0.0;

        for (size_t i = 0; i < linkLengths.size(); ++i) {
            if (linkLengths[i] <= 0) {
                throw::std::invalid_argument("Links must have positive length.");
            }

            totalLength += linkLengths[i];
            longestLink = std::max(longestLink, linkLengths[i]);
        }

        const double tolerance = 1e-9;
        const double distance = point.norm();

        const double maxReach = totalLength;
        const double minReach = std::max(0.0, 2.0 * longestLink - totalLength);

        return (distance >= minReach - tolerance) && (distance <= maxReach + tolerance);
    }

    // Eigen::Matrix<double, 6, Eigen::Dynamic> spatialJacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation)
    // {
    //     if (jointAngles.size() != linkLengths.size()) {
    //         throw std::invalid_argument("The number of joints must be equal to the number of links.");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     int nonZeroAxes = (axisOfRotation.x() != 0) + (axisOfRotation.y() != 0) + (axisOfRotation.z() != 0);
    //     if (nonZeroAxes != 1) {
    //         throw std::invalid_argument("One, and only one, axis of rotation may be defined");
    //     }

    //     Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, jointAngles.size());

    //     // Define the axes of the plane
    //     Eigen::Vector3d omega = axisOfRotation.normalized();

    //     Eigen::Vector3d axisU;
    //     Eigen::Vector3d axisV;
        
    //     if (omega.x()) {
    //         // If rotating about X, arm stretches along Y at zero configuration
    //         axisU << 0, 1, 0;
    //         axisV = omega.cross(axisU);
    //     } else {
    //         // If rotating about Y or Z, arm stretches along X at zero configuration
    //         axisU << 1, 0, 0;
    //         axisV = omega.cross(axisU);
    //     }

    //     Eigen::Vector3d q_current = Eigen::Vector3d::Zero();
    //     double totalAngle = 0.0;

    //     for (size_t i = 0; i < jointAngles.size(); ++i) {
    //         // Angular velocity is unit length about the axis of rotation
    //         Eigen::Vector3d omega_i = omega;

    //         // Linear velocity of joint in space frame
    //         Eigen::Vector3d v_i = -omega_i.cross(q_current);

    //         // Screw vector composed of angular velocity vector and linear velocity vector
    //         Eigen::Matrix<double, 6, 1> S;
    //         S << omega_i, v_i;

    //         // Add screw vector to jacobian matrix
    //         J.col(i) = S;

    //         // Update q_current for next iteration
    //         totalAngle += jointAngles[i];
    //         q_current += linkLengths[i] * (axisU * std::cos(totalAngle) + axisV * std::sin(totalAngle));
    //     }

    //     return J;
    // }

    // Eigen::Matrix<double, 3, Eigen::Dynamic> linearVelocityJacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation)
    // {
    //     if (jointAngles.size() != linkLengths.size()) {
    //         throw std::invalid_argument("The number of joints must be equal to the number of links.");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     int nonZeroAxes = (axisOfRotation.x() != 0) + (axisOfRotation.y() != 0) + (axisOfRotation.z() != 0);
    //     if (nonZeroAxes != 1) {
    //         throw std::invalid_argument("One, and only one, axis of rotation may be defined");
    //     }

    //     auto spatialJ = spatialJacobian(jointAngles, linkLengths, axisOfRotation);

    //     Eigen::Vector2d FK = forwardKinematics(jointAngles, linkLengths);

    //     Eigen::Vector3d omegaAxis = axisOfRotation.normalized();

    //     Eigen::Vector3d axisU;
    //     if (omegaAxis.x() != 0.0) {
    //         axisU << 0.0, 1.0, 0.0;
    //     } else {
    //         axisU << 1.0, 0.0, 0.0;
    //     }

    //     Eigen::Vector3d axisV = omegaAxis.cross(axisU);

    //     Eigen::Vector3d endEffectorPosition = axisU * FK.x() + axisV * FK.y();

    //     Eigen::Matrix<double, 3, Eigen::Dynamic> linearVelocityJ(3, jointAngles.size());

    //     for (Eigen::Index i = 0; i < spatialJ.cols(); ++i) {
    //         Eigen::Vector3d omega = spatialJ.col(i).head<3>();
    //         Eigen::Vector3d v = spatialJ.col(i).tail<3>();

    //         linearVelocityJ.col(i) = omega.cross(endEffectorPosition) + v;
    //     }

    //     return linearVelocityJ;
    // }

    // Eigen::VectorXd endEffectorTwist(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation)
    // {
    //     if (jointAngles.size() != linkLengths.size() || jointAngles.size() != static_cast<size_t>(jointVelocities.size())) {
    //         throw std::invalid_argument("The number of joints, links, and joint velocities must be equal.");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     auto J = spatialJacobian(jointAngles, linkLengths, axisOfRotation);

    //     return J * jointVelocities;
    // }

    // Eigen::Vector3d endEffectorVelocity(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation)
    // {
    //     if (jointAngles.size() != linkLengths.size() || jointAngles.size() != static_cast<size_t>(jointVelocities.size())) {
    //         throw std::invalid_argument("The number of joints, links, and joint velocities must be equal.");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     int nonZeroAxes = (axisOfRotation.x() != 0) + (axisOfRotation.y() != 0) + (axisOfRotation.z() != 0);
    //     if (nonZeroAxes != 1) {
    //         throw std::invalid_argument("One, and only one, axis of rotation may be defined");
    //     }

    //     auto J = linearVelocityJacobian(jointAngles, linkLengths, axisOfRotation);

    //     return J * jointVelocities;
    // }

    // std::vector<std::vector<double>> analyticInverseKinematics(const Eigen::Vector2d& targetPosition, const std::vector<double>& linkLengths)
    // {
    //     if (linkLengths.size() > 2) {
    //         throw std::invalid_argument("Analytic Inverse Kinematics with no orientation target only works for arms with 2 degrees of freedom or less");
    //     } else if (linkLengths.empty()) {
    //         throw std::invalid_argument("Robot must have at least 1 link");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     if (!isReachable(targetPosition, linkLengths)) {
    //         // No solution
    //         return {};
    //     } else if (linkLengths.size() == 1) {
    //         // Solution for single link robotic arm
    //         return std::vector<std::vector<double>>{{std::atan2(targetPosition.y(), targetPosition.x())}};
    //     }

    //     double tolerance = 1e-9;

    //     const double x = targetPosition.x();
    //     const double y = targetPosition.y();

    //     const double l1 = linkLengths[0];
    //     const double l2 = linkLengths[1];

    //     const double radius = std::hypot(x, y);

    //     // Singular solution cases
    //     if (std::abs(radius - (l1 + l2)) < tolerance) {
    //         // Target position lies on the outer border of the task space.
    //         return std::vector<std::vector<double>>{{std::atan2(y, x), 0.0}};
    //     } else if (std::abs(x) < tolerance && std::abs(y) < tolerance) {
    //         // Case where target position is [0, 0]ᵀ.
    //         // This case could techincally provide infinite solutions, instead we use [0, 𝛑].
    //         return std::vector<std::vector<double>>{{0.0, std::numbers::pi}};
    //     } else if (l1 > l2 && std::abs(radius - (l1 - l2)) < tolerance) {
    //         // Case where target position lies on the inner border of the task space and link1 is longer than link2.
    //         return std::vector<std::vector<double>>{{std::atan2(y, x), std::numbers::pi}};
    //     } else if (l1 < l2 && std::abs(radius - (l2 - l1)) < tolerance) {
    //         // Case where target position lies on the inner border of the task space and link2 is longer than link1.
    //         return std::vector<std::vector<double>>{{std::atan2(-y, -x), std::numbers::pi}};
    //     }

    //     double cosTheta2 = (std::pow(x, 2) + std::pow(y, 2) - std::pow(l1, 2) - std::pow(l2, 2))/(2*l1*l2);

    //     // Protect acos() against floating-point overshoot.
    //     cosTheta2 = std::clamp(cosTheta2, -1.0, 1.0);

    //     double leftyTheta2 = std::acos(cosTheta2);
    //     double rightyTheta2 = -std::acos(cosTheta2);

    //     double leftyTheta1 = std::atan2(y, x) - std::atan2(l2 * std::sin(leftyTheta2), l1 + l2 * std::cos(leftyTheta2));
    //     double rightyTheta1 = std::atan2(y, x) - std::atan2(l2 * std::sin(rightyTheta2), l1 + l2 * std::cos(rightyTheta2));

    //     return std::vector<std::vector<double>>{{leftyTheta1, leftyTheta2}, {rightyTheta1, rightyTheta2}};
    // }

    // std::vector<double> numericalInverseKinematics(const Eigen::Vector2d& targetPosition, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation, const std::vector<double>& initialGuess, const Eigen::Vector2d& errorTolerance, const int maximumIterations)
    // {
    //     if (linkLengths.empty()) {
    //         throw std::invalid_argument("Robot must have at least 1 link");
    //     }

    //     for (double length : linkLengths) {
    //         if (length <= 0.0) {
    //             throw std::invalid_argument("Link lengths must be greater than zero");
    //         }
    //     }

    //     if (!isReachable(targetPosition, linkLengths)) {
    //         // No solution
    //         return {};
    //     } else if (linkLengths.size() == 1) {
    //         // Solution for single link robotic arm
    //         return std::vector<double>{std::atan2(targetPosition.y(), targetPosition.x())};
    //     }

    //     double previousErrorNorm = std::numeric_limits<double>::max();
    //     double finalErrorNorm = 0.0;

    //     std::vector<double> currentGuess = initialGuess;
    //     for (int i = 0; i < maximumIterations; ++i) {
    //         Eigen::Vector2d currentPosition = forwardKinematics(currentGuess, linkLengths);

    //         Eigen::Vector2d currentError = targetPosition - currentPosition;
    //         if ((currentError.array().abs() <= errorTolerance.array()).all()) {
    //             return currentGuess;
    //         }

    //         finalErrorNorm = currentError.norm();
    //         if (i == maximumIterations - 2) {
    //             previousErrorNorm = finalErrorNorm;
    //         }

    //         auto jacobian = linearVelocityJacobian(currentGuess, linkLengths, axisOfRotation);
            
    //         Eigen::MatrixXd planarJacobian(2, linkLengths.size());

    //         // Select the two dimensions perpendicular to the rotation axis
    //         if (std::abs(axisOfRotation.x()) > 0.5) {
    //             // Rotation about X: YZ plane
    //             planarJacobian.row(0) = jacobian.row(1);
    //             planarJacobian.row(1) = jacobian.row(2);
    //         } else if (std::abs(axisOfRotation.y()) > 0.5) {
    //             // Rotation about Y: XZ plane
    //             planarJacobian.row(0) = jacobian.row(0);
    //             planarJacobian.row(1) = jacobian.row(2);
    //         } else {
    //             // Rotation about Z: XY plane
    //             planarJacobian.row(0) = jacobian.row(0);
    //             planarJacobian.row(1) = jacobian.row(1);
    //         }

    //         auto jacobianPseudoInverse = planarJacobian.completeOrthogonalDecomposition().pseudoInverse();

    //         Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(currentGuess.data(), currentGuess.size());
    //         q += jacobianPseudoInverse * currentError;

    //         currentGuess.assign(q.data(), q.data() + q.size());
    //     }

    //     double errorReduction = previousErrorNorm - finalErrorNorm;
    //     if (errorReduction > 1e-5) {
    //         return currentGuess;
    //     }

    //     return {};
    // }
}