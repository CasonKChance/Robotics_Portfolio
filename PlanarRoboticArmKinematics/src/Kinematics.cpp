#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <cmath>
#include <stdexcept>

namespace Kinematics {
    Point2D forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths)
    {
        if (jointAngles.size() != linkLengths.size()) {
            throw std::invalid_argument("The number of joints must be equal to the number of links.");
        }

        double x = 0.0;
        double y = 0.0;
        double totalAngle = 0.0;
        for (size_t i = 0; i < jointAngles.size(); ++i) {
            totalAngle += jointAngles[i];
            x += linkLengths[i] * std::cos(totalAngle);
            y += linkLengths[i] * std::sin(totalAngle);
        }

        return Point2D{x, y};
    }

    bool isReachable(const Point2D& point, const std::vector<double>& linkLengths)
    {
        if (linkLengths.empty()) {
            throw std::invalid_argument("The number of links must be greater than zero.");
        }

        const double tolerance = 1e-9;
        const double distance = std::hypot(point.x, point.y);

        double totalLength = 0.0;
        double longestLink = 0.0;
        for (const double length : linkLengths) {
            if (length < 0.0) {
                throw std::invalid_argument("Link lengths must be non-negative.");
            }

            totalLength += length;
            longestLink = std::max(longestLink, length);
        }

        const double maxReach = totalLength;
        const double minReach = std::max(0.0, 2.0 * longestLink - totalLength);

        return (distance >= minReach - tolerance) && (distance <= maxReach + tolerance);
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::Vector3d& axisOfRotation)
    {
        if (jointAngles.size() != linkLengths.size()) {
            throw std::invalid_argument("The number of joints must be equal to the number of links.");
        }

        int nonZeroAxes = (axisOfRotation.x() != 0) + (axisOfRotation.y() != 0) + (axisOfRotation.z() != 0);
        if (nonZeroAxes != 1) {
            throw std::invalid_argument("One, and only one, axis of rotation may be defined");
        }

        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, jointAngles.size());

        // Define the axes of the plane
        Eigen::Vector3d omega = axisOfRotation.normalized();

        Eigen::Vector3d axisU;
        Eigen::Vector3d axisV;
        
        if (omega.x()) {
            // If rotating about X, arm stretches along Y at zero configuration
            axisU << 0, 1, 0;
            axisV = omega.cross(axisU);
        } else {
            // If rotating about Y or Z, arm stretches along X at zero configuration
            axisU << 1, 0, 0;
            axisV = omega.cross(axisU);
        }

        Eigen::Vector3d q_current = Eigen::Vector3d::Zero();
        double totalAngle = 0.0;

        for (size_t i = 0; i < jointAngles.size(); ++i) {
            // Angular velocity is unit length about the axis of rotation
            Eigen::Vector3d omega_i = omega;

            // Linear velocity of joint in space frame
            Eigen::Vector3d v_i = -omega_i.cross(q_current);

            // Screw vector composed of angular velocity vector and linear velocity vector
            Eigen::Matrix<double, 6, 1> S;
            S << omega_i, v_i;

            // Add screw vector to jacobian matrix
            J.col(i) = S;

            // Update q_current for next iteration
            totalAngle += jointAngles[i];
            q_current += linkLengths[i] * (axisU * std::cos(totalAngle) + axisV * std::sin(totalAngle));
        }

        return J;
    }

    Eigen::VectorXd endEffectorTwist(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation)
    {
        if (jointAngles.size() != linkLengths.size() || jointAngles.size() != static_cast<size_t>(jointVelocities.size())) {
            throw std::invalid_argument("The number of joints, links, and joint velocities must be equal.");
        }

        auto J = jacobian(jointAngles, linkLengths, axisOfRotation);

        return J * jointVelocities;
    }

    Eigen::Vector3d endEffectorVelocity(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths, const Eigen::VectorXd& jointVelocities, const Eigen::Vector3d& axisOfRotation)
    {
        if (jointAngles.size() != linkLengths.size() || jointAngles.size() != static_cast<size_t>(jointVelocities.size())) {
            throw std::invalid_argument("The number of joints, links, and joint velocities must be equal.");
        }

        int nonZeroAxes = (axisOfRotation.x() != 0) + (axisOfRotation.y() != 0) + (axisOfRotation.z() != 0);
        if (nonZeroAxes != 1) {
            throw std::invalid_argument("One, and only one, axis of rotation may be defined");
        }

        Eigen::VectorXd spacialTwist = endEffectorTwist(jointAngles, linkLengths, jointVelocities, axisOfRotation);

        Point2D FK = forwardKinematics(jointAngles, linkLengths);
        Eigen::Vector3d endEffectorPosition;
        Eigen::Vector3d axisOfRotationNormalized = axisOfRotation.normalized();
        if (axisOfRotationNormalized.x()) {
            endEffectorPosition << 0, FK.x, FK.y;
        } else if (axisOfRotation.y()) {
            endEffectorPosition << FK.x, 0, FK.y;
        } else {
            endEffectorPosition << FK.x, FK.y, 0;
        }

        Eigen::Vector3d omega = spacialTwist.head(3);
        Eigen::Vector3d v = spacialTwist.tail(3);

        return omega.cross(endEffectorPosition) + v;
    }
}