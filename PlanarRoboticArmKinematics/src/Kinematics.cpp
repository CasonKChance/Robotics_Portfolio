#include <PlanarRoboticArmKinematics/Kinematics.h>

#include <cmath>
#include <stdexcept>

namespace Kinematics {
    Point2D forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths) {
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

    bool isReachable(const Point2D& point, const std::vector<double>& linkLengths) {
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
}