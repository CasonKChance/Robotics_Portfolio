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

        return Point2D(x, y);
    }
}