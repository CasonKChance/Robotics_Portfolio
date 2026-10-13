#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Point2D.h"

#include <vector>

namespace Kinematics{
    /**
     * @brief Computes the position of the end effector of a planar robotic arm using forward kinematics.
     * @param jointAngles The angles of the joints in the robotic arm.
     * @param linkLengths The lengths of the links in the robotic arm.
     * @return The position of the end effector in the base frame.
     */
    Point2D forwardKinematics(const std::vector<double>& jointAngles, const std::vector<double>& linkLengths);
}

#endif