#include "../../OnlineVirtualMotion.h"

#include <cmath>
#include <iostream>

int main()
{
    const Eigen::Vector3d center(0.2, -0.1, 0.03);
    const auto rotating = robot_qt_viewer::onlineRotatingWorkpiecePose(
        center, Eigen::Vector3d::UnitZ(), 30.0, 0.5);
    const Eigen::Vector3d rotated = rotating
        * (center + Eigen::Vector3d(0.01, 0.0, 0.0));
    if((rotated - center - Eigen::Vector3d(0.0, 0.01, 0.0)).norm() > 1.0e-9) {
        std::cerr << "Workpiece rotation did not preserve its center.\n";
        return 1;
    }

    const Eigen::Vector3d start(-0.05, 0.0, 0.11);
    const Eigen::Vector3d end(0.05, 0.0, 0.11);
    const double duration = robot_qt_viewer::onlineGunTravelTimeSeconds(
        start, end, 0.05);
    const auto midpoint = robot_qt_viewer::onlineVirtualGunPose(
        center, start, end, 0.05, duration * 0.5);
    if(std::abs(duration - 2.0) > 1.0e-9
        || (midpoint.translation() - center
            - Eigen::Vector3d(0.0, 0.0, 0.11)).norm() > 1.0e-9
        || (midpoint.linear() * Eigen::Vector3d::UnitZ()
            + Eigen::Vector3d::UnitZ()).norm() > 1.0e-9) {
        std::cerr << "Gun motion or spray direction is incorrect.\n";
        return 1;
    }
    return 0;
}
