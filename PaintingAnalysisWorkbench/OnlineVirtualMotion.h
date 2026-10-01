#pragma once

#include <Eigen/Geometry>

namespace robot_qt_viewer
{
    Eigen::Isometry3d onlineRotatingWorkpiecePose(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& axis,
        double rpm,
        double timeSeconds);

    Eigen::Isometry3d onlineVirtualGunPose(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& startOffset,
        const Eigen::Vector3d& endOffset,
        double speedMetersPerSecond,
        double timeSeconds);

    double onlineGunTravelTimeSeconds(
        const Eigen::Vector3d& startOffset,
        const Eigen::Vector3d& endOffset,
        double speedMetersPerSecond);
}
