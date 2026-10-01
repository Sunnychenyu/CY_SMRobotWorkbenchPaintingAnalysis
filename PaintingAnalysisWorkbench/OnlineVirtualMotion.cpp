#include "OnlineVirtualMotion.h"

#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    Eigen::Isometry3d onlineRotatingWorkpiecePose(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& axis,
        double rpm,
        double timeSeconds)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        const double radians = rpm * (2.0 * std::acos(-1.0) / 60.0)
            * timeSeconds;
        pose.linear() = Eigen::AngleAxisd(radians, axis.normalized()).toRotationMatrix();
        pose.translation() = center - pose.linear() * center;
        return pose;
    }

    double onlineGunTravelTimeSeconds(
        const Eigen::Vector3d& startOffset,
        const Eigen::Vector3d& endOffset,
        double speedMetersPerSecond)
    {
        const double distance = (endOffset - startOffset).norm();
        return distance > 0.0 && speedMetersPerSecond > 0.0
            ? distance / speedMetersPerSecond : 0.0;
    }

    Eigen::Isometry3d onlineVirtualGunPose(
        const Eigen::Vector3d& center,
        const Eigen::Vector3d& startOffset,
        const Eigen::Vector3d& endOffset,
        double speedMetersPerSecond,
        double timeSeconds)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = Eigen::AngleAxisd(
            std::acos(-1.0), Eigen::Vector3d::UnitY()).toRotationMatrix();
        const double travel = onlineGunTravelTimeSeconds(
            startOffset, endOffset, speedMetersPerSecond);
        const double fraction = travel > 0.0
            ? std::clamp(timeSeconds / travel, 0.0, 1.0) : 0.0;
        pose.translation() = center + startOffset
            + fraction * (endOffset - startOffset);
        return pose;
    }
}
