#include "OnlineVirtualMotion.h"

#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kRotationStepSeconds = 0.002;
        constexpr double kAxisTransitionSeconds = 2.0;
    }

    void OnlineRefreshCadence::recordWork(double milliseconds)
    {
        m_workMilliseconds = 0.75 * m_workMilliseconds
            + 0.25 * std::clamp(milliseconds, 1.0, 100.0);
    }

    int OnlineRefreshCadence::intervalMilliseconds() const
    {
        return static_cast<int>(std::ceil(displayIntervalMilliseconds()));
    }

    void OnlineRefreshCadence::setDisplayRefreshRate(double hertz)
    {
        m_displayMilliseconds = 1000.0 / (std::isfinite(hertz) && hertz > 0.0 ? hertz : 60.0);
    }

    double OnlineRefreshCadence::displayIntervalMilliseconds() const
    {
        return std::max(m_displayMilliseconds, m_workMilliseconds * 1.1);
    }

    double onlineNextSampleTimeSeconds(double currentTime, double targetTime,
        double maximumStepSeconds)
    {
        // Keep the final short interval: it contributes its actual duration.
        return std::min(currentTime + maximumStepSeconds, targetTime);
    }

    void OnlineSprayIntegrationSampling::configure(
        const Eigen::Vector3d& boundsMinimum, const Eigen::Vector3d& boundsMaximum,
        double minimumSigmaRadians)
    {
        m_bounds = Eigen::AlignedBox3d(boundsMinimum, boundsMaximum);
        m_radiusMeters = 0.5 * (boundsMaximum - boundsMinimum).norm();
        // At least twelve integration samples across the narrowest +/-3 sigma
        // footprint. Also bound normal/visibility changes for broad patterns.
        m_maximumAngularStep = std::min(0.5 * std::max(minimumSigmaRadians, 1.0e-6),
            std::acos(-1.0) / 180.0);
    }

    double OnlineSprayIntegrationSampling::timeStepSeconds(
        const Eigen::Vector3d& gunPositionInWorkpiece,
        double rotationRpm, double gunSpeedMetersPerSecond) const
    {
        const double angularSpeed = std::abs(rotationRpm) * (2.0 * std::acos(-1.0) / 60.0);
        const double surfaceSpeed = m_radiusMeters * angularSpeed
            + std::abs(gunSpeedMetersPerSecond);
        if(surfaceSpeed == 0.0 && angularSpeed == 0.0) return 0.02;
        // The nearest bounding-box distance is a conservative footprint scale.
        // A box can enclose empty space (e.g. a ring); avoid a zero step there.
        const double distance = std::max(m_bounds.exteriorDistance(gunPositionInWorkpiece), 0.001);
        const double angularRate = angularSpeed + surfaceSpeed / distance;
        return std::min(0.02, m_maximumAngularStep / angularRate);
    }

    void OnlineRandomWorkpieceRotation::reset(
        const Eigen::Vector3d& center, double rpm, std::uint32_t seed)
    {
        m_center = center;
        m_rpm = rpm;
        m_seed = seed;
        m_random.seed(seed);
        m_stepIndex = 0;
        m_axisSegment = 0;
        m_lastTimeSeconds = 0.0;
        m_rotation = Eigen::Quaterniond::Identity();
        m_fromAxis = randomAxis();
        m_toAxis = randomAxis();
        m_axisArc = Eigen::Quaterniond::FromTwoVectors(m_fromAxis, m_toAxis);
        m_axis = m_fromAxis;
    }

    Eigen::Vector3d OnlineRandomWorkpieceRotation::randomAxis()
    {
        // Uniform directions on the sphere, independent of GUI sampling cadence.
        const double z = 2.0 * static_cast<double>(m_random()) / m_random.max() - 1.0;
        const double azimuth = 2.0 * std::acos(-1.0)
            * static_cast<double>(m_random()) / m_random.max();
        const double radius = std::sqrt(std::max(0.0, 1.0 - z * z));
        return Eigen::Vector3d(radius * std::cos(azimuth),
            radius * std::sin(azimuth), z);
    }

    Eigen::Vector3d OnlineRandomWorkpieceRotation::axisAt(double timeSeconds)
    {
        const auto segment = static_cast<std::uint64_t>(timeSeconds / kAxisTransitionSeconds);
        while(m_axisSegment < segment) {
            m_fromAxis = m_toAxis;
            m_toAxis = randomAxis();
            m_axisArc = Eigen::Quaterniond::FromTwoVectors(m_fromAxis, m_toAxis);
            ++m_axisSegment;
        }
        const double u = (timeSeconds - m_axisSegment * kAxisTransitionSeconds)
            / kAxisTransitionSeconds;
        const double smooth = u * u * (3.0 - 2.0 * u);
        return (Eigen::Quaterniond::Identity().slerp(smooth, m_axisArc)
            * m_fromAxis).normalized();
    }

    Eigen::Isometry3d OnlineRandomWorkpieceRotation::poseAt(double timeSeconds)
    {
        timeSeconds = std::max(0.0, timeSeconds);
        if(timeSeconds < m_lastTimeSeconds) reset(m_center, m_rpm, m_seed);
        const double angularSpeed = m_rpm * (2.0 * std::acos(-1.0) / 60.0);
        const auto completeSteps = static_cast<std::uint64_t>(
            timeSeconds / kRotationStepSeconds);
        while(m_stepIndex < completeSteps) {
            const Eigen::Vector3d midpointAxis = axisAt(
                (m_stepIndex + 0.5) * kRotationStepSeconds);
            // Left multiplication integrates angular velocity in world axes.
            m_rotation = (Eigen::Quaterniond(Eigen::AngleAxisd(
                angularSpeed * kRotationStepSeconds, midpointAxis)) * m_rotation).normalized();
            ++m_stepIndex;
        }
        Eigen::Quaterniond rotation = m_rotation;
        const double tail = timeSeconds - m_stepIndex * kRotationStepSeconds;
        if(tail > 0.0) {
            rotation = (Eigen::Quaterniond(Eigen::AngleAxisd(angularSpeed * tail,
                axisAt(m_stepIndex * kRotationStepSeconds + 0.5 * tail))) * rotation).normalized();
        }
        // Do not commit the partial step: query frequency must not change motion.
        m_axis = axisAt(timeSeconds);
        m_lastTimeSeconds = timeSeconds;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.linear() = rotation.toRotationMatrix();
        pose.translation() = m_center - pose.linear() * m_center;
        return pose;
    }

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
