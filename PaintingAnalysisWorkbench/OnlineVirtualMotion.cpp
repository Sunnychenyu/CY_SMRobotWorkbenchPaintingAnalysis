#include "OnlineVirtualMotion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kRotationStepSeconds = 0.002;
        constexpr double kAxisTransitionSeconds = 2.0;
    }

    void OnlineRefreshCadence::setFramesPerSecondLimit(int framesPerSecond)
    {
        m_framesPerSecondLimit = std::max(0, framesPerSecond);
    }

    double OnlineRefreshCadence::displayIntervalMilliseconds() const
    {
        return m_framesPerSecondLimit > 0 ? 1000.0 / m_framesPerSecondLimit : 0.0;
    }

    double onlineNextSampleTimeSeconds(double currentTime, double targetTime,
        double maximumStepSeconds)
    {
        // Keep the final short interval: it contributes its actual duration.
        return std::min(currentTime + maximumStepSeconds, targetTime);
    }

    void OnlineVirtualMotionClock::reset()
    {
        m_timeSeconds = m_rotationSeconds = m_gunSeconds = 0.0;
    }

    void OnlineVirtualMotionClock::advanceTo(double timeSeconds, bool rotating, bool movingGun)
    {
        const double elapsed = std::max(0.0, timeSeconds - m_timeSeconds);
        if(rotating) m_rotationSeconds += elapsed;
        if(movingGun) m_gunSeconds += elapsed;
        m_timeSeconds += elapsed;
    }

    double OnlineVirtualMotionClock::nextGunTurnTimeSeconds(double travelTimeSeconds) const
    {
        return m_timeSeconds + onlineNextGunTurnTimeSeconds(m_gunSeconds, travelTimeSeconds)
            - m_gunSeconds;
    }

    void OnlineSprayIntegrationSampling::configure(
        const Eigen::Vector3d& boundsMinimum, const Eigen::Vector3d& boundsMaximum,
        double minimumSigmaRadians)
    {
        m_bounds = Eigen::AlignedBox3d(boundsMinimum, boundsMaximum);
        m_surfaceDistance = {};
        m_queryDistance = -1.0;
        m_radiusMeters = 0.5 * (boundsMaximum - boundsMinimum).norm();
        // At least twelve integration samples across the narrowest +/-3 sigma
        // footprint. Also bound normal/visibility changes for broad patterns.
        m_maximumAngularStep = std::min(0.5 * std::max(minimumSigmaRadians, 1.0e-6),
            std::acos(-1.0) / 180.0);
    }

    void OnlineSprayIntegrationSampling::setSurfaceDistanceQuery(
        std::function<double(const Eigen::Vector3d&)> query)
    {
        m_surfaceDistance = std::move(query);
        m_queryDistance = -1.0;
    }

    double OnlineSprayIntegrationSampling::timeStepSeconds(
        const Eigen::Vector3d& gunPositionInWorkpiece,
        double rotationRpm, double gunSpeedMetersPerSecond) const
    {
        const double angularSpeed = std::abs(rotationRpm) * (2.0 * std::acos(-1.0) / 60.0);
        const double surfaceSpeed = m_radiusMeters * angularSpeed
            + std::abs(gunSpeedMetersPerSecond);
        if(surfaceSpeed == 0.0 && angularSpeed == 0.0) return 0.02;
        double distance = m_bounds.exteriorDistance(gunPositionInWorkpiece);
        if(m_surfaceDistance) {
            const double movement = (gunPositionInWorkpiece - m_queryPosition).norm();
            if(m_queryDistance < 0.0 || movement > 0.05 * m_queryDistance) {
                m_queryPosition = gunPositionInWorkpiece;
                m_queryDistance = m_surfaceDistance(gunPositionInWorkpiece);
                distance = m_queryDistance;
            } else {
                // Distance to a fixed surface is 1-Lipschitz. Reuse a conservative
                // lower bound until a meaningful movement warrants another BVH query.
                distance = m_queryDistance - movement;
            }
        }
        distance = std::max(distance, 0.001);
        const double angularRate = angularSpeed + surfaceSpeed / distance;
        return std::min(0.02, m_maximumAngularStep / angularRate);
    }

    void OnlineRandomWorkpieceRotation::reset(
        const Eigen::Vector3d& center, double rpm, std::uint32_t seed,
        const Eigen::Vector3d& initialAxis)
    {
        m_center = center;
        m_initialAxis = initialAxis;
        m_rpm = rpm;
        m_seed = seed;
        m_random.seed(seed);
        m_stepIndex = 0;
        m_axisSegment = 0;
        m_lastTimeSeconds = 0.0;
        m_rotation = Eigen::Quaterniond::Identity();
        m_fromAxis = initialAxis.squaredNorm() > 0.0 ? initialAxis.normalized() : randomAxis();
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
        if(timeSeconds < m_lastTimeSeconds) reset(m_center, m_rpm, m_seed, m_initialAxis);
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

    void OnlineReciprocatingGunMotion::reset(const Eigen::Vector3d& start,
        const Eigen::Vector3d& end, double speedMetersPerSecond, double activeTimeSeconds)
    {
        m_start = m_referencePosition = start;
        m_end = end;
        m_speed = speedMetersPerSecond;
        m_referenceTimeSeconds = activeTimeSeconds;
        m_targetIsEnd = true;
    }

    double OnlineReciprocatingGunMotion::entryTravelTimeSeconds() const
    {
        return onlineGunTravelTimeSeconds(m_referencePosition,
            m_targetIsEnd ? m_end : m_start, m_speed);
    }

    bool OnlineReciprocatingGunMotion::targetIsEndAt(double activeTimeSeconds) const
    {
        const double afterEntry = activeTimeSeconds - m_referenceTimeSeconds - entryTravelTimeSeconds();
        if(afterEntry < 0.0) return m_targetIsEnd;
        const double travel = onlineGunTravelTimeSeconds(m_start, m_end, m_speed);
        if(travel <= 0.0) return m_targetIsEnd;
        const auto legs = static_cast<std::uint64_t>(std::floor(afterEntry / travel));
        return legs % 2 == 0 ? !m_targetIsEnd : m_targetIsEnd;
    }

    void OnlineReciprocatingGunMotion::reconfigure(double activeTimeSeconds,
        const Eigen::Vector3d& start, const Eigen::Vector3d& end, double speedMetersPerSecond)
    {
        const Eigen::Vector3d position = positionAt(activeTimeSeconds);
        const bool samePath = start.isApprox(m_start, 1.0e-12) && end.isApprox(m_end, 1.0e-12);
        const bool targetIsEnd = samePath ? targetIsEndAt(activeTimeSeconds) : true;
        m_start = start;
        m_end = end;
        m_referencePosition = position;
        m_referenceTimeSeconds = activeTimeSeconds;
        m_speed = speedMetersPerSecond;
        m_targetIsEnd = targetIsEnd;
    }

    Eigen::Vector3d OnlineReciprocatingGunMotion::positionAt(double activeTimeSeconds) const
    {
        const double elapsed = std::max(0.0, activeTimeSeconds - m_referenceTimeSeconds);
        const double entry = entryTravelTimeSeconds();
        if(elapsed <= entry && entry > 0.0) {
            const Eigen::Vector3d target = m_targetIsEnd ? m_end : m_start;
            return m_referencePosition + (elapsed / entry) * (target - m_referencePosition);
        }
        const double travel = onlineGunTravelTimeSeconds(m_start, m_end, m_speed);
        if(travel <= 0.0) return m_referencePosition;
        // Reach the new path's target first, then reciprocate on that path.
        const double phase = std::fmod(elapsed - entry + (m_targetIsEnd ? travel : 0.0),
            2.0 * travel) / travel;
        return m_start + (1.0 - std::abs(1.0 - phase)) * (m_end - m_start);
    }

    double OnlineReciprocatingGunMotion::nextTurnTimeSeconds(double activeTimeSeconds) const
    {
        const double entry = entryTravelTimeSeconds();
        const double elapsed = activeTimeSeconds - m_referenceTimeSeconds;
        if(entry - elapsed > 8.0 * std::numeric_limits<double>::epsilon()
                * std::max(1.0, activeTimeSeconds)) return m_referenceTimeSeconds + entry;
        const double travel = onlineGunTravelTimeSeconds(m_start, m_end, m_speed);
        double turn = m_referenceTimeSeconds + entry + onlineNextGunTurnTimeSeconds(
            std::max(0.0, elapsed - entry), travel);
        if(turn - activeTimeSeconds <= 8.0 * std::numeric_limits<double>::epsilon()
                * std::max(1.0, activeTimeSeconds)) turn += travel;
        return turn;
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
        // Position depends only on physical time, never on display cadence.
        const double phase = travel > 0.0
            ? std::fmod(std::max(0.0, timeSeconds), 2.0 * travel) / travel : 0.0;
        const double fraction = 1.0 - std::abs(1.0 - phase);
        pose.translation() = center + startOffset
            + fraction * (endOffset - startOffset);
        return pose;
    }

    double onlineNextGunTurnTimeSeconds(double timeSeconds, double travelTimeSeconds)
    {
        if(travelTimeSeconds <= 0.0) return std::numeric_limits<double>::infinity();
        double turn = (std::floor(timeSeconds / travelTimeSeconds) + 1.0) * travelTimeSeconds;
        // Avoid a duplicate endpoint caused by division/multiplication roundoff.
        if(turn - timeSeconds <= 8.0 * std::numeric_limits<double>::epsilon()
                * std::max(1.0, timeSeconds)) turn += travelTimeSeconds;
        return turn;
    }
}
