#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <random>
#include <functional>

namespace robot_qt_viewer
{
    class OnlineRefreshCadence
    {
    public:
        void setFramesPerSecondLimit(int framesPerSecond);
        double displayIntervalMilliseconds() const;
    private:
        int m_framesPerSecondLimit = 0;
    };

    double onlineNextSampleTimeSeconds(double currentTime, double targetTime,
        double maximumStepSeconds = 0.02);

    // Each motion keeps its own phase while the shared simulation clock advances.
    // Pausing one motion must not rewind it or pause the other motion.
    class OnlineVirtualMotionClock
    {
    public:
        void reset();
        void advanceTo(double timeSeconds, bool rotating, bool movingGun);
        double timeSeconds() const { return m_timeSeconds; }
        double rotationSeconds() const { return m_rotationSeconds; }
        double gunSeconds() const { return m_gunSeconds; }
        double nextGunTurnTimeSeconds(double travelTimeSeconds) const;
    private:
        double m_timeSeconds = 0.0;
        double m_rotationSeconds = 0.0;
        double m_gunSeconds = 0.0;
    };

    class OnlineSprayIntegrationSampling
    {
    public:
        void configure(const Eigen::Vector3d& boundsMinimum,
            const Eigen::Vector3d& boundsMaximum, double minimumSigmaRadians);
        void setSurfaceDistanceQuery(std::function<double(const Eigen::Vector3d&)> query);
        double timeStepSeconds(const Eigen::Vector3d& gunPositionInWorkpiece,
            double rotationRpm, double gunSpeedMetersPerSecond) const;

    private:
        Eigen::AlignedBox3d m_bounds;
        double m_radiusMeters = 0.0;
        double m_maximumAngularStep = 0.01;
        std::function<double(const Eigen::Vector3d&)> m_surfaceDistance;
        mutable Eigen::Vector3d m_queryPosition = Eigen::Vector3d::Zero();
        mutable double m_queryDistance = -1.0;
    };

    class OnlineRandomWorkpieceRotation
    {
    public:
        void reset(const Eigen::Vector3d& center, double rpm, std::uint32_t seed,
            const Eigen::Vector3d& initialAxis = Eigen::Vector3d::Zero());
        Eigen::Isometry3d poseAt(double timeSeconds);
        const Eigen::Vector3d& axis() const { return m_axis; }
        std::uint32_t seed() const { return m_seed; }

    private:
        Eigen::Vector3d randomAxis();
        Eigen::Vector3d axisAt(double timeSeconds);
        Eigen::Vector3d m_center = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_initialAxis = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_fromAxis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d m_toAxis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d m_axis = Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond m_axisArc = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond m_rotation = Eigen::Quaterniond::Identity();
        std::mt19937 m_random;
        std::uint32_t m_seed = 0;
        std::uint64_t m_stepIndex = 0;
        std::uint64_t m_axisSegment = 0;
        double m_rpm = 0.0;
        double m_lastTimeSeconds = 0.0;
    };

    class OnlineReciprocatingGunMotion
    {
    public:
        void reset(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
            double speedMetersPerSecond, double activeTimeSeconds = 0.0);
        void reconfigure(double activeTimeSeconds, const Eigen::Vector3d& start,
            const Eigen::Vector3d& end, double speedMetersPerSecond);
        Eigen::Vector3d positionAt(double activeTimeSeconds) const;
        double nextTurnTimeSeconds(double activeTimeSeconds) const;
        const Eigen::Vector3d& startPosition() const { return m_start; }
        const Eigen::Vector3d& endPosition() const { return m_end; }
        double speedMetersPerSecond() const { return m_speed; }
    private:
        bool targetIsEndAt(double activeTimeSeconds) const;
        double entryTravelTimeSeconds() const;
        Eigen::Vector3d m_start = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_end = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_referencePosition = Eigen::Vector3d::Zero();
        double m_referenceTimeSeconds = 0.0;
        double m_speed = 0.0;
        bool m_targetIsEnd = true;
    };

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

    double onlineNextGunTurnTimeSeconds(double timeSeconds, double travelTimeSeconds);
}
