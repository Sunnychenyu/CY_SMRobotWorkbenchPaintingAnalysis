#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <random>

namespace robot_qt_viewer
{
    class OnlineRefreshCadence
    {
    public:
        void recordWork(double milliseconds);
        void setDisplayRefreshRate(double hertz);
        double displayIntervalMilliseconds() const;
        int intervalMilliseconds() const;
    private:
        double m_workMilliseconds = 8.0;
        double m_displayMilliseconds = 1000.0 / 60.0;
    };

    double onlineNextSampleTimeSeconds(double currentTime, double targetTime,
        double maximumStepSeconds = 0.02);

    class OnlineSprayIntegrationSampling
    {
    public:
        void configure(const Eigen::Vector3d& boundsMinimum,
            const Eigen::Vector3d& boundsMaximum, double minimumSigmaRadians);
        double timeStepSeconds(const Eigen::Vector3d& gunPositionInWorkpiece,
            double rotationRpm, double gunSpeedMetersPerSecond) const;

    private:
        Eigen::AlignedBox3d m_bounds;
        double m_radiusMeters = 0.0;
        double m_maximumAngularStep = 0.01;
    };

    class OnlineRandomWorkpieceRotation
    {
    public:
        void reset(const Eigen::Vector3d& center, double rpm, std::uint32_t seed);
        Eigen::Isometry3d poseAt(double timeSeconds);
        const Eigen::Vector3d& axis() const { return m_axis; }
        std::uint32_t seed() const { return m_seed; }

    private:
        Eigen::Vector3d randomAxis();
        Eigen::Vector3d axisAt(double timeSeconds);
        Eigen::Vector3d m_center = Eigen::Vector3d::Zero();
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
