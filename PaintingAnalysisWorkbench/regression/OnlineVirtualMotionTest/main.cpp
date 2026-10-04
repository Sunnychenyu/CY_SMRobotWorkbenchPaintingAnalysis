#include "../../OnlineVirtualMotion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace
{
    bool continuousDepositionTest()
    {
        using namespace robot_qt_viewer;
        const double sigma = 0.028;
        const Eigen::Vector3d minimum(-0.05, -0.05, -0.05);
        const Eigen::Vector3d maximum = -minimum;
        OnlineSprayIntegrationSampling sampling;
        sampling.configure(minimum, maximum, sigma);
        const Eigen::Vector3d gun(0.0, 0.0, 0.17);
        const double slow = sampling.timeStepSeconds(gun, 60.0, 0.0);
        const double fast = sampling.timeStepSeconds(gun, 600.0, 0.0);
        if(!(fast > 0.0 && fast < slow && slow <= 0.02)
            || std::abs(fast - sampling.timeStepSeconds(gun, -600.0, 0.0)) > 1.0e-12
            || sampling.timeStepSeconds(gun, 0.0, 0.0) != 0.02) return false;
        OnlineSprayIntegrationSampling narrow;
        narrow.configure(minimum, maximum, sigma * 0.5);
        if(narrow.timeStepSeconds(gun, 600.0, 0.0) >= fast
            || sampling.timeStepSeconds(Eigen::Vector3d(0.0, 0.0, 0.08), 600.0, 0.0)
                >= fast) return false;

        // One full revolution at 600 rpm: every surface location must receive
        // the same Gaussian exposure, even with slow or irregular GUI callbacks.
        std::array<double, 180> reference{};
        for(int cadence = 0; cadence < 3; ++cadence) {
            std::array<double, 180> exposure{};
            const double omega = 20.0 * std::acos(-1.0);
            double time = 0.0;
            double target = 0.0;
            double totalTime = 0.0;
            std::size_t frame = 0;
            while(target < 0.1) {
                const double frameStep = cadence == 0 ? 0.004
                    : cadence == 1 ? 0.02 : (frame++ % 2 == 0 ? 0.003 : 0.037);
                target = std::min(target + frameStep, 0.1);
                while(time < target) {
                    const double next = onlineNextSampleTimeSeconds(time, target, fast);
                    if(next <= time || next - time > fast + 1.0e-12) return false;
                    for(std::size_t vertex = 0; vertex < exposure.size(); ++vertex) {
                        const double angle = 2.0 * std::acos(-1.0) * vertex / exposure.size();
                        const double delta = std::remainder(omega * time - angle, 2.0 * std::acos(-1.0));
                        exposure[vertex] += std::exp(-0.5 * delta * delta / (sigma * sigma))
                            * (next - time);
                    }
                    totalTime += next - time;
                    time = next;
                }
            }
            if(std::abs(totalTime - 0.1) > 1.0e-12) return false;
            const double expected = std::sqrt(2.0 * std::acos(-1.0)) * sigma / omega;
            for(std::size_t vertex = 0; vertex < exposure.size(); ++vertex) {
                if(std::abs(exposure[vertex] - expected) > expected * 0.02) return false;
                if(cadence == 0) reference[vertex] = exposure[vertex];
                else if(std::abs(exposure[vertex] - reference[vertex]) > expected * 0.02)
                    return false;
            }
        }
        // A fast translating gun must also move by less than half a spot sigma.
        const double movingStep = sampling.timeStepSeconds(gun, 0.0, 2.0);
        if(movingStep * 2.0 > 0.5 * sigma * 0.12 + 1.0e-12) return false;
        const double tail = onlineNextSampleTimeSeconds(0.1, 0.1000001, fast);
        return tail == 0.1000001;
    }

    bool randomRotationTest()
    {
        using robot_qt_viewer::OnlineRandomWorkpieceRotation;
        const Eigen::Vector3d center(0.2, -0.1, 0.03);
        OnlineRandomWorkpieceRotation motion;
        motion.reset(center, 30.0, 12345);
        const Eigen::Vector3d initialAxis = motion.axis();
        const double angularSpeed = std::acos(-1.0);
        Eigen::Matrix3d previous = Eigen::Matrix3d::Identity();
        for(int step = 1; step <= 4000; ++step) {
            const auto pose = motion.poseAt(step * 0.002);
            const Eigen::Matrix3d rotation = pose.linear();
            if((pose * center - center).norm() > 1.0e-12
                || (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1.0e-12
                || std::abs(rotation.determinant() - 1.0) > 1.0e-12
                || std::abs(Eigen::AngleAxisd(rotation * previous.transpose()).angle()
                    - angularSpeed * 0.002) > 1.0e-9) return false;
            previous = rotation;
        }
        if((initialAxis - motion.axis()).norm() < 0.1) return false;
        for(double boundary : { 2.0, 4.0, 6.0 }) {
            const auto before = motion.poseAt(boundary - 1.0e-5);
            const Eigen::Vector3d beforeAxis = motion.axis();
            const auto after = motion.poseAt(boundary + 1.0e-5);
            if((beforeAxis - motion.axis()).norm() > 1.0e-6
                || (before.linear() - after.linear()).norm() > 1.0e-4) return false;
        }

        // Query cadence and a pause at the same simulation time must not change
        // the integrated pose; only complete fixed physical steps are committed.
        OnlineRandomWorkpieceRotation fast, slow;
        fast.reset(center, 30.0, 12345);
        slow.reset(center, 30.0, 12345);
        for(double time = 0.003; time < 8.013; time += 0.003) fast.poseAt(time);
        for(double time = 0.020; time < 8.013; time += 0.020) slow.poseAt(time);
        const auto expected = motion.poseAt(8.013);
        const auto fastPose = fast.poseAt(8.013);
        const auto slowPose = slow.poseAt(8.013);
        if((fastPose.matrix() - expected.matrix()).norm() > 1.0e-12
            || (slowPose.matrix() - expected.matrix()).norm() > 1.0e-12
            || (fast.poseAt(8.013).matrix() - fastPose.matrix()).norm() > 1.0e-12
            || (fast.poseAt(8.033).matrix() - slow.poseAt(8.033).matrix()).norm() > 1.0e-12)
            return false;
        motion.reset(center, 30.0, 54321);
        if((motion.poseAt(8.013).matrix() - expected.matrix()).norm() < 0.1) return false;
        motion.reset(center, 0.0, 12345);
        if((motion.poseAt(8.013).matrix() - Eigen::Isometry3d::Identity().matrix()).norm()
            > 1.0e-12) return false;
        motion.reset(center, -30.0, 12345);
        const auto backwards = motion.poseAt(0.002);
        const Eigen::AngleAxisd delta(backwards.linear());
        if(delta.axis().dot(initialAxis) > -0.999) return false;
        return true;
    }
}

int main()
{
    if(!continuousDepositionTest()) {
        std::cerr << "Continuous deposition sampling, exposure or display cadence failed.\n";
        return 1;
    }
    if(!randomRotationTest()) {
        std::cerr << "Random rotation continuity, speed, center or cadence failed.\n";
        return 1;
    }
    robot_qt_viewer::OnlineRefreshCadence cadence;
    for(int i = 0; i < 30; ++i) cadence.recordWork(2.0);
    if(cadence.intervalMilliseconds() >= 20) return 1;
    const int fastInterval = cadence.intervalMilliseconds();
    cadence.setDisplayRefreshRate(320.0);
    if(std::abs(cadence.displayIntervalMilliseconds() - 3.125) > 1.0e-6) return 1;
    if(cadence.intervalMilliseconds() >= fastInterval) return 1;
    cadence.setDisplayRefreshRate(60.0);
    if(cadence.intervalMilliseconds() != fastInterval) return 1;
    for(int i = 0; i < 30; ++i) cadence.recordWork(60.0);
    if(cadence.intervalMilliseconds() <= fastInterval) return 1;
    double time = 0.0;
    double exposure = 0.0;
    for(double target : { 0.003, 0.008, 0.055, 0.151 }) {
        while(time < target) {
            const double next = robot_qt_viewer::onlineNextSampleTimeSeconds(time, target);
            if(next <= time || next - time > 0.020000001) return 1;
            exposure += next - time;
            time = next;
        }
    }
    if(std::abs(exposure - 0.151) > 1.0e-12) return 1;
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
