#include "../../OnlineVirtualMotion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace
{
    bool reciprocatingGunTest()
    {
        using namespace robot_qt_viewer;
        const Eigen::Vector3d center(0.2, -0.1, 0.03);
        const Eigen::Vector3d start(-0.025, 0.01, 0.11);
        const double speed = 0.05;
        for(int axis = 0; axis < 3; ++axis) {
            for(double sign : { -1.0, 1.0 }) {
                const Eigen::Vector3d end = start + sign * 0.05 * Eigen::Vector3d::Unit(axis);
                const double travel = onlineGunTravelTimeSeconds(start, end, speed);
                for(int halfStep = 0; halfStep <= 8; ++halfStep) {
                    const double time = halfStep * travel * 0.5;
                    const double fraction = halfStep % 4 == 2 ? 1.0
                        : (halfStep % 2 == 1 ? 0.5 : 0.0);
                    const auto gun = onlineVirtualGunPose(center, start, end, speed, time);
                    if((gun.translation() - center - start - fraction * (end - start)).norm() > 1.0e-12
                        || (gun.linear() * Eigen::Vector3d::UnitZ()
                            + Eigen::Vector3d::UnitZ()).norm() > 1.0e-12) return false;
                    // Both motions use the same physical time, including at reversals.
                    const auto table = onlineRotatingWorkpiecePose(center,
                        Eigen::Vector3d::UnitZ(), 999.0, time);
                    const Eigen::Isometry3d relative = table.inverse() * gun;
                    if(((table * relative).matrix() - gun.matrix()).norm() > 1.0e-10) return false;
                }
                double time = 0.0, accumulated = 0.0;
                const double finish = 4.055 * travel;
                while(time < finish) {
                    const double target = std::min(finish, onlineNextGunTurnTimeSeconds(time, travel));
                    const double next = onlineNextSampleTimeSeconds(time, target, 0.37 * travel);
                    const auto a = onlineVirtualGunPose(center, start, end, speed, time);
                    const auto b = onlineVirtualGunPose(center, start, end, speed, next);
                    // No interval straddles a turn; distance equals speed * actual dt.
                    if(next <= time || std::abs((b.translation() - a.translation()).norm()
                        - speed * (next - time)) > 1.0e-11) return false;
                    accumulated += next - time;
                    time = next;
                }
                if(std::abs(accumulated - finish) > 1.0e-12) return false;
            }
        }
        OnlineSprayIntegrationSampling sampling;
        sampling.configure(Eigen::Vector3d::Constant(-0.05), Eigen::Vector3d::Constant(0.05), 0.028);
        const Eigen::Vector3d gun(0.0, 0.0, 0.17);
        const double combined = sampling.timeStepSeconds(gun, 999.0, speed);
        return combined < sampling.timeStepSeconds(gun, 999.0, 0.0)
            && combined < sampling.timeStepSeconds(gun, 0.0, speed);
    }

    bool editableGunMotionTest()
    {
        using namespace robot_qt_viewer;
        const Eigen::Vector3d start = Eigen::Vector3d::Zero();
        const Eigen::Vector3d end(0.04, 0.0, 0.0);
        OnlineReciprocatingGunMotion motion;
        motion.reset(start, end, 0.02);
        const auto halfway = motion.positionAt(1.0);
        motion.reconfigure(1.0, start, end, 0.04);
        if((motion.positionAt(1.0) - halfway).norm() > 1.0e-12
            || (motion.positionAt(1.25) - Eigen::Vector3d(0.03, 0.0, 0.0)).norm() > 1.0e-12
            || std::abs(motion.nextTurnTimeSeconds(1.0) - 1.5) > 1.0e-12) return false;
        const auto returning = motion.positionAt(1.75);
        motion.reconfigure(1.75, start, end, 0.02);
        if((motion.positionAt(1.75) - returning).norm() > 1.0e-12
            || (motion.positionAt(2.25) - Eigen::Vector3d(0.02, 0.0, 0.0)).norm() > 1.0e-12
            || std::abs(motion.nextTurnTimeSeconds(1.75) - 3.25) > 1.0e-12) return false;
        const auto beforeDirectionChange = motion.positionAt(2.25);
        const Eigen::Vector3d newEnd(0.0, 0.04, 0.0);
        motion.reconfigure(2.25, start, newEnd, 0.02);
        const double arrival = motion.nextTurnTimeSeconds(2.25);
        if((motion.positionAt(2.25) - beforeDirectionChange).norm() > 1.0e-12
            || std::abs((motion.positionAt(2.35) - beforeDirectionChange).norm() - 0.002) > 1.0e-12
            || (motion.positionAt(arrival) - newEnd).norm() > 1.0e-12
            || (motion.positionAt(arrival + 0.5) - Eigen::Vector3d(0.0, 0.03, 0.0)).norm() > 1.0e-12)
            return false;
        // The entry leg and every subsequent turn must split integration intervals.
        double time = 2.25;
        while(time < arrival + 5.0) {
            const double next = std::min({ time + 0.37, motion.nextTurnTimeSeconds(time), arrival + 5.0 });
            if(next <= time || std::abs((motion.positionAt(next) - motion.positionAt(time)).norm()
                    - 0.02 * (next - time)) > 1.0e-11) return false;
            time = next;
        }
        const Eigen::Vector3d newStart(0.01, 0.02, 0.03);
        motion.reset(newStart, newStart + end, 0.02, time);
        return (motion.positionAt(time) - newStart).norm() < 1.0e-12;
    }

    bool independentMotionClockTest()
    {
        using namespace robot_qt_viewer;
        const Eigen::Vector3d center(0.2, -0.1, 0.03);
        const Eigen::Vector3d start(0.0, 0.0, 0.1), end(0.04, 0.0, 0.1);
        OnlineVirtualMotionClock clock;
        clock.advanceTo(0.4, true, true);
        const auto frozenGun = onlineVirtualGunPose(center, start, end, 0.05, clock.gunSeconds());
        const auto initialTable = onlineRotatingWorkpiecePose(center, Eigen::Vector3d::UnitZ(),
            30.0, clock.rotationSeconds());
        clock.advanceTo(1.4, true, false);
        if(std::abs(clock.gunSeconds() - 0.4) > 1.0e-12
            || std::abs(clock.rotationSeconds() - 1.4) > 1.0e-12
            || (onlineVirtualGunPose(center, start, end, 0.05, clock.gunSeconds()).matrix()
                - frozenGun.matrix()).norm() > 1.0e-12) return false;
        const auto frozenTable = onlineRotatingWorkpiecePose(center, Eigen::Vector3d::UnitZ(),
            30.0, clock.rotationSeconds());
        if((frozenTable.matrix() - initialTable.matrix()).norm() < 0.1) return false;
        clock.advanceTo(1.6, false, true);
        if(std::abs(clock.gunSeconds() - 0.6) > 1.0e-12
            || (onlineRotatingWorkpiecePose(center, Eigen::Vector3d::UnitZ(), 30.0,
                    clock.rotationSeconds()).matrix() - frozenTable.matrix()).norm() > 1.0e-12
            || std::abs(clock.nextGunTurnTimeSeconds(0.8) - 1.8) > 1.0e-12) return false;
        clock.advanceTo(2.6, false, false);
        clock.advanceTo(2.8, false, true);
        if(std::abs(clock.gunSeconds() - 0.8) > 1.0e-12
            || std::abs(clock.nextGunTurnTimeSeconds(0.8) - 3.6) > 1.0e-12) return false;
        // Random-axis rotation uses its active time, even during powder-off motion.
        OnlineRandomWorkpieceRotation random;
        random.reset(center, 30.0, 12345);
        const auto frozenRandom = random.poseAt(clock.rotationSeconds());
        clock.advanceTo(3.0, false, true);
        if((random.poseAt(clock.rotationSeconds()).matrix() - frozenRandom.matrix()).norm() > 1.0e-12)
            return false;
        clock.advanceTo(3.2, true, false);
        if((random.poseAt(clock.rotationSeconds()).matrix() - frozenRandom.matrix()).norm() < 0.1)
            return false;
        clock.reset();
        return clock.timeSeconds() == 0.0 && clock.rotationSeconds() == 0.0 && clock.gunSeconds() == 0.0;
    }

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
        // Restart the velocity integration at a parameter-change boundary with
        // the current axis, so a speed change does not reset its direction.
        motion.reset(center, 30.0, 12345);
        motion.poseAt(2.731);
        const Eigen::Vector3d axisBeforeSpeedChange = motion.axis();
        motion.reset(center, 120.0, 12345, axisBeforeSpeedChange);
        if((motion.axis() - axisBeforeSpeedChange).norm() > 1.0e-12
            || (motion.poseAt(0.0).matrix() - Eigen::Isometry3d::Identity().matrix()).norm() > 1.0e-12)
            return false;
        const auto faster = motion.poseAt(0.002);
        if(std::abs(Eigen::AngleAxisd(faster.linear()).angle() - 4.0 * std::acos(-1.0) * 0.002) > 1.0e-9)
            return false;
        return true;
    }
}

int main()
{
    if(!editableGunMotionTest()) {
        std::cerr << "Live gun speed/path editing lost continuity or turn boundaries.\n";
        return 1;
    }
    if(!independentMotionClockTest()) {
        std::cerr << "Independent motion pause/resume or shared clock failed.\n";
        return 1;
    }
    if(!reciprocatingGunTest()) {
        std::cerr << "Reciprocating gun endpoints, turns or simultaneous rotation failed.\n";
        return 1;
    }
    if(!continuousDepositionTest()) {
        std::cerr << "Continuous deposition sampling, exposure or display cadence failed.\n";
        return 1;
    }
    if(!randomRotationTest()) {
        std::cerr << "Random rotation continuity, speed, center or cadence failed.\n";
        return 1;
    }
    robot_qt_viewer::OnlineRefreshCadence cadence;
    if(cadence.displayIntervalMilliseconds() != 0.0) return 1;
    cadence.setFramesPerSecondLimit(320);
    if(std::abs(cadence.displayIntervalMilliseconds() - 3.125) > 1.0e-6) return 1;
    cadence.setFramesPerSecondLimit(500);
    if(cadence.displayIntervalMilliseconds() != 2.0) return 1;
    cadence.setFramesPerSecondLimit(0);
    if(cadence.displayIntervalMilliseconds() != 0.0) return 1;
    // A nozzle inside a sphere's AABB can still be centimeters from its surface.
    robot_qt_viewer::OnlineSprayIntegrationSampling sphereSampling;
    sphereSampling.configure(Eigen::Vector3d::Constant(-0.05),
        Eigen::Vector3d::Constant(0.05), 0.028);
    const Eigen::Vector3d sphereGun(0.045, 0.045, 0.045);
    const double boxStep = sphereSampling.timeStepSeconds(sphereGun, 100.0, 0.0);
    sphereSampling.setSurfaceDistanceQuery([](const Eigen::Vector3d& position) {
        return std::abs(position.norm() - 0.05);
    });
    const double surfaceStep = sphereSampling.timeStepSeconds(sphereGun, 100.0, 0.0);
    if(surfaceStep <= boxStep * 10.0) return 1;
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
