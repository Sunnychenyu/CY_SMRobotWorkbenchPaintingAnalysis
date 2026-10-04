#include "OnlineThicknessPredictionJobController.h"
#include "OnlineVirtualMotion.h"
#include "CoatingAnalysisPanel.h"
#include "CoatingAnalysisInfoPanel.h"
#include "CoatingAnalysisTreeModel.h"

#include <QApplication>
#include <QGuiApplication>
#include <QPersistentModelIndex>
#include <QLabel>
#include <QToolButton>
#include <QThread>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool uiRefreshTest(bool benchmarkOnly)
    {
        robot_qt_viewer::CoatingAnalysisPanel panel;
        robot_qt_viewer::CoatingAnalysisInfoPanel info;
        robot_qt_viewer::CoatingAnalysisTreeModel tree;
        robot_qt_viewer::CoatingAnalysisViewModel view;
        view.mode = robot_qt_viewer::CoatingAnalysisMode::Online;
        view.hasModel = true;
        view.hasResult = true;
        view.workpieces.push_back({ QStringLiteral("plate"), QStringLiteral("Plate") });
        view.selectedWorkpieceId = QStringLiteral("plate");
        tree.setWorkpieces(view.workpieces, view.selectedWorkpieceId);
        robot_qt_viewer::CoatingAnalysisInfoView readouts;
        readouts.hasModel = true;
        readouts.modelInfo.vertexCount = 1584740;
        readouts.hasThickness = true;
        readouts.predictionTiming.valid = true;
        readouts.predictionTiming.predictionVertexCount = 1584740;
        panel.setLanguageCode(QStringLiteral("zh-CN"));
        info.setLanguageCode(QStringLiteral("zh-CN"));
        info.applyInfo(readouts);
        const auto* modelReadout = info.findChild<QLabel*>(QStringLiteral("ModelReadout"));
        const auto* thicknessReadout = info.findChild<QLabel*>(QStringLiteral("ThicknessReadout"));
        const QString modelText = modelReadout->text();
        QToolButton* thicknessSection = nullptr;
        for(auto* button : info.findChildren<QToolButton*>()) {
            if(button->property("sourceTitle").toString() == QStringLiteral("Thickness")) {
                thicknessSection = button;
                break;
            }
        }
        if(thicknessSection == nullptr) return false;
        thicknessSection->setChecked(true);
        const auto* content = thicknessReadout->parentWidget();
        tree.setThickness(true, readouts.thicknessMetrics);
        QPersistentModelIndex modelIndex(tree.index(0, 0, tree.index(2, 0, QModelIndex())));
        int resets = 0;
        QObject::connect(&tree, &QAbstractItemModel::modelReset, [&]() { ++resets; });
        std::vector<double> durations;
        for(int i = 1; i <= 100; ++i) {
            readouts.thicknessMetrics.maxThickness = i * 1.0e-6;
            readouts.thicknessMetrics.averageThickness = i * 0.5e-6;
            readouts.predictionElapsedSeconds = i * 0.2;
            const auto start = std::chrono::steady_clock::now();
            if(benchmarkOnly) {
                panel.applyViewModel(view);
                info.applyInfo(readouts);
            } else {
                info.applyThicknessInfo(readouts);
            }
            tree.setThickness(true, readouts.thicknessMetrics);
            durations.push_back(std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count());
        }
        std::sort(durations.begin(), durations.end());
        double total = 0.0;
        for(double duration : durations) total += duration;
        std::cout << "UI readouts: meanMs=" << total / durations.size()
            << ", p95Ms=" << durations[94] << ", maxMs=" << durations.back()
            << ", treeResets=" << resets << '\n';
        // Structural insertion/removal must also preserve the workpiece selection.
        tree.setThickness(false, readouts.thicknessMetrics);
        tree.setThickness(true, readouts.thicknessMetrics);
        if(!benchmarkOnly && (resets != 0 || !modelIndex.isValid()
            || modelReadout->text() != modelText || !thicknessSection->isChecked()
            || content->isHidden() || !thicknessReadout->text().contains(QStringLiteral("\u5e73\u5747")))) {
            std::cerr << "Thickness refresh lost model selection, readings, language or expansion.\n";
            return false;
        }
        info.setLanguageCode(QStringLiteral("en"));
        if(!thicknessReadout->text().contains(QStringLiteral("Average:50.0"))) {
            std::cerr << "English readout failed: " << thicknessReadout->text().toStdString() << '\n';
            return false;
        }
        info.setLanguageCode(QStringLiteral("zh-CN"));
        if(!thicknessSection->isChecked() || content->isHidden()) {
            std::cerr << "Language switch reset the thickness section.\n";
            return false;
        }
        panel.setOnlineRefreshStatistics(200.0, 200.0, 10.0, false, 200.0, 30.0, 50.0, 320.0);
        const auto* cadenceReadout = panel.findChild<QLabel*>(QStringLiteral("onlineRefreshStatistics"));
        if(!cadenceReadout->text().contains(QStringLiteral("P95"))
            || !cadenceReadout->text().contains(QStringLiteral("50.00 ms"))) {
            std::cerr << "Cadence readout failed: " << cadenceReadout->text().toStdString() << '\n';
            return false;
        }
        return true;
    }

    spraythickness::ThicknessPredictionTask task()
    {
        spraythickness::ThicknessPredictionTask value;
        value.options.enableBvhOcclusion = true;
        value.options.enableHistoryCorrection = false;
        value.options.base.trajectorySamplingMode = spraythickness::TrajectorySamplingMode::OriginalPoints;
        value.options.deposition.sigmaPhiRadians = 0.2;
        value.options.deposition.sigmaPsiRadians = 0.2;
        for(double z : { 0.0, -0.015 }) {
            const auto offset = static_cast<std::uint32_t>(value.workpiece.samples.size());
            for(int y = -1; y <= 1; ++y) {
                for(int x = -1; x <= 1; ++x) {
                    sprayworkpiece::SurfaceSample sample;
                    sample.position = Eigen::Vector3d(0.01 * x, 0.01 * y, z);
                    sample.normal = Eigen::Vector3d::UnitZ();
                    value.workpiece.samples.push_back(sample);
                }
            }
            for(std::uint32_t y = 0; y < 2; ++y) {
                for(std::uint32_t x = 0; x < 2; ++x) {
                    const auto a = offset + 3 * y + x;
                    value.workpiece.triangleIndices.insert(value.workpiece.triangleIndices.end(),
                        { a, a + 1, a + 3, a + 1, a + 4, a + 3 });
                }
            }
        }
        return value;
    }

    spraytrajectory::SprayTrajectory interval(int index)
    {
        spraytrajectory::SprayTrajectory result;
        spraytrajectory::SpraySegment segment;
        for(int endpoint : { index, index + 1 }) {
            spraytrajectory::SprayPathPoint point;
            point.time = endpoint * 0.02;
            point.tcpPose.translation() = Eigen::Vector3d(0, 0, 0.12);
            point.sprayEnabled = true;
            segment.points.push_back(point);
        }
        result.segments.push_back(std::move(segment));
        return result;
    }

    robot_qt_viewer::OnlinePredictionFrame frameAt(double time)
    {
        robot_qt_viewer::OnlinePredictionFrame frame;
        frame.timeSeconds = time;
        frame.tablePose.translation().x() = time;
        frame.gunPose.translation() = Eigen::Vector3d(time, 0, 0.12);
        return frame;
    }

    bool randomRotationDepositionTest()
    {
        robot_qt_viewer::OnlineRandomWorkpieceRotation motion;
        motion.reset(Eigen::Vector3d(0, 0, -0.0075), 1.0, 12345);
        const Eigen::Isometry3d gun = [] {
            Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
            value.translation() = Eigen::Vector3d(0, 0, 0.12);
            return value;
        }();
        std::vector<spraytrajectory::SprayPathPoint> points;
        std::vector<robot_qt_viewer::OnlinePredictionFrame> frames;
        const auto addPoint = [&](double time) {
            robot_qt_viewer::OnlinePredictionFrame frame;
            frame.timeSeconds = time;
            frame.tablePose = motion.poseAt(time);
            frame.gunPose = gun;
            frame.rotationAxis = motion.axis();
            spraytrajectory::SprayPathPoint point;
            point.time = time;
            point.tcpPose = frame.tablePose.inverse() * gun;
            point.sprayEnabled = true;
            points.push_back(point);
            frames.push_back(frame);
        };
        for(int i = 0; i <= 405; ++i) addPoint(i * 0.01);
        addPoint(4.055);
        const auto run = [&](std::size_t batchIntervals) {
            robot_qt_viewer::OnlineThicknessPredictionJobController job;
            std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> result;
            robot_qt_viewer::OnlinePredictionFrame delivered;
            QString error;
            QObject::connect(&job,
                &robot_qt_viewer::OnlineThicknessPredictionJobController::predictionFailed,
                [&](const QString& message) { error = message; });
            QObject::connect(&job,
                &robot_qt_viewer::OnlineThicknessPredictionJobController::fieldReady,
                [&](const auto& field, const auto&, const auto& frame) {
                    result = field;
                    delivered = frame;
                    job.acknowledgeFrame(frame.id);
                });
            job.begin(task());
            for(std::size_t first = 0; first + 1 < points.size();) {
                const auto last = std::min(first + batchIntervals, points.size() - 1);
                spraytrajectory::SprayTrajectory trajectory;
                spraytrajectory::SpraySegment segment;
                segment.points.assign(points.begin() + first, points.begin() + last + 1);
                trajectory.segments.push_back(std::move(segment));
                job.append(std::move(trajectory), frames[last]);
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
                while(error.isEmpty() && (!result || result->timing.sprayPointCount < last)
                    && std::chrono::steady_clock::now() < deadline) {
                    QGuiApplication::processEvents();
                    QThread::msleep(1);
                }
                if(!error.isEmpty() || !result || result->timing.sprayPointCount != last
                    || (delivered.tablePose.matrix() - frames[last].tablePose.matrix()).norm() > 1.0e-12
                    || (delivered.rotationAxis - frames[last].rotationAxis).norm() > 1.0e-12
                    || std::abs(delivered.timeSeconds - points[last].time) > 1.0e-12) {
                    std::cerr << "Random rotation streaming failed: " << error.toStdString() << '\n';
                    return std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>();
                }
                first = last;
            }
            return result;
        };
        const auto whole = run(points.size() - 1);
        const auto streamed = run(17);
        if(!whole || !streamed || whole->thicknessMeters(4) <= 0.0
            || whole->thicknessMeters(13) != 0.0
            || streamed->thicknessMeters(13) != 0.0) return false;
        for(std::size_t i = 0; i < whole->size(); ++i) {
            const double a = whole->thicknessMeters(i);
            const double b = streamed->thicknessMeters(i);
            if(!std::isfinite(a) || !std::isfinite(b) || a < 0.0 || b < 0.0
                || std::abs(a - b) > std::max(1.0e-12, std::abs(a) * 1.0e-4)) return false;
        }
        std::cout << "Random rotation: changing-axis relative poses, upper-plane deposition, "
            "lower-center occlusion, irregular tail and batch invariance passed.\n";
        return true;
    }

    int benchmark()
    {
        // Steady online batches on the same planar geometry, varying sample density.
        // This measures the compute/result pipeline, not viewer rendering FPS.
        for(int side : { 205, 337, 891 }) {
            auto input = task();
            input.workpiece.samples.clear();
            input.workpiece.triangleIndices.clear();
            for(double z : { 0.0, -0.015 }) {
                const auto offset = static_cast<std::uint32_t>(input.workpiece.samples.size());
                for(int y = 0; y < side; ++y) {
                    for(int x = 0; x < side; ++x) {
                        sprayworkpiece::SurfaceSample sample;
                        sample.position = Eigen::Vector3d(0.1 * x / (side - 1) - 0.05,
                            0.1 * y / (side - 1) - 0.05, z);
                        sample.normal = Eigen::Vector3d::UnitZ();
                        input.workpiece.samples.push_back(sample);
                    }
                }
                const auto right = offset + side - 1;
                const auto top = offset + (side - 1) * side;
                input.workpiece.triangleIndices.insert(input.workpiece.triangleIndices.end(),
                    { offset, right, top, right, top + side - 1, top });
            }
            const auto count = input.workpiece.samples.size();
            robot_qt_viewer::OnlineThicknessPredictionJobController job;
            int completed = 0;
            double backendMs = 0.0, gpuMs = 0.0, statisticsMs = 0.0;
            QString error;
            QObject::connect(&job, &robot_qt_viewer::OnlineThicknessPredictionJobController::predictionFailed,
                [&](const QString& message) { error = message; });
            QObject::connect(&job, &robot_qt_viewer::OnlineThicknessPredictionJobController::fieldReady,
                [&](const auto& result, const auto&, const auto& frame) {
                    if(completed >= 3) {
                        backendMs += result->timing.backendTotalMilliseconds;
                        gpuMs += result->timing.pureGpuMilliseconds;
                        statisticsMs += frame.statisticsMilliseconds;
                    }
                    ++completed;
                    job.acknowledgeFrame(frame.id);
                    if(completed < 15) job.append(interval(completed), frameAt((completed + 1) * 0.02));
                });
            job.begin(std::move(input));
            job.append(interval(0), frameAt(0.02));
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            while(completed < 15 && error.isEmpty() && std::chrono::steady_clock::now() < deadline) {
                QGuiApplication::processEvents();
                QThread::msleep(1);
            }
            if(completed != 15 || !error.isEmpty()) return 1;
            std::cout << "Online benchmark: vertices=" << count << ", measuredBatches=12"
                << ", meanBackendMs=" << backendMs / 12.0 << ", meanGpuMs=" << gpuMs / 12.0
                << ", meanStatisticsMs=" << statisticsMs / 12.0 << '\n';
        }
        return 0;
    }
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    if(argc > 1 && std::string(argv[1]) == "--benchmark") return benchmark();
    if(argc > 1 && std::string(argv[1]) == "--ui-benchmark") return uiRefreshTest(true) ? 0 : 1;
    if(!uiRefreshTest(false)) return 1;
    robot_qt_viewer::OnlineThicknessPredictionJobController job;
    std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> final;
    QString error;
    int deliveries = 0;
    bool invalid = false;
    robot_qt_viewer::OnlinePredictionFrame displayedFrame;
    std::chrono::steady_clock::time_point firstAcknowledgedAt{};
    bool prefetched = false;
    bool autoAcknowledge = false;
    QObject::connect(&job, &robot_qt_viewer::OnlineThicknessPredictionJobController::predictionFailed,
        [&](const QString& message) { error = message; });
    QObject::connect(&job, &robot_qt_viewer::OnlineThicknessPredictionJobController::fieldReady,
        [&](const auto& result, const robot_qt_viewer::ThicknessUniformityStatistics& uniformity,
            const robot_qt_viewer::OnlinePredictionFrame& frame) {
            if(final && result->timing.sprayPointCount < final->timing.sprayPointCount) invalid = true;
            if(!uniformity.valid || uniformity.totalVertexCount != 18) invalid = true;
            final = result;
            displayedFrame = frame;
            if(deliveries == 1 && firstAcknowledgedAt != std::chrono::steady_clock::time_point())
                prefetched = frame.computedAt != std::chrono::steady_clock::time_point()
                    && frame.computedAt < firstAcknowledgedAt;
            if(std::abs(frame.tablePose.translation().x() - frame.timeSeconds) > 1.0e-12
                || std::abs((frame.tablePose.inverse() * frame.gunPose)
                    .translation().z() - 0.12) > 1.0e-12) invalid = true;
            ++deliveries;
            if(autoAcknowledge) job.acknowledgeFrame(frame.id);
        });
    const auto waitFor = [&](std::size_t count) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while(error.isEmpty() && (!final || final->timing.sprayPointCount < count)
            && std::chrono::steady_clock::now() < deadline) {
            QGuiApplication::processEvents();
            QThread::msleep(1);
        }
        return error.isEmpty() && final && final->timing.sprayPointCount == count;
    };
    job.begin(task());
    job.append(interval(0), frameAt(0.02));
    if(!waitFor(1)) { std::cerr << error.toStdString() << '\n'; return 1; }
    const double first = final->thicknessMeters(4);
    const auto firstFrameId = displayedFrame.id;
    for(int index = 1; index < 50; ++index)
        job.append(interval(index), frameAt((index + 1) * 0.02));
    const auto processFor = [](int milliseconds) {
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
        while(std::chrono::steady_clock::now() < end) {
            QGuiApplication::processEvents();
            QThread::msleep(1);
        }
    };
    job.acknowledgeFrame(firstFrameId + 1);
    processFor(100);
    if(deliveries != 1 || final->timing.sprayPointCount != 1) return 1;
    firstAcknowledgedAt = std::chrono::steady_clock::now();
    autoAcknowledge = true;
    job.acknowledgeFrame(firstFrameId);
    if(!waitFor(50) || first <= 0.0 || invalid
        || std::abs(final->thicknessMeters(4) - 50.0 * first) > first * 0.01
        || final->thicknessMeters(13) != 0.0
        || std::abs(displayedFrame.timeSeconds - 1.0) > 1.0e-12
        || deliveries < 2) return 1;
    if(!prefetched) {
        std::cerr << "Next batch was not computed while the previous frame awaited presentation.\n";
        return 1;
    }
    // Starting another session discards stale display snapshots, not intervals
    // belonging to the new session. The previous shared result stays immutable.
    const auto saved = final;
    autoAcknowledge = false;
    firstAcknowledgedAt = {};
    const auto staleFrameId = displayedFrame.id;
    job.reset();
    job.begin(task());
    final.reset();
    job.append(interval(0), frameAt(0.02));
    if(!waitFor(1) || saved->timing.sprayPointCount != 50
        || std::abs(final->thicknessMeters(4) - first) > first * 1.0e-4) return 1;
    const auto resetFrameId = displayedFrame.id;
    auto tail = interval(1);
    tail.segments.front().points.back().time = 0.055;
    job.append(std::move(tail), frameAt(0.055));
    job.acknowledgeFrame(staleFrameId);
    processFor(30);
    if(final->timing.sprayPointCount != 1) return 1;
    job.acknowledgeFrame(resetFrameId);
    if(!waitFor(2) || std::abs(displayedFrame.timeSeconds - 0.055) > 1.0e-12
        || std::abs(final->thicknessMeters(4) - 2.75 * first) > first * 0.01) return 1;
    // Reset also releases a worker waiting for an unpresented frame.
    job.reset();
    std::cout << "Streaming: presentation backpressure; matching poses; 50 intervals conserved; "
        "lower plane occluded; irregular tail and stale-frame reset passed; deliveries="
        << deliveries << '\n';
    return invalid || !randomRotationDepositionTest() ? 1 : 0;
}
