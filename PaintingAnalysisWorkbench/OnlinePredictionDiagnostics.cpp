#include "OnlinePredictionDiagnostics.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>

namespace robot_qt_viewer
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        constexpr double kHeartbeatMilliseconds = 20.0;
        constexpr std::size_t kMaximumQueuedRows = 4096;
        constexpr std::size_t kStageCount = static_cast<std::size_t>(OnlineDiagnosticStage::Count);
        const std::array<const char*, kStageCount> kColumns{
            "input_gap_ms", "input_queue_ms", "backend_other_cpu_ms", "upload_ms",
            "dispatch_cpu_ms", "gpu_compute_ms", "readback_including_gpu_wait_ms",
            "result_conversion_ms", "gpu_timer_read_ms", "statistics_ms", "mapping_ms",
            "gui_delivery_queue_ms", "previous_frame_presentation_wait_ms", "pacing_ms", "gui_result_handling_ms",
            "gui_pose_ms", "gui_overlay_ms", "gui_info_ms", "paint_queue_ms", "scene_update_ms",
            "draw_submission_ms", "swap_callback_wait_ms", "gui_timer_lateness_ms",
            "gui_startup_ms", "context_startup_ms", "backend_initialization_ms",
            "gpu_display_copy_ms", "gpu_statistics_ms", "gpu_completion_wait_ms", "gpu_scalar_draw_ms"
        };
        const std::array<const char*, kStageCount> kNames{
            "Input sampling / submission", "Input backlog / worker queue", "Backend CPU preparation",
            "GPU input upload", "Compute submission", "GPU compute", "GPU wait / thickness readback",
            "Thickness conversion", "GPU timer result wait", "Uniformity statistics", "Cloud mapping",
            "GUI event delivery", "Previous frame presentation", "Display pacing", "GUI result handling", "Pose submission",
            "Cloud conversion / upload / copy", "Information / legend update", "Repaint scheduling wait", "Scene update",
            "Draw submission", "Qt swap / compositor wait", "GUI event loop delay",
            "Model / GUI startup", "Compute context startup", "BVH / initial GPU preparation",
            "GPU display snapshot", "GPU statistics reduction", "GPU completion wait", "GPU scalar drawing"
        };

        double milliseconds(Clock::duration duration)
        {
            return std::chrono::duration<double, std::milli>(duration).count();
        }

        QString csvText(QString value)
        {
            value.replace(QLatin1Char('"'), QStringLiteral("\"\""));
            return QLatin1Char('"') + value + QLatin1Char('"');
        }
    }

    struct OnlinePredictionDiagnostics::Impl
    {
        struct Row
        {
            enum class Action { Begin, Record, End } action = Action::Record;
            QString kind;
            QString note;
            OnlineDiagnosticFrame frame;
            double elapsedMilliseconds = 0.0;
            double intervalMilliseconds = 0.0;
            double baselineMilliseconds = 0.0;
        };

        std::mutex mutex;
        std::condition_variable condition;
        std::deque<Row> rows;
        std::thread writer;
        bool stopWriter = false;
        std::atomic<std::size_t> droppedRows{0};
        QString writeError;
        QString path;
        QString lastAnomaly;
        QString lastWaitingPhase;
        bool running = false;
        std::size_t presentedFrames = 0;
        std::size_t anomalies = 0;
        std::uint64_t firstFrameId = 0;
        std::uint64_t lastGpuFrameId = 0;
        std::uint64_t run = 0;
        double intervalAverage = 0.0;
        double guiLateness = 0.0;
        Clock::time_point startedAt{};
        Clock::time_point lastPresentedAt{};
        Clock::time_point lastHeartbeatAt{};
        std::array<double, kStageCount> stageAverages{};

        Impl() { writer = std::thread([this]() { writeLoop(); }); }

        ~Impl()
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                stopWriter = true;
            }
            condition.notify_one();
            writer.join();
        }

        void enqueue(Row row)
        {
            {
                std::lock_guard<std::mutex> lock(mutex);
                // Diagnostic pressure may discard rows, never prediction input.
                if(row.action == Row::Action::Record && rows.size() >= kMaximumQueuedRows) {
                    ++droppedRows;
                    return;
                }
                rows.push_back(std::move(row));
            }
            condition.notify_one();
        }

        void writeLoop()
        {
            QFile file;
            auto lastFlush = Clock::now();
            for(;;) {
                Row row;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    condition.wait_for(lock, std::chrono::seconds(1), [this]() {
                        return stopWriter || !rows.empty();
                    });
                    if(rows.empty()) {
                        if(stopWriter) break;
                        lock.unlock();
                        if(file.isOpen()) file.flush();
                        continue;
                    }
                    row = std::move(rows.front());
                    rows.pop_front();
                }
                if(row.action == Row::Action::Begin) {
                    file.close();
                    QDir().mkpath(QFileInfo(row.note).absolutePath());
                    file.setFileName(row.note);
                    if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                        std::lock_guard<std::mutex> lock(mutex);
                        writeError = file.errorString();
                        continue;
                    }
                    QByteArray header("event,elapsed_ms,frame_id,displayed,vertices,spray_points,interval_ms,baseline_ms,expected_interval_ms,input_to_present_ms");
                    for(const char* column : kColumns) header += QByteArray(",") + column;
                    header += ",gpu_resident_display,thickness_readback_bytes,statistics_readback_bytes,scalar_upload_bytes,integrated_ms,compute_backlog_ms,physical_time_s,note\n";
                    file.write(header);
                    continue;
                }
                if(!file.isOpen()) continue;
                QByteArray line = row.kind.toUtf8();
                const auto number = [&line](double value) {
                    line += ',';
                    line += QByteArray::number(value, 'f', 4);
                };
                number(row.elapsedMilliseconds);
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.id));
                number(row.frame.displayed ? 1.0 : 0.0);
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.vertices));
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.sprayPoints));
                number(row.intervalMilliseconds);
                number(row.baselineMilliseconds);
                number(row.frame.expectedIntervalMilliseconds);
                number(row.frame.inputToPresentMilliseconds);
                for(double value : row.frame.stages) number(value);
                number(row.frame.gpuResidentDisplay ? 1.0 : 0.0);
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.thicknessReadbackBytes));
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.statisticsReadbackBytes));
                line += ',' + QByteArray::number(static_cast<qulonglong>(row.frame.scalarUploadBytes));
                number(row.frame.integratedMilliseconds);
                number(row.frame.computeBacklogMilliseconds);
                number(row.frame.physicalTimeSeconds);
                line += ',' + csvText(row.note).toUtf8() + '\n';
                if(file.write(line) < 0) {
                    std::lock_guard<std::mutex> lock(mutex);
                    writeError = file.errorString();
                }
                if(row.action == Row::Action::End || Clock::now() - lastFlush >= std::chrono::seconds(1)) {
                    file.flush();
                    lastFlush = Clock::now();
                }
                if(row.action == Row::Action::End) file.close();
            }
            if(file.isOpen()) file.flush();
        }
    };

    OnlinePredictionDiagnostics::OnlinePredictionDiagnostics() : m_impl(std::make_unique<Impl>()) {}
    OnlinePredictionDiagnostics::~OnlinePredictionDiagnostics() = default;

    void OnlinePredictionDiagnostics::begin(const QString& configuration)
    {
        finish(QStringLiteral("new run"));
        auto& state = *m_impl;
        state.running = true;
        state.presentedFrames = state.anomalies = 0;
        state.firstFrameId = state.lastGpuFrameId = 0;
        state.intervalAverage = state.guiLateness = 0.0;
        state.stageAverages = {};
        state.lastAnomaly.clear();
        state.lastWaitingPhase.clear();
        state.startedAt = state.lastHeartbeatAt = Clock::now();
        state.lastPresentedAt = {};
        state.droppedRows = 0;
        {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.writeError.clear();
        }
        // Keep diagnostics with the running Debug/Release build, independently
        // of the working directory chosen by Visual Studio or a shortcut.
        const QString directory = QCoreApplication::applicationDirPath();
        state.path = QDir(directory).filePath(QStringLiteral("diagnostics/online_prediction_%1_%2_%3.csv")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")))
            .arg(QCoreApplication::applicationPid()).arg(++state.run));
        Impl::Row row;
        row.action = Impl::Row::Action::Begin;
        row.note = state.path;
        state.enqueue(std::move(row));
        event(QStringLiteral("START"), 0.0, configuration + QStringLiteral(
            "; long_frame=max(20ms,1.5x_recent_interval); stage_spike=10ms_and_2.5x_recent_stage; warmup=5_frames. GPU time overlaps readback wait; stages overlap across threads and must not be summed as a frame interval. Input queue starts at the oldest merged command, input-to-present starts at the latest submitted input. Paint queue is inferred from request-to-render time minus measured paint work. Draw timing is CPU submission; swap timing is a Qt callback, not physical scanout."));
        event(QStringLiteral("GPU_DIAGNOSTICS"), 0.0, QStringLiteral(
            "GPU_DRAW rows are delayed nonblocking scalar-pass measurements keyed by frame_id; they are not presented frames. In GPU display mode, readback is statistics only; GPU completion waiting has a separate column."));
    }

    void OnlinePredictionDiagnostics::event(const QString& kind, double duration, const QString& detail)
    {
        if(!active()) return;
        Impl::Row row;
        row.kind = kind;
        row.elapsedMilliseconds = milliseconds(Clock::now() - m_impl->startedAt);
        row.intervalMilliseconds = duration;
        row.note = detail;
        m_impl->enqueue(std::move(row));
    }

    void OnlinePredictionDiagnostics::gpuDraw(std::uint64_t frameId, double duration)
    {
        if(!active() || !m_impl->firstFrameId || frameId < m_impl->firstFrameId
            || frameId <= m_impl->lastGpuFrameId) return;
        m_impl->lastGpuFrameId = frameId;
        Impl::Row row;
        row.kind = QStringLiteral("GPU_DRAW");
        row.elapsedMilliseconds = milliseconds(Clock::now() - m_impl->startedAt);
        row.frame.id = frameId;
        row.frame.at(OnlineDiagnosticStage::GpuScalarDraw) = duration;
        row.note = QStringLiteral("Asynchronous GPU scalar draw; excludes Qt composition and display scanout.");
        m_impl->enqueue(std::move(row));
    }

    void OnlinePredictionDiagnostics::heartbeat()
    {
        if(!active()) return;
        const auto now = Clock::now();
        const double late = std::max(0.0, milliseconds(now - m_impl->lastHeartbeatAt) - kHeartbeatMilliseconds);
        m_impl->lastHeartbeatAt = now;
        m_impl->guiLateness = std::max(m_impl->guiLateness, late);
        if(late >= 20.0) event(QStringLiteral("GUI_DELAY"), late,
            QStringLiteral("20 ms heartbeat arrived late; includes GUI work and OS scheduling, not proof of a specific function."));
    }

    void OnlinePredictionDiagnostics::record(OnlineDiagnosticFrame frame, Clock::time_point presentedAt)
    {
        if(!active()) return;
        auto& state = *m_impl;
        if(state.firstFrameId == 0) state.firstFrameId = frame.id;
        // Use the swap callback's entry timestamp, not time spent logging or
        // refreshing widgets inside the callback before reaching this method.
        const auto now = presentedAt;
        // Account for a blocked event loop even when its overdue timer has not
        // had a chance to run before this frame's swap callback.
        frame.at(OnlineDiagnosticStage::GuiTimerDelay) = std::max(state.guiLateness,
            std::max(0.0, milliseconds(now - state.lastHeartbeatAt) - kHeartbeatMilliseconds));
        Impl::Row row;
        row.frame = frame;
        row.elapsedMilliseconds = milliseconds(now - state.startedAt);
        row.baselineMilliseconds = state.intervalAverage;
        row.kind = frame.displayed ? QStringLiteral("FRAME") : QStringLiteral("NOT_PRESENTED");
        if(frame.displayed) {
            const bool first = state.presentedFrames == 0;
            row.intervalMilliseconds = first ? 0.0 : milliseconds(now - state.lastPresentedAt);
            const bool warmup = state.presentedFrames < 5;
            const bool longFrame = !first && row.intervalMilliseconds >= std::max(20.0, state.intervalAverage * 1.5);
            double largestExcess = 0.0;
            double largestWorkExcess = 0.0;
            std::size_t suspect = kStageCount;
            std::size_t workSuspect = kStageCount;
            bool stageSpike = false;
            for(std::size_t stage = 0; stage < kStageCount; ++stage) {
                // GPU compute overlaps readback; pacing and previous-frame waits
                // are pipeline policy. Keep them in CSV without blaming them twice.
                if(stage == static_cast<std::size_t>(OnlineDiagnosticStage::GpuCompute)
                    || stage == static_cast<std::size_t>(OnlineDiagnosticStage::Pacing)
                    || stage == static_cast<std::size_t>(OnlineDiagnosticStage::PreviousPresentation)) continue;
                const double value = frame.stages[stage];
                const double previous = state.stageAverages[stage];
                if(value >= 10.0 && previous > 0.5 && value > previous * 2.5) stageSpike = true;
                const double excess = warmup ? value : value - previous;
                if(excess > largestExcess) { largestExcess = excess; suspect = stage; }
                // Prefer a measured work stage over its downstream symptoms
                // (late input, a queued frame or a late GUI heartbeat).
                if(stage != static_cast<std::size_t>(OnlineDiagnosticStage::InputGap)
                    && stage != static_cast<std::size_t>(OnlineDiagnosticStage::InputQueue)
                    && stage != static_cast<std::size_t>(OnlineDiagnosticStage::DeliveryQueue)
                    && stage != static_cast<std::size_t>(OnlineDiagnosticStage::PaintQueue)
                    && stage != static_cast<std::size_t>(OnlineDiagnosticStage::GuiTimerDelay)
                    && excess > largestWorkExcess) {
                    largestWorkExcess = excess;
                    workSuspect = stage;
                }
            }
            if(largestWorkExcess >= 3.0) { suspect = workSuspect; largestExcess = largestWorkExcess; }
            if((first && row.elapsedMilliseconds >= 50.0) || longFrame || (!warmup && stageSpike)) {
                ++state.anomalies;
                row.kind = warmup ? QStringLiteral("STARTUP_SPIKE") : QStringLiteral("SPIKE");
                const QString cause = suspect < kStageCount && largestExcess >= 3.0
                    ? QString::fromLatin1(kNames[suspect]) : QStringLiteral("Unmeasured scheduling / input wait");
                row.note = QStringLiteral("Suspected stage:%1; observed=%2 ms; recent=%3 ms; phase=%4")
                    .arg(cause).arg(suspect < kStageCount ? frame.stages[suspect] : 0.0, 0, 'f', 2)
                    .arg(suspect < kStageCount ? state.stageAverages[suspect] : 0.0, 0, 'f', 2)
                    .arg(warmup ? QStringLiteral("warm-up") : QStringLiteral("steady"));
                state.lastAnomaly = QStringLiteral("%1%2 ms\nSuspected stage:%3 (%4 ms; recent %5 ms)")
                    .arg(warmup ? QStringLiteral("Startup / warm-up:") : QStringLiteral("Last long frame:"))
                    .arg(first ? row.elapsedMilliseconds : row.intervalMilliseconds, 0, 'f', 2)
                    .arg(cause).arg(suspect < kStageCount ? frame.stages[suspect] : 0.0, 0, 'f', 2)
                    .arg(suspect < kStageCount ? state.stageAverages[suspect] : 0.0, 0, 'f', 2);
            }
            if(!first) state.intervalAverage = state.intervalAverage == 0.0 ? row.intervalMilliseconds
                : 0.9 * state.intervalAverage + 0.1 * row.intervalMilliseconds;
            for(std::size_t stage = 0; stage < kStageCount; ++stage) {
                state.stageAverages[stage] = first ? frame.stages[stage]
                    : 0.9 * state.stageAverages[stage] + 0.1 * frame.stages[stage];
            }
            ++state.presentedFrames;
            state.lastPresentedAt = now;
            state.guiLateness = 0.0;
        }
        state.enqueue(std::move(row));
    }

    void OnlinePredictionDiagnostics::checkWaitingPhase(const QString& phase, double duration)
    {
        if(!active() || phase.isEmpty()) return;
        if(duration < std::max(100.0, m_impl->intervalAverage * 3.0)) {
            m_impl->lastWaitingPhase.clear();
            return;
        }
        if(m_impl->lastWaitingPhase != phase) ++m_impl->anomalies;
        m_impl->lastWaitingPhase = phase;
        m_impl->lastAnomaly = QStringLiteral("Current wait:%1 (%2 ms)").arg(phase).arg(duration, 0, 'f', 2);
        event(QStringLiteral("PIPELINE_WAIT"), duration, phase);
    }

    void OnlinePredictionDiagnostics::finish(const QString& reason)
    {
        if(!active()) return;
        Impl::Row row;
        row.action = Impl::Row::Action::End;
        row.kind = QStringLiteral("END");
        row.elapsedMilliseconds = milliseconds(Clock::now() - m_impl->startedAt);
        row.note = QStringLiteral("%1; frames=%2; anomalies=%3; dropped_diagnostic_rows=%4")
            .arg(reason).arg(static_cast<qulonglong>(m_impl->presentedFrames))
            .arg(static_cast<qulonglong>(m_impl->anomalies))
            .arg(static_cast<qulonglong>(m_impl->droppedRows.load()));
        m_impl->enqueue(std::move(row));
        m_impl->running = false;
    }

    bool OnlinePredictionDiagnostics::active() const { return m_impl->running; }
    QString OnlinePredictionDiagnostics::filePath() const { return m_impl->path; }

    QString OnlinePredictionDiagnostics::summary() const
    {
        QString text = m_impl->running ? QStringLiteral("Timing detection:running") : QStringLiteral("Timing detection:stopped");
        text += QStringLiteral("\nTiming anomalies:%1").arg(static_cast<qulonglong>(m_impl->anomalies));
        text += m_impl->lastAnomaly.isEmpty() ? QStringLiteral("\nWaiting for timing samples...")
            : QLatin1Char('\n') + m_impl->lastAnomaly;
        std::lock_guard<std::mutex> lock(m_impl->mutex);
        if(!m_impl->writeError.isEmpty()) text += QStringLiteral("\nDiagnostic write failed:") + m_impl->writeError;
        if(m_impl->droppedRows > 0) text += QStringLiteral("\nDropped diagnostic rows:%1").arg(static_cast<qulonglong>(m_impl->droppedRows.load()));
        return text;
    }
}
