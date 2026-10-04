#pragma once

#include <QString>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace robot_qt_viewer
{
    enum class OnlineDiagnosticStage : std::size_t
    {
        InputGap, InputQueue, BackendCpu, Upload, Dispatch, GpuCompute,
        Readback, ResultConversion, GpuTimerWait, Statistics, Mapping,
        DeliveryQueue, PreviousPresentation, Pacing, GuiResult, GuiPose, GuiOverlay,
        GuiInfo, PaintQueue, SceneUpdate, DrawSubmission, SwapWait, GuiTimerDelay,
        GuiPreparation, ContextPreparation, BackendInitialization, Count
    };

    struct OnlineDiagnosticFrame
    {
        std::uint64_t id = 0;
        std::size_t vertices = 0;
        std::size_t sprayPoints = 0;
        double inputToPresentMilliseconds = 0.0;
        double expectedIntervalMilliseconds = 0.0;
        bool displayed = false;
        std::array<double, static_cast<std::size_t>(OnlineDiagnosticStage::Count)> stages{};

        double& at(OnlineDiagnosticStage stage) { return stages[static_cast<std::size_t>(stage)]; }
    };

    // GUI-owned timing aggregation. Only the writer thread formats and writes CSV rows.
    class OnlinePredictionDiagnostics
    {
    public:
        OnlinePredictionDiagnostics();
        ~OnlinePredictionDiagnostics();
        OnlinePredictionDiagnostics(const OnlinePredictionDiagnostics&) = delete;
        OnlinePredictionDiagnostics& operator=(const OnlinePredictionDiagnostics&) = delete;

        void begin(const QString& configuration);
        void event(const QString& kind, double milliseconds, const QString& detail = {});
        void heartbeat();
        void checkWaitingPhase(const QString& phase, double milliseconds);
        void record(OnlineDiagnosticFrame frame, std::chrono::steady_clock::time_point presentedAt);
        void finish(const QString& reason);
        bool active() const;
        QString summary() const;
        QString filePath() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
