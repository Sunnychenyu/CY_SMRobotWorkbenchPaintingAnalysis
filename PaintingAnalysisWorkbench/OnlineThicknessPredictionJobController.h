#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include "ThicknessUniformityAnalysis.h"
#include "PaintingAnalysisMeshAdapter.h"

#include <QObject>
#include <QString>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <memory>
#include <optional>

class QOffscreenSurface;
class QThread;

namespace robot_qt_viewer
{
    struct OnlinePredictionObjectBinding
    {
        std::string objectId;
        PaintingAnalysisMeshBinding mesh;
    };

    struct OnlinePredictionFrame
    {
        std::uint64_t id = 0;
        double timeSeconds = 0.0;
        std::chrono::steady_clock::time_point computedAt{};
        std::chrono::steady_clock::time_point submittedAt{};
        double inputIntervalMilliseconds = 0.0;
        double inputQueueMilliseconds = 0.0;
        double deliveryQueueMilliseconds = 0.0;
        double previousPresentationMilliseconds = 0.0;
        double contextPreparationMilliseconds = 0.0;
        double backendInitializationMilliseconds = 0.0;
        std::size_t processedSprayPointCount = 0;
        double statisticsMilliseconds = 0.0;
        bool statisticsUpdated = false;
        double mappingMilliseconds = 0.0;
        std::shared_ptr<const std::vector<smrobot::visualization::SurfaceScalarOverlay>> displayOverlays;
        Eigen::Isometry3d tablePose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d gunPose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d rotationAxis = Eigen::Vector3d::UnitZ();
    };

    struct OnlinePredictionWorkerState
    {
        QString phase;
        double milliseconds = 0.0;
    };

    class OnlineThicknessPredictionJobController : public QObject
    {
        Q_OBJECT

    public:
        explicit OnlineThicknessPredictionJobController(QObject* parent = nullptr);
        ~OnlineThicknessPredictionJobController() override;

        std::uint64_t begin(spraythickness::ThicknessPredictionTask task,
            std::vector<OnlinePredictionObjectBinding> objects = {});
        std::uint64_t append(spraytrajectory::SprayTrajectory trajectory,
            OnlinePredictionFrame frame);
        void acknowledgeFrame(std::uint64_t frameId);
        OnlinePredictionWorkerState diagnosticState();
        void reset();

    signals:
        void fieldReady(const std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>& result,
            const ThicknessUniformityStatistics& uniformity,
            const OnlinePredictionFrame& frame);
        void predictionFailed(const QString& message);

    private:
        struct Command
        {
            enum class Kind { Begin, Append, Reset } kind{ Kind::Reset };
            std::uint64_t generation{ 0 };
            std::optional<spraythickness::ThicknessPredictionTask> task;
            std::vector<OnlinePredictionObjectBinding> objects;
            spraytrajectory::SprayTrajectory trajectory;
            OnlinePredictionFrame frame;
            std::chrono::steady_clock::time_point queuedAt = std::chrono::steady_clock::now();
        };

        void workerLoop();
        void setDiagnosticPhase(const QString& phase);
        void deliverLatestField(std::uint64_t generation);

        QOffscreenSurface* m_surface = nullptr;
        QThread* m_thread = nullptr;
        std::mutex m_mutex;
        std::condition_variable m_condition;
        std::deque<Command> m_commands;
        std::atomic<std::uint64_t> m_generation{ 0 };
        bool m_stopRequested = false;
        bool m_active = false;
        std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> m_pendingField;
        ThicknessUniformityStatistics m_pendingUniformity;
        OnlinePredictionFrame m_pendingFrame;
        std::uint64_t m_pendingGeneration = 0;
        std::chrono::steady_clock::time_point m_pendingQueuedAt{};
        std::chrono::steady_clock::time_point m_deliveryPostedAt{};
        std::chrono::steady_clock::time_point m_lastDeliveryLog{};
        bool m_deliveryPosted = false;
        std::uint64_t m_nextFrameId = 0;
        std::uint64_t m_awaitingFrameId = 0;
        QString m_diagnosticPhase;
        std::chrono::steady_clock::time_point m_diagnosticPhaseStartedAt{};
    };
}
