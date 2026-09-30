#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QObject>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

class QOffscreenSurface;
class QThread;

namespace robot_qt_viewer
{
    class OnlineThicknessPredictionJobController : public QObject
    {
        Q_OBJECT

    public:
        explicit OnlineThicknessPredictionJobController(QObject* parent = nullptr);
        ~OnlineThicknessPredictionJobController() override;

        std::uint64_t begin(spraythickness::ThicknessPredictionTask task);
        void append(spraytrajectory::SprayTrajectory trajectory);
        void reset();

    signals:
        void fieldReady(const spraythickness::ThicknessPredictionResult& result);
        void predictionFailed(const QString& message);

    private:
        struct Command
        {
            enum class Kind { Begin, Append, Reset } kind{ Kind::Reset };
            std::uint64_t generation{ 0 };
            std::optional<spraythickness::ThicknessPredictionTask> task;
            spraytrajectory::SprayTrajectory trajectory;
        };

        void workerLoop();

        QOffscreenSurface* m_surface = nullptr;
        QThread* m_thread = nullptr;
        std::mutex m_mutex;
        std::condition_variable m_condition;
        std::deque<Command> m_commands;
        std::atomic<std::uint64_t> m_generation{ 0 };
        bool m_stopRequested = false;
        bool m_active = false;
    };
}
