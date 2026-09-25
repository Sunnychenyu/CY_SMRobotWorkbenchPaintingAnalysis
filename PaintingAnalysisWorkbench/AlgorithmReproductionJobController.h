#pragma once

#include <SprayThicknessPrediction/AlgorithmReproduction.h>

#include <QObject>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>

class QThread;
class QString;

namespace robot_qt_viewer
{
    class AlgorithmReproductionJobController : public QObject
    {
        Q_OBJECT

    public:
        explicit AlgorithmReproductionJobController(QObject* parent = nullptr);
        ~AlgorithmReproductionJobController() override;

        bool start(
            spraythickness::AlgorithmReproductionTask task,
            QString* errorMessage = nullptr);
        void cancel();
        bool isRunning() const;

    signals:
        void runningChanged(bool running);
        void progressChanged(double progress, const QString& message);
        void reproductionFinished(
            const spraythickness::AlgorithmReproductionResult& result);
        void reproductionFailed(const QString& message);

    private:
        void workerLoop();
        void postProgress(double progress, const std::string& message);
        void postFinished(spraythickness::AlgorithmReproductionResult result);
        void postFailure(QString message);
        void postRunningChanged(bool running);

        QThread* m_thread = nullptr;
        std::atomic_bool m_cancelRequested{ false };
        std::atomic_bool m_running{ false };
        std::atomic_bool m_stopRequested{ false };
        std::mutex m_taskMutex;
        std::condition_variable m_taskCondition;
        std::optional<spraythickness::AlgorithmReproductionTask> m_pendingTask;
    };
}
