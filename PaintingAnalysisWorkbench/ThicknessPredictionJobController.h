#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QObject>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>

class QOffscreenSurface;
class QThread;
class QString;

namespace robot_qt_viewer
{
    class ThicknessPredictionJobController : public QObject
    {
        Q_OBJECT

    public:
        explicit ThicknessPredictionJobController(QObject* parent = nullptr);
        ~ThicknessPredictionJobController() override;

        bool start(
            spraythickness::ThicknessPredictionTask task,
            QString* errorMessage = nullptr);
        void cancel();
        bool isRunning() const;

    signals:
        void runningChanged(bool running);
        void progressChanged(double progress, const QString& message);
        void predictionFinished(const spraythickness::ThicknessPredictionResult& result);
        void predictionFailed(const QString& message);
        void surfacePreviewReady(std::shared_ptr<const spraythickness::DynamicSurfaceSnapshot> snapshot);

    private:
        void workerLoop();
        void postProgress(double progress, const std::string& message);
        void postPreview(std::shared_ptr<const spraythickness::DynamicSurfaceSnapshot> snapshot);
        void postFinished(spraythickness::ThicknessPredictionResult result);
        void postFailure(QString message);
        void postRunningChanged(bool running);

        QOffscreenSurface* m_surface = nullptr;
        QThread* m_thread = nullptr;
        std::atomic_bool m_cancelRequested{ false };
        std::atomic_bool m_running{ false };
        std::atomic_bool m_stopRequested{ false };
        std::mutex m_previewMutex;
        std::shared_ptr<const spraythickness::DynamicSurfaceSnapshot> m_latestPreview;
        bool m_previewQueued = false;
        std::mutex m_taskMutex;
        std::condition_variable m_taskCondition;
        std::optional<spraythickness::ThicknessPredictionTask> m_pendingTask;
    };
}
