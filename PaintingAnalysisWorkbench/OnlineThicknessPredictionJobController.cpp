#include "OnlineThicknessPredictionJobController.h"

#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>

#include <GLRuntime/GLRuntime.h>
#include <CustomLog/CustomLog.h>

#include <QMetaObject>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QThread>
#include <QString>

#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>
#include <array>

namespace robot_qt_viewer
{
    namespace
    {
        QSurfaceFormat computeFormat()
        {
            QSurfaceFormat format;
            format.setVersion(4, 3);
            format.setProfile(QSurfaceFormat::CoreProfile);
            return format;
        }
    }

    OnlineThicknessPredictionJobController::OnlineThicknessPredictionJobController(
        QObject* parent)
        : QObject(parent)
        , m_surface(new QOffscreenSurface())
    {
        m_surface->setFormat(computeFormat());
        m_surface->create();
        m_thread = QThread::create([this]() { workerLoop(); });
        m_thread->start();
    }

    OnlineThicknessPredictionJobController::~OnlineThicknessPredictionJobController()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stopRequested = true;
            m_commands.clear();
            m_pendingField.reset();
            m_pendingFrame = {};
        }
        m_condition.notify_all();
        m_thread->wait();
        delete m_thread;
        delete m_surface;
    }

    std::uint64_t OnlineThicknessPredictionJobController::begin(
        spraythickness::ThicknessPredictionTask task,
        std::vector<OnlinePredictionObjectBinding> objects)
    {
        const std::uint64_t generation = ++m_generation;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_commands.clear();
            m_pendingField.reset();
            m_pendingFrame = {};
            m_awaitingFrameId = 0;
            m_deliveryPosted = false;
            Command command;
            command.kind = Command::Kind::Begin;
            command.generation = generation;
            command.task = std::move(task);
            command.objects = std::move(objects);
            m_commands.push_back(std::move(command));
            m_active = true;
        }
        m_condition.notify_one();
        return generation;
    }

    std::uint64_t OnlineThicknessPredictionJobController::append(
        spraytrajectory::SprayTrajectory trajectory, OnlinePredictionFrame frame)
    {
        std::uint64_t frameId = 0;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if(!m_active) {
                return 0;
            }
            frame.id = frameId = ++m_nextFrameId;
            if(!m_commands.empty()
                && m_commands.back().kind == Command::Kind::Append
                && m_commands.back().generation == m_generation.load()) {
                auto& segments = m_commands.back().trajectory.segments;
                for(auto& segment : trajectory.segments) {
                    segments.push_back(std::move(segment));
                }
                m_commands.back().frame = std::move(frame);
                return frameId;
            }
            Command command;
            command.kind = Command::Kind::Append;
            command.generation = m_generation.load();
            command.trajectory = std::move(trajectory);
            command.frame = std::move(frame);
            m_commands.push_back(std::move(command));
        }
        m_condition.notify_one();
        return frameId;
    }

    void OnlineThicknessPredictionJobController::acknowledgeFrame(std::uint64_t frameId)
    {
        bool postDelivery = false;
        std::uint64_t generation = 0;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if(frameId == 0 || frameId != m_awaitingFrameId) return;
            m_awaitingFrameId = 0;
            postDelivery = m_pendingField && !m_deliveryPosted;
            if(postDelivery) {
                m_deliveryPosted = true;
                m_deliveryPostedAt = std::chrono::steady_clock::now();
                generation = m_pendingGeneration;
            }
        }
        m_condition.notify_one();
        if(postDelivery) {
            QMetaObject::invokeMethod(this,
                [this, generation]() { deliverLatestField(generation); }, Qt::QueuedConnection);
        }
    }

    void OnlineThicknessPredictionJobController::reset()
    {
        const std::uint64_t generation = ++m_generation;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_commands.clear();
            m_pendingField.reset();
            m_pendingFrame = {};
            m_active = false;
            m_awaitingFrameId = 0;
            m_deliveryPosted = false;
            Command command;
            command.kind = Command::Kind::Reset;
            command.generation = generation;
            m_commands.push_back(std::move(command));
        }
        m_condition.notify_one();
    }

    void OnlineThicknessPredictionJobController::setDiagnosticPhase(const QString& phase)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_diagnosticPhase = phase;
        m_diagnosticPhaseStartedAt = std::chrono::steady_clock::now();
    }

    OnlinePredictionWorkerState OnlineThicknessPredictionJobController::diagnosticState()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return { m_diagnosticPhase, m_diagnosticPhaseStartedAt == std::chrono::steady_clock::time_point{}
            ? 0.0 : std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - m_diagnosticPhaseStartedAt).count() };
    }

    void OnlineThicknessPredictionJobController::workerLoop()
    {
        std::unique_ptr<QOpenGLContext> context;
        std::unique_ptr<spraythickness::opengl::OpenGLThicknessPredictionBackend>
            backend;
        std::array<std::shared_ptr<spraythickness::OnlineThicknessSnapshot>, 3> snapshots;
        using Overlays = std::vector<smrobot::visualization::SurfaceScalarOverlay>;
        std::array<std::shared_ptr<Overlays>, 3> overlaySnapshots;
        std::vector<OnlinePredictionObjectBinding> objects;
        ThicknessUniformityStatistics uniformity;
        std::chrono::steady_clock::time_point lastStatisticsAt{};
        double contextPreparationMilliseconds = 0.0;
        double backendInitializationMilliseconds = 0.0;
        std::size_t previousSprayPointCount = 0;
        for(;;) {
            Command command;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                if(m_commands.empty() || m_pendingField) {
                    m_diagnosticPhase = m_pendingField ? QStringLiteral("Previous frame presentation")
                        : QStringLiteral("Input sampling / submission");
                    m_diagnosticPhaseStartedAt = std::chrono::steady_clock::now();
                }
                m_condition.wait(lock, [this]() {
                    return m_stopRequested || (!m_commands.empty()
                        && (!m_pendingField
                            || m_commands.front().kind != Command::Kind::Append));
                });
                if(m_stopRequested) {
                    break;
                }
                command = std::move(m_commands.front());
                m_commands.pop_front();
            }
            if(command.generation != m_generation.load()) {
                continue;
            }
            try {
                if(command.kind == Command::Kind::Reset) {
                    if(backend) {
                        backend->endOnline();
                    }
                    objects.clear();
                    overlaySnapshots = {};
                    continue;
                }
                const auto contextStart = std::chrono::steady_clock::now();
                setDiagnosticPhase(QStringLiteral("Compute context startup"));
                if(!context) {
                    context = std::make_unique<QOpenGLContext>();
                    context->setFormat(computeFormat());
                    if(!context->create() || !context->makeCurrent(m_surface)
                        || !GLRuntime::instance().initialize()) {
                        throw std::runtime_error(
                            "Failed to create an online OpenGL 4.3 compute context.");
                    }
                    backend = std::make_unique<spraythickness::opengl::
                        OpenGLThicknessPredictionBackend>();
                } else if(QOpenGLContext::currentContext() != context.get()
                    && !context->makeCurrent(m_surface)) {
                    throw std::runtime_error(
                        "Failed to activate the online OpenGL compute context.");
                }
                if(command.kind == Command::Kind::Begin) {
                    contextPreparationMilliseconds = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - contextStart).count();
                    const auto initializationStart = std::chrono::steady_clock::now();
                    setDiagnosticPhase(QStringLiteral("BVH / initial GPU preparation"));
                    backend->beginOnline(std::move(*command.task));
                    backendInitializationMilliseconds = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - initializationStart).count();
                    previousSprayPointCount = 0;
                    snapshots = {};
                    overlaySnapshots = {};
                    objects = std::move(command.objects);
                    uniformity = {};
                    lastStatisticsAt = {};
                    continue;
                }
                command.frame.inputQueueMilliseconds = std::chrono::duration<double, std::milli>(
                    contextStart - command.queuedAt).count();
                command.frame.contextPreparationMilliseconds = contextPreparationMilliseconds;
                command.frame.backendInitializationMilliseconds = backendInitializationMilliseconds;
                contextPreparationMilliseconds = backendInitializationMilliseconds = 0.0;
                std::shared_ptr<spraythickness::OnlineThicknessSnapshot> result;
                for(auto& snapshot : snapshots) {
                    if(!snapshot) snapshot = std::make_shared<spraythickness::OnlineThicknessSnapshot>();
                    if(snapshot.use_count() == 1) {
                        result = snapshot;
                        break;
                    }
                }
                // External consumers can retain old snapshots. Never mutate them.
                if(!result) result = std::make_shared<spraythickness::OnlineThicknessSnapshot>();
                setDiagnosticPhase(QStringLiteral("GPU backend accumulation / readback"));
                backend->appendOnline(command.trajectory, *result);
                command.frame.processedSprayPointCount = result->timing.sprayPointCount - previousSprayPointCount;
                previousSprayPointCount = result->timing.sprayPointCount;
                const auto statisticsStart = std::chrono::steady_clock::now();
                setDiagnosticPhase(QStringLiteral("Uniformity statistics"));
                command.frame.statisticsUpdated = !uniformity.valid
                    || statisticsStart - lastStatisticsAt >= std::chrono::milliseconds(200);
                if(command.frame.statisticsUpdated) {
                    uniformity = calculateThicknessUniformityStatistics(*result,
                        result->metrics.minThickness, result->metrics.maxThickness);
                    lastStatisticsAt = statisticsStart;
                }
                const auto mappingStart = std::chrono::steady_clock::now();
                setDiagnosticPhase(QStringLiteral("Cloud mapping"));
                command.frame.statisticsMilliseconds = std::chrono::duration<double, std::milli>(
                    mappingStart - statisticsStart).count();
                if(!objects.empty()) {
                    std::shared_ptr<Overlays> overlays;
                    for(auto& snapshot : overlaySnapshots) {
                        if(!snapshot) snapshot = std::make_shared<Overlays>();
                        if(snapshot.use_count() == 1) {
                            overlays = snapshot;
                            break;
                        }
                    }
                    // A retained display frame is immutable, just like its field.
                    if(!overlays) overlays = std::make_shared<Overlays>();
                    overlays->resize(objects.size());
                    for(std::size_t object = 0; object < objects.size(); ++object) {
                        PaintingAnalysisMeshAdapter::updateOverlay(objects[object].objectId,
                            objects[object].mesh, *result, (*overlays)[object]);
                    }
                    command.frame.displayOverlays = std::move(overlays);
                }
                command.frame.computedAt = std::chrono::steady_clock::now();
                command.frame.mappingMilliseconds = std::chrono::duration<double, std::milli>(
                    command.frame.computedAt - mappingStart).count();
                bool postDelivery = false;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if(command.generation != m_generation.load()) continue;
                    // One displayed frame plus one computed frame. Input intervals
                    // merge while the bounded pipeline is full; none are discarded.
                    m_pendingField = std::move(result);
                    m_pendingUniformity = uniformity;
                    m_pendingFrame = std::move(command.frame);
                    m_pendingGeneration = command.generation;
                    m_pendingQueuedAt = command.queuedAt;
                    postDelivery = m_awaitingFrameId == 0 && !m_deliveryPosted;
                    if(postDelivery) {
                        m_deliveryPosted = true;
                        m_deliveryPostedAt = std::chrono::steady_clock::now();
                    }
                }
                if(postDelivery) {
                    QMetaObject::invokeMethod(this,
                        [this, generation = command.generation]() { deliverLatestField(generation); }, Qt::QueuedConnection);
                }
            } catch(const std::exception& exception) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if(command.generation == m_generation.load()) {
                        m_active = false;
                        m_commands.clear();
                        m_pendingField.reset();
                        m_pendingFrame = {};
                        m_awaitingFrameId = 0;
                    }
                }
                if(backend) {
                    backend->endOnline();
                }
                const QString message = QString::fromLocal8Bit(exception.what());
                QMetaObject::invokeMethod(this,
                    [this, generation = command.generation, message]() {
                        if(generation == m_generation.load()) {
                            emit predictionFailed(message);
                        }
                    }, Qt::QueuedConnection);
            }
        }
        backend.reset();
        if(context) {
            context->doneCurrent();
        }
    }

    void OnlineThicknessPredictionJobController::deliverLatestField(std::uint64_t expectedGeneration)
    {
        std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> result;
        ThicknessUniformityStatistics uniformity;
        OnlinePredictionFrame frame;
        std::uint64_t generation = 0;
        std::chrono::steady_clock::time_point queuedAt;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if(expectedGeneration != m_generation.load()) return;
            result = std::move(m_pendingField);
            uniformity = m_pendingUniformity;
            frame = std::move(m_pendingFrame);
            if(result) {
                const auto now = std::chrono::steady_clock::now();
                frame.deliveryQueueMilliseconds = std::chrono::duration<double, std::milli>(
                    now - m_deliveryPostedAt).count();
                frame.previousPresentationMilliseconds = std::chrono::duration<double, std::milli>(
                    m_deliveryPostedAt - frame.computedAt).count();
            }
            m_pendingFrame = {};
            generation = m_pendingGeneration;
            queuedAt = m_pendingQueuedAt;
            m_deliveryPosted = false;
            if(result) m_awaitingFrameId = frame.id;
        }
        m_condition.notify_one();
        if(!result || generation != m_generation.load()) return;
        const auto now = std::chrono::steady_clock::now();
        if(now - m_lastDeliveryLog >= std::chrono::seconds(1)) {
            m_lastDeliveryLog = now;
            LOG_DEBUG("rs2026") << "Online prediction delivery: inputToGuiMs="
                << std::chrono::duration<double, std::milli>(now - queuedAt).count()
                << ", accumulatedSprayPoints=" << result->timing.sprayPointCount
                << ", backendMs=" << result->timing.backendTotalMilliseconds
                << ", uploadMs=" << result->timing.uploadMilliseconds
                << ", gpuMs=" << result->timing.pureGpuMilliseconds
                << ", readbackMs=" << result->timing.readbackMilliseconds;
            LOG_DEBUG("rs2026") << "Online pipeline: computedToGuiMs="
                << std::chrono::duration<double, std::milli>(now - frame.computedAt).count()
                << ", statisticsMs=" << frame.statisticsMilliseconds
                << ", statisticsUpdated=" << frame.statisticsUpdated
                << ", workerMappingMs=" << frame.mappingMilliseconds;
        }
        emit fieldReady(result, uniformity, frame);
    }
}
