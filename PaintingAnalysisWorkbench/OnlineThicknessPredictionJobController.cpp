#include "OnlineThicknessPredictionJobController.h"

#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>

#include <GLRuntime/GLRuntime.h>
#include <CustomLog/CustomLog.h>

#include <QMetaObject>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLExtraFunctions>
#include <QSurfaceFormat>
#include <QThread>
#include <QString>

#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>
#include <array>
#include <limits>
#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        struct DisplaySlot
        {
            std::shared_ptr<QOpenGLBuffer> buffer;
            std::shared_ptr<std::atomic<void*>> fence =
                std::make_shared<std::atomic<void*>>(nullptr);
        };
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
            m_intervalBudget = 4;
            m_completedFrames = 0;
            m_completedTimeSeconds = 0.0;
            m_latestInputTimeSeconds = 0.0;
            m_lastDeliveredStatisticsRevision = 0;
            m_surfaceDistance = {};
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

    void OnlineThicknessPredictionJobController::prepareModel(
        std::shared_ptr<const assetcore::ModelDesc> model, const Eigen::Isometry3d& transform)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if(m_active || !model) return;
        m_commands.clear();
        Command command;
        command.kind = Command::Kind::Prepare;
        command.generation = ++m_generation;
        command.model = std::move(model);
        command.transform = transform;
        m_commands.push_back(std::move(command));
        m_condition.notify_one();
    }

    std::function<double(const Eigen::Vector3d&)>
        OnlineThicknessPredictionJobController::surfaceDistanceQuery()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_surfaceDistance;
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
            // Each command ends at its own pose/time. Never turn a stalled GUI
            // into a single unbounded compute dispatch by merging all commands.
            if(m_commands.size() >= 2048) {
                QMetaObject::invokeMethod(this, [this, generation = m_generation.load()]() {
                    if(generation == m_generation.load())
                        emit predictionFailed(QStringLiteral("Online input backlog exceeded 2048 batches; accumulation stopped."));
                }, Qt::QueuedConnection);
                return 0;
            }
            Command command;
            command.kind = Command::Kind::Append;
            command.generation = m_generation.load();
            command.trajectory = std::move(trajectory);
            m_latestInputTimeSeconds = frame.timeSeconds;
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
            m_surfaceDistance = {};
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
                std::chrono::steady_clock::now() - m_diagnosticPhaseStartedAt).count(),
            m_completedFrames, m_completedTimeSeconds };
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
        spraycore::SprayTool onlineTool;
        bool sharedGpuDisplay = false;
        std::size_t displayValueCount = 0;
        std::vector<ViewportGpuScalarField> displayLayouts;
        std::array<DisplaySlot, 3> displaySlots;
        ThicknessUniformityStatistics uniformity;
        std::chrono::steady_clock::time_point lastStatisticsAt{};
        std::uint64_t statisticsRevision = 0;
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
                        && (!m_pendingField || m_awaitingFrameId != 0
                            || m_commands.front().kind != Command::Kind::Append));
                });
                if(m_stopRequested) {
                    break;
                }
                command = std::move(m_commands.front());
                m_commands.pop_front();
                if(command.kind == Command::Kind::Append && m_awaitingFrameId != 0) {
                    // A manual display cap must not throttle physical integration.
                    // Retire only the unpublished display snapshot, never its
                    // accumulated field or any input interval.
                    m_pendingField.reset();
                    m_pendingFrame = {};
                }
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
                    displayLayouts.clear();
                    displayValueCount = 0;
                    overlaySnapshots = {};
                    continue;
                }
                const auto contextStart = std::chrono::steady_clock::now();
                if(command.kind == Command::Kind::Prepare && !QOpenGLContext::globalShareContext()) continue;
                setDiagnosticPhase(QStringLiteral("Compute context startup"));
                if(!context) {
                    context = std::make_unique<QOpenGLContext>();
                    context->setFormat(computeFormat());
                    // AA_ShareOpenGLContexts covers widgets, not manually created
                    // compute contexts. Join their share group explicitly.
                    context->setShareContext(QOpenGLContext::globalShareContext());
                    if(!context->create() || !context->makeCurrent(m_surface)
                        || !GLRuntime::instance().initialize()) {
                        throw std::runtime_error(
                            "Failed to create an online OpenGL 4.3 compute context.");
                    }
                    backend = std::make_unique<spraythickness::opengl::
                        OpenGLThicknessPredictionBackend>();
                    sharedGpuDisplay = context->shareContext()
                        && QOpenGLContext::areSharing(context.get(), context->shareContext());
                } else if(QOpenGLContext::currentContext() != context.get()
                    && !context->makeCurrent(m_surface)) {
                    throw std::runtime_error(
                        "Failed to activate the online OpenGL compute context.");
                }
                if(command.kind == Command::Kind::Prepare) {
                    setDiagnosticPhase(QStringLiteral("Background model / BVH / GPU preparation"));
                    spraythickness::ThicknessPredictionTask task;
                    task.model = spraythickness::ThicknessModelKind::PaperGaussian;
                    task.workpiece = PaintingAnalysisMeshAdapter::build(*command.model,
                        "Online prepared workpiece", "", command.transform).workpiece;
                    backend->beginOnline(std::move(task));
                    continue;
                }
                if(command.kind == Command::Kind::Begin) {
                    contextPreparationMilliseconds = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - contextStart).count();
                    const auto initializationStart = std::chrono::steady_clock::now();
                    setDiagnosticPhase(QStringLiteral("BVH / initial GPU preparation"));
                    onlineTool = command.task->tool;
                    backend->beginOnline(std::move(*command.task));
                    previousSprayPointCount = 0;
                    snapshots = {};
                    overlaySnapshots = {};
                    objects = std::move(command.objects);
                    displayLayouts.clear();
                    displayValueCount = 0;
                    if(sharedGpuDisplay && !objects.empty()) {
                        std::vector<std::uint32_t> indices;
                        for(const auto& object : objects) {
                            ViewportGpuScalarField layout;
                            for(const auto& submesh : object.mesh.sampleIndicesBySubMesh) {
                                layout.subMeshOffsets.push_back(indices.size());
                                layout.subMeshCounts.push_back(submesh.size());
                                for(const auto index : submesh) {
                                    if(index > std::numeric_limits<std::uint32_t>::max())
                                        throw std::runtime_error("Online display sample index is too large.");
                                    indices.push_back(static_cast<std::uint32_t>(index));
                                }
                            }
                            displayLayouts.push_back(std::move(layout));
                        }
                        GLint maximumTextureValues = 0;
                        context->extraFunctions()->glGetIntegerv(GL_MAX_TEXTURE_BUFFER_SIZE, &maximumTextureValues);
                        if(indices.size() > static_cast<std::size_t>(maximumTextureValues)
                            || indices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / sizeof(float)) {
                            throw std::runtime_error("Online display exceeds the GPU texture-buffer capacity.");
                        }
                        backend->setOnlineDisplayMapping(indices);
                        displayValueCount = indices.size();
                    }
                    LOG_DEBUG("rs2026") << "Online display transport: "
                        << (displayValueCount ? "GPU shared immutable field" : "CPU compatibility field");
                    uniformity = {};
                    lastStatisticsAt = {};
                    statisticsRevision = 0;
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        if(command.generation != m_generation.load()) continue;
                        m_surfaceDistance = backend->onlineSurfaceDistanceQuery();
                    }
                    ViewportSprayInfluenceGeometry geometry;
                    if(sharedGpuDisplay) {
                        auto lease = std::make_shared<std::array<QOpenGLBuffer, 4>>();
                        for(std::size_t i = 0; i < lease->size(); ++i) {
                            if(!(*lease)[i].create())
                                throw std::runtime_error("Failed to allocate visibility preview geometry.");
                            geometry.buffers[i] = (*lease)[i].bufferId();
                        }
                        backend->copyOnlineVisibilityGeometry(geometry.buffers);
                        auto* gl = context->extraFunctions();
                        const auto fence = gl->glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                        if(!fence) throw std::runtime_error("Failed to fence visibility preview geometry.");
                        gl->glFlush();
                        GLenum ready = GL_TIMEOUT_EXPIRED;
                        bool stopped = false;
                        while(ready == GL_TIMEOUT_EXPIRED && command.generation == m_generation.load()) {
                            {
                                std::lock_guard<std::mutex> lock(m_mutex);
                                stopped = m_stopRequested;
                            }
                            if(stopped) break;
                            ready = gl->glClientWaitSync(fence, 0, 1000000);
                        }
                        gl->glDeleteSync(fence);
                        if(ready == GL_WAIT_FAILED)
                            throw std::runtime_error("Visibility preview geometry synchronization failed.");
                        if(stopped || command.generation != m_generation.load()) continue;
                        geometry.lifetime = std::move(lease);
                    }
                    backendInitializationMilliseconds = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - initializationStart).count();
                    QMetaObject::invokeMethod(this, [this, geometry, generation = command.generation]() {
                        if(generation == m_generation.load()) emit prepared(geometry);
                    }, Qt::QueuedConnection);
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
                std::shared_ptr<QOpenGLBuffer> displayBuffer;
                std::shared_ptr<std::atomic<void*>> displayFence;
                const bool fullStatistics = lastStatisticsAt == std::chrono::steady_clock::time_point{}
                    || command.frame.finalInterval
                    || contextStart - lastStatisticsAt >= std::chrono::milliseconds(200);
                if(command.frame.tool) {
                    backend->setOnlineToolDirections(command.frame.tool->sprayDirectionLocal,
                        command.frame.tool->powderFeedDirectionLocal);
                    onlineTool.sprayDirectionLocal = command.frame.tool->sprayDirectionLocal.normalized();
                    onlineTool.powderFeedDirectionLocal = command.frame.tool->powderFeedDirectionLocal.normalized();
                }
                command.frame.tool = onlineTool;
                if(displayValueCount) {
                    setDiagnosticPhase(QStringLiteral("GPU accumulation / display snapshot / reduction"));
                    // Three stores bound memory usage. CPU ownership AND the
                    // renderer's completion fence must both permit reuse.
                    while(!displayBuffer) {
                        if(m_generation.load() != command.generation) break;
                        for(auto& slot : displaySlots) {
                            if(!slot.buffer) slot.buffer = std::make_shared<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
                            if(slot.buffer.use_count() != 1) continue;
                            auto* functions = context->extraFunctions();
                            const auto fence = static_cast<GLsync>(slot.fence->load());
                            if(fence) {
                                const auto status = functions->glClientWaitSync(fence, 0, 0);
                                if(status == GL_TIMEOUT_EXPIRED) continue;
                                if(status == GL_WAIT_FAILED) throw std::runtime_error("Display buffer fence wait failed.");
                                functions->glDeleteSync(fence);
                                slot.fence->store(nullptr);
                            }
                            displayBuffer = slot.buffer;
                            displayFence = slot.fence;
                            break;
                        }
                        if(!displayBuffer) {
                            std::unique_lock<std::mutex> lock(m_mutex);
                            if(m_stopRequested) break;
                            m_condition.wait_for(lock, std::chrono::milliseconds(1));
                        }
                    }
                    if(!displayBuffer) continue;
                    if(!displayBuffer->isCreated() && !displayBuffer->create())
                        throw std::runtime_error("Failed to create shared display buffer.");
                    if(!displayBuffer->bind()) throw std::runtime_error("Failed to bind shared display buffer.");
                    const auto byteCount = static_cast<int>(displayValueCount * sizeof(float));
                    if(displayBuffer->size() != byteCount) {
                        displayBuffer->setUsagePattern(QOpenGLBuffer::StreamCopy);
                        displayBuffer->allocate(byteCount);
                    }
                    displayBuffer->release();
                    backend->appendOnlineGpu(command.trajectory, displayBuffer->bufferId(), *result,
                        [this, generation = command.generation]() {
                            std::lock_guard<std::mutex> lock(m_mutex);
                            return m_stopRequested || generation != m_generation.load();
                        }, fullStatistics);
                } else {
                    setDiagnosticPhase(QStringLiteral("GPU accumulation / CPU compatibility readback"));
                    backend->appendOnline(command.trajectory, *result);
                }
                command.frame.processedSprayPointCount = result->timing.sprayPointCount - previousSprayPointCount;
                previousSprayPointCount = result->timing.sprayPointCount;
                if(command.frame.processedSprayPointCount && result->timing.pureGpuMilliseconds > 0.0) {
                    const double perPoint = result->timing.pureGpuMilliseconds / command.frame.processedSprayPointCount;
                    const auto previous = m_intervalBudget.load();
                    // Target a short GPU slice; grow slowly, shrink immediately.
                    m_intervalBudget = std::clamp<std::size_t>(static_cast<std::size_t>(
                        std::max(1.0, 2.0 / perPoint)), 1, std::min<std::size_t>(64, previous * 2));
                }
                const auto statisticsStart = std::chrono::steady_clock::now();
                setDiagnosticPhase(QStringLiteral("Uniformity statistics"));
                command.frame.statisticsUpdated = fullStatistics;
                if(command.frame.statisticsUpdated) {
                    uniformity = calculateThicknessUniformityStatistics(*result,
                        result->metrics.minThickness, result->metrics.maxThickness);
                    if(command.frame.statisticsUpdated) lastStatisticsAt = statisticsStart;
                    ++statisticsRevision;
                }
                command.frame.statisticsRevision = statisticsRevision;
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
                    std::shared_ptr<std::vector<ViewportGpuScalarField>> fields;
                    if(displayBuffer) fields = std::make_shared<std::vector<ViewportGpuScalarField>>(displayLayouts);
                    for(std::size_t object = 0; object < objects.size(); ++object) {
                        if(displayBuffer) {
                            auto& overlay = (*overlays)[object];
                            overlay.objectId = objects[object].objectId;
                            overlay.quantityName = "Thickness";
                            overlay.unit = "m";
                            overlay.range.minimum = result->metrics.minThickness;
                            overlay.range.maximum = result->metrics.maxThickness;
                            overlay.subMeshes.resize(displayLayouts[object].subMeshCounts.size());
                            auto& field = (*fields)[object];
                            field.buffer = displayBuffer->bufferId();
                            field.valueCount = displayValueCount;
                            field.lifetime = displayBuffer;
                            field.lastUseFence = displayFence;
                        } else {
                            PaintingAnalysisMeshAdapter::updateOverlay(objects[object].objectId,
                                objects[object].mesh, *result, (*overlays)[object]);
                        }
                    }
                    command.frame.gpuDisplayFields = std::move(fields);
                    command.frame.displayOverlays = std::move(overlays);
                }
                command.frame.computedAt = std::chrono::steady_clock::now();
                command.frame.mappingMilliseconds = std::chrono::duration<double, std::milli>(
                    command.frame.computedAt - mappingStart).count();
                const auto completedFrameId = command.frame.id;
                bool postDelivery = false;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if(command.generation != m_generation.load()) continue;
                    ++m_completedFrames;
                    m_completedTimeSeconds = command.frame.timeSeconds;
                    command.frame.computeBacklogMilliseconds = std::max(0.0,
                        m_latestInputTimeSeconds - m_completedTimeSeconds) * 1000.0;
                    // Retain only the newest unpublished display snapshot. Every
                    // input command has already been integrated in temporal order.
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
                QMetaObject::invokeMethod(this, [this, generation = command.generation, completedFrameId]() {
                    if(generation == m_generation.load()) emit inputCapacityAvailable(completedFrameId);
                }, Qt::QueuedConnection);
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
        if(context) {
            for(auto& slot : displaySlots) {
                const auto previous = slot.fence->exchange(reinterpret_cast<void*>(std::uintptr_t{1}));
                if(previous) context->extraFunctions()->glDeleteSync(static_cast<GLsync>(previous));
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
                frame.statisticsUpdated = frame.statisticsRevision != m_lastDeliveredStatisticsRevision;
                m_lastDeliveredStatisticsRevision = frame.statisticsRevision;
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
