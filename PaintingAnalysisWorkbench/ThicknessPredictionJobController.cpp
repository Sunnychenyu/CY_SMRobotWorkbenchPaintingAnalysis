#include "ThicknessPredictionJobController.h"

#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>
#include <SprayThicknessPredictionOpenGL/MeshSamplingEvaluator.h>

#include <GLRuntime/GLRuntime.h>

#include <QMetaObject>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QThread>
#include <QString>

#include <condition_variable>
#include <exception>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>

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

    ThicknessPredictionJobController::ThicknessPredictionJobController(QObject* parent)
        : QObject(parent)
        , m_surface(new QOffscreenSurface())
    {
        m_surface->setFormat(computeFormat());
        m_surface->create();
        m_thread = QThread::create([this]() {
            workerLoop();
        });
        m_thread->start();
    }

    ThicknessPredictionJobController::~ThicknessPredictionJobController()
    {
        cancel();
        m_stopRequested.store(true);
        m_taskCondition.notify_all();
        if(m_thread != nullptr) {
            m_thread->wait();
            delete m_thread;
            m_thread = nullptr;
        }
        delete m_surface;
    }

    bool ThicknessPredictionJobController::start(
        spraythickness::ThicknessPredictionTask task,
        QString* errorMessage)
    {
        const auto fail = [errorMessage](const QString& message) {
            if(errorMessage != nullptr) {
                *errorMessage = message;
            }
            return false;
        };
        if(isRunning()) {
            return fail(QStringLiteral("A thickness prediction task is already running."));
        }
        if(m_surface == nullptr || !m_surface->isValid()) {
            return fail(QStringLiteral("The offscreen OpenGL surface is not valid."));
        }
        if(m_thread == nullptr || !m_thread->isRunning()) {
            return fail(QStringLiteral("The GPU prediction worker thread is not running."));
        }
        m_cancelRequested.store(false);
        {
            std::lock_guard<std::mutex> lock(m_taskMutex);
            if(m_pendingTask.has_value()) {
                return fail(QStringLiteral("A GPU prediction task is already queued."));
            }
            m_pendingTask = std::move(task);
            m_running.store(true);
        }
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        m_taskCondition.notify_one();
        emit runningChanged(true);
        return true;
    }

    void ThicknessPredictionJobController::cancel()
    {
        m_cancelRequested.store(true);
    }

    bool ThicknessPredictionJobController::isRunning() const
    {
        return m_running.load();
    }

    void ThicknessPredictionJobController::workerLoop()
    {
        std::unique_ptr<QOpenGLContext> context;
        std::unique_ptr<spraythickness::opengl::OpenGLThicknessPredictionBackend>
            backend;
        std::string deviceInfo;

        while(!m_stopRequested.load()) {
            std::optional<spraythickness::ThicknessPredictionTask> task;
            {
                std::unique_lock<std::mutex> lock(m_taskMutex);
                m_taskCondition.wait(lock, [this]() {
                    return m_stopRequested.load() || m_pendingTask.has_value();
                });
                if(m_stopRequested.load()) {
                    break;
                }
                task = std::move(m_pendingTask);
                m_pendingTask.reset();
            }

            try {
                if(context == nullptr) {
                    context = std::make_unique<QOpenGLContext>();
                    context->setFormat(computeFormat());
                    if(!context->create() || !context->makeCurrent(m_surface)) {
                        context.reset();
                        throw std::runtime_error(
                            "Failed to create the OpenGL 4.3 compute context.");
                    }
                    if(!GLRuntime::instance().initialize()) {
                        context->doneCurrent();
                        context.reset();
                        throw std::runtime_error(
                            "Failed to initialize OpenGL compute functions.");
                    }
                    backend = std::make_unique<
                        spraythickness::opengl::OpenGLThicknessPredictionBackend>();
                    deviceInfo = std::string("OpenGL compute device: vendor=")
                        + GLRuntime::instance().vendor() + ", renderer="
                        + GLRuntime::instance().renderer() + ", version="
                        + GLRuntime::instance().version();
                } else if(QOpenGLContext::currentContext() != context.get()
                    && !context->makeCurrent(m_surface)) {
                    throw std::runtime_error(
                        "Failed to activate the persistent OpenGL compute context.");
                }

                postProgress(0.01, deviceInfo + "\nUsing persistent GPU compute session.");
                spraythickness::ThicknessPredictionExecution execution;
                execution.cancelRequested = &m_cancelRequested;
                const bool assessMesh = task->options.evaluateMeshSampling;
                execution.progress = [this, deviceInfo, assessMesh](
                    double progress,
                    const std::string& message) {
                    postProgress(progress * (assessMesh ? 0.85 : 1.0), deviceInfo + "\n" + message);
                };
                auto result = backend->predict(*task, execution);
                if(assessMesh) {
                    postProgress(0.85, "Preparing mesh sampling assessment...");
                    execution.progress = [this](double progress, const std::string& message) {
                        postProgress(0.85 + 0.15 * progress, message);
                    };
                    try {
                        result.meshSamplingAssessment = std::make_shared<const
                            spraythickness::MeshSamplingAssessment>(
                                spraythickness::opengl::MeshSamplingEvaluator::evaluate(*task, execution));
                    } catch(const std::exception& exception) {
                        if(m_cancelRequested.load()) throw;
                        // A diagnostic failure must not discard a completed thickness field.
                        auto assessment = std::make_shared<spraythickness::MeshSamplingAssessment>();
                        assessment->unavailableReason = std::string("Mesh sampling assessment failed: ") + exception.what();
                        result.meshSamplingAssessment = std::move(assessment);
                    }
                }
                postFinished(std::move(result));
            } catch(const std::exception& exception) {
                postFailure(QString::fromLocal8Bit(exception.what()));
            } catch(...) {
                postFailure(QStringLiteral("Unknown GPU thickness prediction error."));
            }

            m_running.store(false);
            postRunningChanged(false);
        }

        backend.reset();
        if(context != nullptr) {
            context->doneCurrent();
        }
    }

    void ThicknessPredictionJobController::postProgress(
        double progress,
        const std::string& message)
    {
        QMetaObject::invokeMethod(this, [this, progress, message]() {
            emit progressChanged(progress, QString::fromStdString(message));
        }, Qt::QueuedConnection);
    }

    void ThicknessPredictionJobController::postFinished(
        spraythickness::ThicknessPredictionResult result)
    {
        QMetaObject::invokeMethod(this, [this, result = std::move(result)]() {
            emit predictionFinished(result);
        }, Qt::QueuedConnection);
    }

    void ThicknessPredictionJobController::postFailure(QString message)
    {
        QMetaObject::invokeMethod(this, [this, message = std::move(message)]() {
            emit predictionFailed(message);
        }, Qt::QueuedConnection);
    }

    void ThicknessPredictionJobController::postRunningChanged(bool running)
    {
        QMetaObject::invokeMethod(this, [this, running]() {
            emit runningChanged(running);
        }, Qt::QueuedConnection);
    }
}
