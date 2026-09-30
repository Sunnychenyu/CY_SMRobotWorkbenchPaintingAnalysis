#include "OnlineThicknessPredictionJobController.h"

#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>

#include <GLRuntime/GLRuntime.h>

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
        }
        m_condition.notify_all();
        m_thread->wait();
        delete m_thread;
        delete m_surface;
    }

    std::uint64_t OnlineThicknessPredictionJobController::begin(
        spraythickness::ThicknessPredictionTask task)
    {
        const std::uint64_t generation = ++m_generation;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_commands.clear();
            Command command;
            command.kind = Command::Kind::Begin;
            command.generation = generation;
            command.task = std::move(task);
            m_commands.push_back(std::move(command));
            m_active = true;
        }
        m_condition.notify_one();
        return generation;
    }

    void OnlineThicknessPredictionJobController::append(
        spraytrajectory::SprayTrajectory trajectory)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if(!m_active) {
                return;
            }
            if(!m_commands.empty()
                && m_commands.back().kind == Command::Kind::Append
                && m_commands.back().generation == m_generation.load()) {
                auto& segments = m_commands.back().trajectory.segments;
                for(auto& segment : trajectory.segments) {
                    segments.push_back(std::move(segment));
                }
                return;
            }
            Command command;
            command.kind = Command::Kind::Append;
            command.generation = m_generation.load();
            command.trajectory = std::move(trajectory);
            m_commands.push_back(std::move(command));
        }
        m_condition.notify_one();
    }

    void OnlineThicknessPredictionJobController::reset()
    {
        const std::uint64_t generation = ++m_generation;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_commands.clear();
            m_active = false;
            Command command;
            command.kind = Command::Kind::Reset;
            command.generation = generation;
            m_commands.push_back(std::move(command));
        }
        m_condition.notify_one();
    }

    void OnlineThicknessPredictionJobController::workerLoop()
    {
        std::unique_ptr<QOpenGLContext> context;
        std::unique_ptr<spraythickness::opengl::OpenGLThicknessPredictionBackend>
            backend;
        for(;;) {
            Command command;
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_condition.wait(lock, [this]() {
                    return m_stopRequested || !m_commands.empty();
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
                    continue;
                }
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
                    backend->beginOnline(std::move(*command.task));
                    continue;
                }
                spraythickness::ThicknessPredictionResult result =
                    backend->appendOnline(command.trajectory);
                QMetaObject::invokeMethod(this,
                    [this, generation = command.generation,
                     result = std::move(result)]() {
                        if(generation == m_generation.load()) {
                            emit fieldReady(result);
                        }
                    }, Qt::QueuedConnection);
            } catch(const std::exception& exception) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if(command.generation == m_generation.load()) {
                        m_active = false;
                        m_commands.clear();
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
}
