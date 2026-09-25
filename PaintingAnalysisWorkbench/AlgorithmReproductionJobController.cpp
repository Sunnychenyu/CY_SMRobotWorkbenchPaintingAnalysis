#include "AlgorithmReproductionJobController.h"

#include <QMetaObject>
#include <QString>
#include <QThread>

#include <exception>
#include <string>
#include <utility>

namespace robot_qt_viewer
{
    AlgorithmReproductionJobController::AlgorithmReproductionJobController(
        QObject* parent)
        : QObject(parent)
    {
        m_thread = QThread::create([this]() { workerLoop(); });
        m_thread->start();
    }

    AlgorithmReproductionJobController::~AlgorithmReproductionJobController()
    {
        cancel();
        m_stopRequested.store(true);
        m_taskCondition.notify_all();
        if(m_thread != nullptr) {
            m_thread->wait();
            delete m_thread;
            m_thread = nullptr;
        }
    }

    bool AlgorithmReproductionJobController::start(
        spraythickness::AlgorithmReproductionTask task,
        QString* errorMessage)
    {
        const auto fail = [errorMessage](const QString& message) {
            if(errorMessage != nullptr) {
                *errorMessage = message;
            }
            return false;
        };
        if(isRunning()) {
            return fail(QStringLiteral("An algorithm reproduction task is already running."));
        }
        if(m_thread == nullptr || !m_thread->isRunning()) {
            return fail(QStringLiteral("The algorithm reproduction worker is not running."));
        }
        m_cancelRequested.store(false);
        {
            std::lock_guard<std::mutex> lock(m_taskMutex);
            if(m_pendingTask.has_value()) {
                return fail(QStringLiteral("An algorithm reproduction task is already queued."));
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

    void AlgorithmReproductionJobController::cancel()
    {
        m_cancelRequested.store(true);
    }

    bool AlgorithmReproductionJobController::isRunning() const
    {
        return m_running.load();
    }

    void AlgorithmReproductionJobController::workerLoop()
    {
        while(!m_stopRequested.load()) {
            std::optional<spraythickness::AlgorithmReproductionTask> task;
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
                spraythickness::published::ReproductionExecution execution;
                execution.cancelRequested = &m_cancelRequested;
                execution.progress = [this](double value, const std::string& message) {
                    postProgress(value, message);
                };
                postFinished(spraythickness::AlgorithmReproducer::run(*task, execution));
            } catch(const std::exception& exception) {
                postFailure(QString::fromLocal8Bit(exception.what()));
            } catch(...) {
                postFailure(QStringLiteral("Unknown algorithm reproduction error."));
            }

            m_running.store(false);
            postRunningChanged(false);
        }
    }

    void AlgorithmReproductionJobController::postProgress(
        double progress,
        const std::string& message)
    {
        QMetaObject::invokeMethod(this, [this, progress, message]() {
            emit progressChanged(progress, QString::fromStdString(message));
        }, Qt::QueuedConnection);
    }

    void AlgorithmReproductionJobController::postFinished(
        spraythickness::AlgorithmReproductionResult result)
    {
        QMetaObject::invokeMethod(this, [this, result = std::move(result)]() {
            emit reproductionFinished(result);
        }, Qt::QueuedConnection);
    }

    void AlgorithmReproductionJobController::postFailure(QString message)
    {
        QMetaObject::invokeMethod(this, [this, message = std::move(message)]() {
            emit reproductionFailed(message);
        }, Qt::QueuedConnection);
    }

    void AlgorithmReproductionJobController::postRunningChanged(bool running)
    {
        QMetaObject::invokeMethod(this, [this, running]() {
            emit runningChanged(running);
        }, Qt::QueuedConnection);
    }
}
