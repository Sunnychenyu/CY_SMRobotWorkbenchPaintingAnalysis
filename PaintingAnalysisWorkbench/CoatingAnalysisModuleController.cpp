#include "CoatingAnalysisModuleController.h"

#include "CoatingAnalysisPanel.h"
#include "CoatingAnalysisViewModel.h"
#include "PaintingAnalysisDialogService.h"
#include "PaintingAnalysisMeshAdapter.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerEvents.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"
#include "SceneEntityWorkflowController.h"
#include "ViewportReloadWorkflowController.h"

#include <AssetCore/AssetManager.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/ProjectSession.h>
#include <SimulationProject/RuntimePaths.h>
#include <SprayThicknessPrediction/DemoThicknessPrediction.h>

#include <filesystem>

namespace
{
    constexpr double kMetersToMicrometers = 1.0e6;

    const simulation_project::SceneObjectDesc* findObject(
        const simulation_project::ProjectDocument& document,
        const QString& objectId)
    {
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(QString::fromStdString(object.id) == objectId) {
                return &object;
            }
        }
        return nullptr;
    }

    std::filesystem::path projectBasePath(const simulation_project::ProjectSession& session)
    {
        return session.path().empty()
            ? simulation_project::RuntimePaths::applicationRoot()
            : session.path().parent_path();
    }

    simulation_project::AssetResolveContext makeResolveContext(
        const simulation_project::ProjectSession& session)
    {
        simulation_project::AssetResolveContext context;
        context.projectBasePath = projectBasePath(session);
        context.sourceRootPath = simulation_project::RuntimePaths::sourceRoot();
        context.dataRootPath = simulation_project::RuntimePaths::dataRoot();
        context.appRootPath = simulation_project::RuntimePaths::applicationRoot();
        context.assetSearchPaths = session.document().assetSearchPaths;
        return context;
    }
}

namespace robot_qt_viewer
{
    CoatingAnalysisModuleController::CoatingAnalysisModuleController(
        CoatingAnalysisPanel& panel,
        RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_panel(panel)
        , m_context(context)
    {
        connect(&m_panel, &CoatingAnalysisPanel::openModelRequested,
            this, &CoatingAnalysisModuleController::openModel);
        connect(&m_panel, &CoatingAnalysisPanel::predictionRequested,
            this, &CoatingAnalysisModuleController::predictThickness);
        connect(&m_panel, &CoatingAnalysisPanel::showThicknessChanged,
            this, &CoatingAnalysisModuleController::setShowThickness);
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::activate()
    {
        m_active = true;
        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                if(!services->setSurfaceScalarOverlayVisible(m_session.objectId, true)) {
                    QString error;
                    if(!services->applySurfaceScalarOverlay(m_session.overlay, &error)) {
                        m_status = error;
                        m_session.clearResult();
                        refreshViewModel();
                        publishStateChanged();
                        return;
                    }
                }
                services->setSurfaceScalarProbeEnabled(
                    m_session.showThickness,
                    m_session.objectId);
            }
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::deactivate()
    {
        m_active = false;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            clearSession();
        } else if(event.kind == RobotQtViewerEventKind::ViewportReloaded &&
            event.viewport.reloadSucceeded) {
            applyOverlayAfterReload();
        } else if(event.kind == RobotQtViewerEventKind::ProjectDocumentChanged &&
            !m_session.objectId.isEmpty() &&
            findObject(m_context.document(), m_session.objectId) == nullptr) {
            clearSession();
        }
    }

    void CoatingAnalysisModuleController::handleSurfaceScalarHover(
        const QString& objectId,
        double valueMeters,
        const QPoint& viewportPosition,
        bool hit)
    {
        if(!m_active || !m_session.hasResult || !m_session.showThickness ||
            objectId != m_session.objectId || !hit) {
            m_hasCurrentThickness = false;
            emit thicknessToolTipRequested(QString(), viewportPosition, false);
            refreshViewModel();
            return;
        }

        m_hasCurrentThickness = true;
        m_currentThicknessMeters = valueMeters;
        const QString text = QStringLiteral("Thickness: %1 um")
            .arg(valueMeters * kMetersToMicrometers, 0, 'f', 2);
        emit thicknessToolTipRequested(text, viewportPosition, true);
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::openModel()
    {
        const QString selectedPath = PaintingAnalysisDialogService::selectModelFile(&m_panel);
        if(selectedPath.isEmpty()) {
            return;
        }

        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_session.clear();
        m_hasCurrentThickness = false;
        publishStateChanged();

        SceneEntityWorkflowController importWorkflow(m_context);
        const SceneEntityImportResult importResult = importWorkflow.importSceneObjectFromPath(
            std::filesystem::path(selectedPath.toStdWString()),
            "workpiece");
        if(!importResult.success) {
            m_status = importResult.message;
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        ViewportReloadWorkflowController reloadWorkflow(m_context);
        const ViewportReloadWorkflowResult reloadResult =
            reloadWorkflow.reload(QStringLiteral("coatingAnalysisOpenModel"));
        if(!reloadResult.success) {
            importWorkflow.restoreImportState(importResult);
            reloadWorkflow.reload(QStringLiteral("coatingAnalysisOpenModelRollback"));
            m_status = reloadResult.errorMessage;
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        m_session.clear();
        m_session.objectId = importResult.entityId;
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        m_session.modelName = object != nullptr
            ? QString::fromStdString(object->name)
            : QString::fromStdWString(
                std::filesystem::path(selectedPath.toStdWString()).stem().wstring());
        m_session.sourcePath = importResult.storedPath;
        m_context.selectionModel().selectSceneObject(
            m_session.objectId,
            QStringLiteral("coatingAnalysisOpenModel"));
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->selectSceneObject(m_session.objectId);
        }
        m_status = QStringLiteral("Model loaded. Run Thickness Prediction.");
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::predictThickness()
    {
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(object == nullptr || services == nullptr) {
            m_status = QStringLiteral("The analysis model or viewport is unavailable.");
            refreshViewModel();
            return;
        }

        const std::filesystem::path sourcePath = simulation_project::AssetResolver::resolveProjectPath(
            makeResolveContext(m_context.projectSession()),
            object->sourcePath);
        std::string loadError;
        const std::shared_ptr<assetcore::ModelDesc> model =
            assetcore::AssetManager::instance().tryLoadModel(
                sourcePath.generic_u8string(),
                static_cast<float>(object->visualScale),
                &loadError);
        if(!model) {
            m_status = QString::fromStdString(loadError.empty()
                ? "Failed to load the analysis mesh."
                : loadError);
            refreshViewModel();
            return;
        }

        try {
            PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
                *model,
                object->name,
                sourcePath.generic_u8string());
            if(mesh.workpiece.empty()) {
                m_status = QStringLiteral("The model has no mesh vertices.");
                refreshViewModel();
                return;
            }
            spraythickness::ThicknessPredictionResult prediction =
                spraythickness::DemoThicknessPredictor::predict(mesh.workpiece);
            smrobot::visualization::SurfaceScalarOverlay overlay =
                PaintingAnalysisMeshAdapter::makeOverlay(
                    object->id,
                    mesh.binding,
                    prediction);
            QString applyError;
            if(!services->applySurfaceScalarOverlay(overlay, &applyError)) {
                m_status = applyError.isEmpty()
                    ? QStringLiteral("Failed to display the thickness result.")
                    : applyError;
                refreshViewModel();
                return;
            }

            m_session.binding = std::move(mesh.binding);
            m_session.prediction = std::move(prediction);
            m_session.overlay = std::move(overlay);
            m_session.hasResult = true;
            m_session.showThickness = false;
            m_hasCurrentThickness = false;
            services->setSurfaceScalarProbeEnabled(false, QString());
            m_status = QStringLiteral("Demo thickness prediction completed.");
            refreshViewModel();
            publishStateChanged();
            emit statusMessageRequested(m_status, 3000);
        } catch(const std::exception& exception) {
            services->clearSurfaceScalarOverlay(m_session.objectId);
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            refreshViewModel();
            publishStateChanged();
        }
    }

    void CoatingAnalysisModuleController::setShowThickness(bool enabled)
    {
        m_session.showThickness = enabled && m_session.hasResult;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(
                m_active && m_session.showThickness,
                m_session.objectId);
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::clearSession()
    {
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.clear();
        m_hasCurrentThickness = false;
        m_status = QStringLiteral("Open a mesh model to begin.");
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::applyOverlayAfterReload()
    {
        if(!m_session.hasResult || !m_active ||
            findObject(m_context.document(), m_session.objectId) == nullptr) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            QString error;
            if(services->applySurfaceScalarOverlay(m_session.overlay, &error)) {
                services->setSurfaceScalarProbeEnabled(
                    m_session.showThickness,
                    m_session.objectId);
            } else {
                m_status = error;
                m_session.clearResult();
                refreshViewModel();
                publishStateChanged();
            }
        }
    }

    void CoatingAnalysisModuleController::refreshViewModel()
    {
        CoatingAnalysisViewModel viewModel;
        viewModel.hasModel = !m_session.objectId.isEmpty();
        viewModel.modelName = viewModel.hasModel
            ? m_session.modelName
            : QStringLiteral("No model loaded");
        viewModel.modelPath = m_session.sourcePath;
        viewModel.status = m_status;
        viewModel.hasResult = m_session.hasResult;
        viewModel.showThickness = m_session.showThickness;
        viewModel.hasCurrentThickness = m_hasCurrentThickness;
        viewModel.currentMicrometers = m_currentThicknessMeters * kMetersToMicrometers;
        if(m_session.hasResult) {
            viewModel.minimumMicrometers =
                m_session.prediction.metrics.minThickness * kMetersToMicrometers;
            viewModel.maximumMicrometers =
                m_session.prediction.metrics.maxThickness * kMetersToMicrometers;
            viewModel.midpointMicrometers =
                (viewModel.minimumMicrometers + viewModel.maximumMicrometers) * 0.5;
            viewModel.averageMicrometers =
                m_session.prediction.metrics.averageThickness * kMetersToMicrometers;
        }
        m_panel.applyViewModel(viewModel);
        emit thicknessLegendChanged(
            m_active && viewModel.hasResult,
            viewModel.minimumMicrometers,
            viewModel.maximumMicrometers);
    }

    void CoatingAnalysisModuleController::publishStateChanged()
    {
        RobotQtViewerCoatingAnalysisPayload payload;
        payload.objectId = m_session.objectId;
        payload.hasResult = m_session.hasResult;
        payload.showThickness = m_session.showThickness;
        if(m_session.hasResult) {
            payload.minimumThicknessMeters = m_session.prediction.metrics.minThickness;
            payload.maximumThicknessMeters = m_session.prediction.metrics.maxThickness;
        }
        m_context.documentController().publishCoatingAnalysisChanged(
            payload,
            QStringLiteral("coatingAnalysis"));
    }
}
