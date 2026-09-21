#include "CoatingAnalysisModuleController.h"

#include "CoatingAnalysisInfoPanel.h"
#include "CoatingAnalysisPanel.h"
#include "CoatingAnalysisTreePanel.h"
#include "CoatingAnalysisVisibilityBar.h"
#include "CoatingAnalysisWaypointDialog.h"
#include "AxisymmetricProfileSelectionDialog.h"
#include "PaintingAnalysisMeshAdapter.h"
#include "PublishedReproductionAdapter.h"
#include "PublishedReproductionDisplayAdapter.h"
#include "ThicknessPredictionJobController.h"
#include "AlgorithmReproductionJobController.h"
#include "PaintingAnalysisDialogService.h"
#include "SimulationExperiment.h"
#include "CoatingAnalysisLanguage.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerEvents.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"
#include "SceneEntityWorkflowController.h"
#include "ViewportReloadWorkflowController.h"

#include <AssetCore/AssetManager.h>
#include <CustomLog/CustomLog.h>
#include <SprayThicknessPrediction/RotationalSurfaceFitter.h>
#include <SprayThicknessPredictionOpenGL/PeriodicSectorReduction.h>
#include <SprayThicknessPredictionOpenGL/AxisymmetricProfileReduction.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/ProjectSession.h>
#include <SimulationProject/RuntimePaths.h>
#include <SprayTrajectoryCore/SprayTrajectoryIo.h>
#include <QDir>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QMenu>
#include <QSettings>
#include <QTextStream>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <type_traits>

namespace
{
    constexpr double kMetersToMicrometers = 1.0e6;
    constexpr double kValidationActiveThicknessMeters = 1.0e-12;
    const QString kFixedModelPath = QStringLiteral(
        "K:/rs2026/data/ThickPredictData/STL/libing/yangjian_2_0.02.STL");
    const QString kFixedTrajectoryPath = QStringLiteral(
        "K:/rs2026/data/ThickPredictData/PointData/libing_pointdata/MergedTrajectory_0_0.txt");
    constexpr const char* kSimulationExportDirectorySettingsKey =
        "PaintingAnalysis/SimulationExportDirectory";
    constexpr const char* kReproductionExportDirectorySettingsKey =
        "PaintingAnalysis/ReproductionExportDirectory";

    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }

    std::size_t activeThicknessCount(const spraythickness::ThicknessField& field)
    {
        return static_cast<std::size_t>(std::count_if(
            field.results.begin(),
            field.results.end(),
            [](const spraythickness::ThicknessSampleResult& result) {
                return std::abs(result.thickness) > kValidationActiveThicknessMeters;
            }));
    }

    std::size_t countActiveSprayIntervals(
        const spraytrajectory::SprayTrajectory& trajectory)
    {
        const auto samples = spraytrajectory::SprayTrajectorySampler::originalSamples(
            trajectory);
        std::size_t activeIntervals = 0;
        for(std::size_t index = 0; index + 1 < samples.size(); ++index) {
            if(!samples[index].sprayEnabled) {
                continue;
            }
            const double duration = samples[index + 1].time - samples[index].time;
            if(std::isfinite(duration) && duration > 0.0) {
                ++activeIntervals;
            }
        }
        return activeIntervals;
    }

    double activeSprayDurationSeconds(
        const spraytrajectory::SprayTrajectory& trajectory)
    {
        const auto samples = spraytrajectory::SprayTrajectorySampler::originalSamples(
            trajectory);
        double duration = 0.0;
        for(std::size_t index = 0; index + 1 < samples.size(); ++index) {
            if(samples[index].sprayEnabled) {
                duration += std::max(0.0, samples[index + 1].time - samples[index].time);
            }
        }
        return duration;
    }

    double thicknessVolumeCubicMillimeters(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const spraythickness::ThicknessField& thicknessField)
    {
        if(thicknessField.results.size() != workpiece.samples.size()) {
            return 0.0;
        }

        std::vector<double> thicknesses(workpiece.samples.size(), 0.0);
        for(const auto& result : thicknessField.results) {
            if(result.sampleIndex < thicknesses.size()) {
                thicknesses[result.sampleIndex] = std::max(0.0, result.thickness);
            }
        }

        double volumeCubicMeters = 0.0;
        for(std::size_t index = 0; index + 2 < workpiece.triangleIndices.size(); index += 3) {
            const std::uint32_t first = workpiece.triangleIndices[index];
            const std::uint32_t second = workpiece.triangleIndices[index + 1];
            const std::uint32_t third = workpiece.triangleIndices[index + 2];
            if(first >= workpiece.samples.size() || second >= workpiece.samples.size()
                || third >= workpiece.samples.size()) {
                continue;
            }
            const Eigen::Vector3d& a = workpiece.samples[first].position;
            const Eigen::Vector3d& b = workpiece.samples[second].position;
            const Eigen::Vector3d& c = workpiece.samples[third].position;
            const double areaSquareMeters = 0.5 * (b - a).cross(c - a).norm();
            const double averageThickness =
                (thicknesses[first] + thicknesses[second] + thicknesses[third]) / 3.0;
            volumeCubicMeters += areaSquareMeters * averageThickness;
        }
        return volumeCubicMeters * 1.0e9;
    }

    QString simulationExportFileName(
        const robot_qt_viewer::SimulationExperimentParameters& parameters,
        double activeSprayDurationSeconds,
        double thicknessVolumeCubicMillimeters)
    {
        return QStringLiteral("distance_%1mm-angle_%2deg-time_%3s-volume_%4mm3.csv")
            .arg(parameters.sprayDistanceMillimeters, 0, 'f', 2)
            .arg(parameters.incidenceAngleDegrees, 0, 'f', 1)
            .arg(activeSprayDurationSeconds, 0, 'f', 3)
            .arg(thicknessVolumeCubicMillimeters, 0, 'f', 6);
    }

    QString predictionModeName(robot_qt_viewer::PredictionInputMode mode)
    {
        using robot_qt_viewer::PredictionInputMode;
        switch(mode) {
        case PredictionInputMode::CompleteAllSprayPoints:
            return QStringLiteral("Complete - all spray points");
        case PredictionInputMode::LocalAllSprayPoints:
            return QStringLiteral("Local - all spray points");
        case PredictionInputMode::CompleteSpatialFilteredSprayPoints:
            return QStringLiteral("Complete - spatial filtering");
        case PredictionInputMode::LocalSpatialFilteredSprayPoints:
            return QStringLiteral("Local - spatial filtering");
        case PredictionInputMode::AxisymmetricProfileSpatialFilteredSprayPoints:
            return QStringLiteral("Axisymmetric profile - spatial filtering");
        case PredictionInputMode::CompleteSpatialFilteredCandidateVertices:
            return QStringLiteral("Complete - candidate vertices");
        case PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices:
            return QStringLiteral("Adaptive mesh - candidate filtering");
        case PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh:
            return QStringLiteral("Local candidates - full BVH");
        }
        return QStringLiteral("Unknown");
    }

    bool usesNonzeroVertexComparison(robot_qt_viewer::PredictionInputMode mode)
    {
        using robot_qt_viewer::PredictionInputMode;
        switch(mode) {
        case PredictionInputMode::LocalAllSprayPoints:
        case PredictionInputMode::LocalSpatialFilteredSprayPoints:
        case PredictionInputMode::AxisymmetricProfileSpatialFilteredSprayPoints:
        case PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices:
        case PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh:
            return true;
        case PredictionInputMode::CompleteAllSprayPoints:
        case PredictionInputMode::CompleteSpatialFilteredSprayPoints:
        case PredictionInputMode::CompleteSpatialFilteredCandidateVertices:
            return false;
        }
        return false;
    }

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

    std::filesystem::path normalizedPath(const std::filesystem::path& path)
    {
        std::error_code error;
        std::filesystem::path normalized = std::filesystem::weakly_canonical(path, error);
        if(error) {
            normalized = std::filesystem::absolute(path, error);
        }
        return (error ? path : normalized).lexically_normal();
    }

    Eigen::Isometry3d makeTransform(const simulation_project::TransformDesc& desc)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(desc.x, desc.y, desc.z);
        transform.linear() =
            Eigen::AngleAxisd(desc.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        return transform;
    }

    robot_qt_viewer::CoatingAnalysisModelInfo makeModelInfo(
        const assetcore::ModelDesc& model,
        const Eigen::Isometry3d& worldFromModel)
    {
        robot_qt_viewer::CoatingAnalysisModelInfo info;
        info.subMeshCount = model.subMeshCount();
        Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::max());
        Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::lowest());

        for(const assetcore::SubMeshDesc& subMesh : model.subMeshes()) {
            const assetcore::GeometryDesc& geometry = subMesh.geometry;
            info.vertexCount += geometry.positions.size();
            for(const Eigen::Vector3f& position : geometry.positions) {
                const Eigen::Vector3d worldPosition =
                    worldFromModel * position.cast<double>();
                minimum = minimum.cwiseMin(worldPosition);
                maximum = maximum.cwiseMax(worldPosition);
            }

            if(!geometry.indices.empty()) {
                for(std::size_t offset = 0; offset + 2 < geometry.indices.size(); offset += 3) {
                    if(geometry.indices[offset] < geometry.positions.size() &&
                        geometry.indices[offset + 1] < geometry.positions.size() &&
                        geometry.indices[offset + 2] < geometry.positions.size()) {
                        ++info.triangleCount;
                    }
                }
            } else {
                info.triangleCount += geometry.positions.size() / 3;
            }
        }

        info.valid = info.vertexCount > 0;
        if(info.valid) {
            const Eigen::Vector3d size = maximum - minimum;
            info.sizeXMeters = size.x();
            info.sizeYMeters = size.y();
            info.sizeZMeters = size.z();
        }
        return info;
    }

    robot_qt_viewer::CoatingAnalysisTrajectoryInfo makeTrajectoryInfo(
        const spraytrajectory::SprayTrajectory& trajectory,
        std::size_t warningCount)
    {
        robot_qt_viewer::CoatingAnalysisTrajectoryInfo info;
        info.warningCount = warningCount;
        const std::vector<spraytrajectory::SprayPathPoint> points =
            trajectory.flattenedPoints();
        info.pointCount = points.size();
        if(points.empty()) {
            return info;
        }

        Eigen::Vector3d minimum = points.front().tcpPose.translation();
        Eigen::Vector3d maximum = minimum;
        for(std::size_t index = 1; index < points.size(); ++index) {
            const Eigen::Vector3d position = points[index].tcpPose.translation();
            minimum = minimum.cwiseMin(position);
            maximum = maximum.cwiseMax(position);
            info.pathLengthMeters +=
                (position - points[index - 1].tcpPose.translation()).norm();
        }

        info.startTimeSeconds = points.front().time;
        info.endTimeSeconds = points.back().time;
        info.durationSeconds = std::max(0.0,
            info.endTimeSeconds - info.startTimeSeconds);
        info.averageSpeedMetersPerSecond = info.durationSeconds > 0.0
            ? info.pathLengthMeters / info.durationSeconds
            : 0.0;
        const Eigen::Vector3d range = maximum - minimum;
        info.rangeXMeters = range.x();
        info.rangeYMeters = range.y();
        info.rangeZMeters = range.z();
        info.valid = true;
        return info;
    }

    robot_qt_viewer::CoatingPredictionDebugTriangle makeDebugTriangle(
        const sprayworkpiece::WorkpieceModel& workpiece,
        std::size_t triangleIndex)
    {
        robot_qt_viewer::CoatingPredictionDebugTriangle result;
        const std::size_t offset = triangleIndex * 3;
        if(offset + 2 >= workpiece.triangleIndices.size()) {
            return result;
        }
        const std::uint32_t indices[3] = {
            workpiece.triangleIndices[offset],
            workpiece.triangleIndices[offset + 1],
            workpiece.triangleIndices[offset + 2]};
        if(indices[0] >= workpiece.samples.size() ||
            indices[1] >= workpiece.samples.size() ||
            indices[2] >= workpiece.samples.size()) {
            return result;
        }
        const Eigen::Vector3d positions[3] = {
            workpiece.samples[indices[0]].position,
            workpiece.samples[indices[1]].position,
            workpiece.samples[indices[2]].position};
        result.ax = positions[0].x();
        result.ay = positions[0].y();
        result.az = positions[0].z();
        result.bx = positions[1].x();
        result.by = positions[1].y();
        result.bz = positions[1].z();
        result.cx = positions[2].x();
        result.cy = positions[2].y();
        result.cz = positions[2].z();
        return result;
    }

    robot_qt_viewer::CoatingPredictionDebugState makeSeedDebugState(
        const robot_qt_viewer::PaintingAnalysisMeshData& mesh,
        std::size_t triangleIndex)
    {
        robot_qt_viewer::CoatingPredictionDebugState state;
        state.visible = true;
        if(triangleIndex < mesh.workpiece.triangleIndices.size() / 3) {
            state.seedTriangles.push_back(makeDebugTriangle(mesh.workpiece, triangleIndex));
        }
        return state;
    }

    void configureAxisDebugState(
        robot_qt_viewer::CoatingPredictionDebugState& state,
        const sprayworkpiece::WorkpieceModel& workpiece,
        const Eigen::Vector3d& axisOrigin,
        const Eigen::Vector3d& axisDirection)
    {
        state.axisOriginX = axisOrigin.x();
        state.axisOriginY = axisOrigin.y();
        state.axisOriginZ = axisOrigin.z();
        state.axisDirectionX = axisDirection.x();
        state.axisDirectionY = axisDirection.y();
        state.axisDirectionZ = axisDirection.z();

        Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::max());
        Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::lowest());
        for(const auto& sample : workpiece.samples) {
            minimum = minimum.cwiseMin(sample.position);
            maximum = maximum.cwiseMax(sample.position);
        }
        const double diagonal = (maximum - minimum).norm();
        state.axisLength = std::max(0.02, diagonal * 0.75);
        state.markerRadius = std::max(0.0005, diagonal * 0.006);
    }

    robot_qt_viewer::CoatingPredictionDebugState makeRotationFitDebugState(
        const robot_qt_viewer::PaintingAnalysisMeshData& mesh,
        const spraythickness::CylindricalSurfaceFitResult& fit)
    {
        robot_qt_viewer::CoatingPredictionDebugState state;
        state.visible = true;
        state.seedTriangles.push_back(makeDebugTriangle(mesh.workpiece, fit.seedTriangleIndex));
        state.cylindricalTriangles.reserve(fit.selectedTriangleIndices.size());
        for(const std::size_t triangleIndex : fit.selectedTriangleIndices) {
            state.cylindricalTriangles.push_back(
                makeDebugTriangle(mesh.workpiece, triangleIndex));
        }
        configureAxisDebugState(
            state, mesh.workpiece, fit.axisOrigin, fit.axisDirection);
        return state;
    }
}

namespace robot_qt_viewer
{
    namespace
    {
        constexpr const char* kSimulationPlateObjectId = "__coating_simulation_plate__";
    }
    struct CoatingAnalysisModuleController::AxisymmetricProfileState
    {
        spraythickness::opengl::AxisymmetricProfileSlice slice;
        spraythickness::opengl::AxisymmetricProfileSelection selection;
        spraythickness::opengl::AxisymmetricProfileReduction reduction;
        QString details;
    };

    CoatingAnalysisModuleController::CoatingAnalysisModuleController(
        CoatingAnalysisPanel& panel,
        CoatingAnalysisTreePanel& treePanel,
        CoatingAnalysisInfoPanel& infoPanel,
        CoatingAnalysisVisibilityBar& visibilityBar,
        RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_panel(panel)
        , m_treePanel(treePanel)
        , m_infoPanel(infoPanel)
        , m_visibilityBar(visibilityBar)
        , m_context(context)
        , m_predictionJob(std::make_unique<ThicknessPredictionJobController>())
        , m_reproductionJob(
              std::make_unique<AlgorithmReproductionJobController>())
        , m_axisymmetricProfile(std::make_unique<AxisymmetricProfileState>())
    {
        connect(&m_panel, &CoatingAnalysisPanel::openModelRequested,
            this, &CoatingAnalysisModuleController::openModelFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::openTrajectoryRequested,
            this, &CoatingAnalysisModuleController::openTrajectoryFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::selectModelFileRequested,
            this, &CoatingAnalysisModuleController::selectModelFileFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::selectTrajectoryFileRequested,
            this, &CoatingAnalysisModuleController::selectTrajectoryFileFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::predictionRequested,
            this, &CoatingAnalysisModuleController::predictThickness);
        connect(&m_panel, &CoatingAnalysisPanel::cancelPredictionRequested,
            this, &CoatingAnalysisModuleController::cancelPrediction);
        connect(&m_panel, &CoatingAnalysisPanel::setReferenceRequested,
            this, &CoatingAnalysisModuleController::setCurrentResultAsReference);
        connect(&m_panel, &CoatingAnalysisPanel::clearReferenceRequested,
            this, &CoatingAnalysisModuleController::clearReferenceResult);
        connect(&m_panel, &CoatingAnalysisPanel::checkReferenceRequested,
            this, &CoatingAnalysisModuleController::checkCurrentResultAgainstReference);
        connect(&m_panel, &CoatingAnalysisPanel::localInputPreviewRequested,
            this, &CoatingAnalysisModuleController::previewLocalInputs);
        connect(&m_panel, &CoatingAnalysisPanel::profileRegionSelectionRequested,
            this, &CoatingAnalysisModuleController::selectAxisymmetricProfileRegion);
        connect(&m_panel, &CoatingAnalysisPanel::adaptiveRegionSelectionRequested,
            this, &CoatingAnalysisModuleController::selectAxisymmetricProfileRegion);
        connect(&m_panel, &CoatingAnalysisPanel::axisymmetricProfileSampleCountChanged,
            this, [this]() {
                if(anyPredictionRunning()
                    || !m_panel.axisymmetricProfilePredictionEnabled()
                    || !m_axisymmetricProfile->selection.enabled) {
                    return;
                }
                rebuildAxisymmetricProfileReduction();
            });
        connect(&m_panel, &CoatingAnalysisPanel::localDebugVisibilityChanged,
            this, &CoatingAnalysisModuleController::setLocalDebugVisibility);
        connect(&m_panel, &CoatingAnalysisPanel::localPreviewParametersChanged,
            this, [this]() {
                if(anyPredictionRunning()) {
                    return;
                }
                if(m_session.hasResult) {
                    if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                        services->setSurfaceScalarProbeEnabled(false, QString());
                        services->clearSurfaceScalarOverlay(m_session.objectId);
                        services->clearCoatingPredictionModel(m_session.objectId);
                    }
                    m_session.clearResult();
                    publishStateChanged();
                }
                if(!m_panel.rotationBasedPredictionEnabled()
                    && !m_panel.adaptiveMeshPredictionEnabled()) {
                    m_hasLocalPreview = false;
                    m_localPreviewDetails.clear();
                    clearAxisymmetricProfileSelection();
                    if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                        services->clearCoatingPredictionDebugState();
                    }
                    applyModelVisibilityOverrides();
                    refreshViewModel();
                    return;
                }
                if(m_panel.axisymmetricProfilePredictionEnabled()
                    || m_panel.localCandidateVertexPredictionEnabled()) {
                    clearAxisymmetricProfileSelection();
                    applyModelVisibilityOverrides();
                    if(ensurePreviewWorkpieceLoaded() && hasEffectiveRotationAxis()) {
                        if(ensureAxisymmetricProfileSlice()) {
                            updateAxisymmetricProfileDebugState();
                            m_status = m_panel.localCandidateVertexPredictionEnabled()
                                ? QStringLiteral(
                                    "Select the local prediction region; complete-model BVH will be used for occlusion.")
                                : QStringLiteral(
                                    "Select a profile prediction region to prepare axisymmetric prediction.");
                        }
                    }
                    refreshViewModel();
                    return;
                }
                if(m_panel.adaptiveMeshPredictionEnabled()) {
                    m_hasLocalPreview = false;
                    m_localPreviewDetails.clear();
                    if(ensurePreviewWorkpieceLoaded() && hasEffectiveRotationAxis()) {
                        if(ensureAxisymmetricProfileSlice()) {
                            updateAxisymmetricProfileDebugState();
                            m_status = QStringLiteral(
                                "Select the dense prediction region to prepare adaptive mesh prediction.");
                        }
                    }
                    refreshViewModel();
                    return;
                }
                applyModelVisibilityOverrides();
                if(ensurePreviewWorkpieceLoaded() && hasEffectiveRotationAxis()
                    && !m_session.trajectory.empty()) {
                    previewLocalInputs();
                } else {
                    refreshViewModel();
                }
            });
        connect(&m_panel, &CoatingAnalysisPanel::rotationAxisChanged,
            this, [this]() {
                if(anyPredictionRunning()) {
                    return;
                }
                if(!m_panel.rotationBasedPredictionEnabled()
                    && !m_panel.adaptiveMeshPredictionEnabled()) {
                    return;
                }
                clearAxisymmetricProfileSelection();
                m_hasLocalPreview = false;
                m_localPreviewDetails.clear();
                if(ensurePreviewWorkpieceLoaded() && hasEffectiveRotationAxis()
                    && ensureAxisymmetricProfileSlice()) {
                    updateAxisymmetricProfileDebugState();
                    m_status = m_panel.adaptiveMeshPredictionEnabled()
                        ? QStringLiteral("Rotation axis changed. Select the dense prediction region again.")
                        : QStringLiteral("Rotation axis changed. Select the profile prediction region again.");
                }
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel, &CoatingAnalysisPanel::spatialGridParametersChanged,
            this, [this]() {
                if(anyPredictionRunning() || !m_session.hasResult) {
                    return;
                }
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->setSurfaceScalarProbeEnabled(false, QString());
                    services->clearSurfaceScalarOverlay(m_session.objectId);
                }
                m_session.clearResult();
                m_hasCurrentThickness = false;
                m_status = QStringLiteral(
                    "Spatial grid settings changed. Run prediction to apply them.");
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel, &CoatingAnalysisPanel::depositionDirectionsChanged,
            this, [this]() {
                if(anyPredictionRunning()) {
                    return;
                }
                if(m_session.hasResult) {
                    if(RobotQtViewerViewportServices* services =
                            m_context.viewportServices()) {
                        services->setSurfaceScalarProbeEnabled(false, QString());
                        services->clearSurfaceScalarOverlay(m_session.objectId);
                        services->clearCoatingPredictionModel(m_session.objectId);
                    }
                    m_session.clearResult();
                    m_hasCurrentThickness = false;
                    emit thicknessToolTipRequested(QString(), QPoint(), false);
                }
                m_status = QStringLiteral(
                    "Deposition directions changed. Run prediction to apply them.");
                if(m_hasLocalPreview && m_panel.periodicLocalPredictionEnabled()) {
                    previewLocalInputs();
                    return;
                }
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel, &CoatingAnalysisPanel::workpieceChanged,
            this, &CoatingAnalysisModuleController::selectWorkpiece);
        connect(&m_panel, &CoatingAnalysisPanel::rotationSurfacePickRequested,
            this, [this]() {
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->beginRotationSurfacePick();
                    m_status = QStringLiteral("Click a cylindrical side face in the viewport.");
                    refreshViewModel();
                }
            });
        connect(&m_panel, &CoatingAnalysisPanel::enterSimulationRequested,
            this, &CoatingAnalysisModuleController::enterSimulation);
        connect(&m_panel, &CoatingAnalysisPanel::exitSimulationRequested,
            this, &CoatingAnalysisModuleController::exitSimulation);
        connect(&m_panel, &CoatingAnalysisPanel::simulationParametersChanged,
            this, [this]() {
                if(simulationActive() && !anyPredictionRunning()) {
                    rebuildSimulation(false);
                }
            });
        connect(&m_panel, &CoatingAnalysisPanel::simulationPredictionRequested,
            this, &CoatingAnalysisModuleController::runSimulationPrediction);
        connect(&m_panel, &CoatingAnalysisPanel::simulationExportRequested,
            this, &CoatingAnalysisModuleController::exportSimulationResult);
        connect(&m_panel, &CoatingAnalysisPanel::enterReproductionRequested,
            this, &CoatingAnalysisModuleController::enterReproduction);
        connect(&m_panel, &CoatingAnalysisPanel::exitReproductionRequested,
            this, &CoatingAnalysisModuleController::exitReproduction);
        connect(&m_panel, &CoatingAnalysisPanel::reproductionRequested,
            this, &CoatingAnalysisModuleController::runAlgorithmReproduction);
        connect(&m_panel, &CoatingAnalysisPanel::reproductionInputsChanged,
            this, [this]() {
                if(!reproductionActive() || anyPredictionRunning()) {
                    return;
                }
                if(m_session.hasReproductionResult) {
                    if(RobotQtViewerViewportServices* services =
                            m_context.viewportServices()) {
                        services->setSurfaceScalarProbeEnabled(false, QString());
                        services->clearSurfaceScalarOverlay(m_session.objectId);
                    }
                    m_session.clearResult();
                }
                m_reproductionStatus = QStringLiteral(
                    "Reproduction inputs changed. Run the selected method again.");
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel, &CoatingAnalysisPanel::reproductionTemplateRequested,
            this, &CoatingAnalysisModuleController::createReproductionTemplate);
        connect(&m_panel, &CoatingAnalysisPanel::cancelReproductionRequested,
            this, &CoatingAnalysisModuleController::cancelAlgorithmReproduction);
        connect(&m_panel, &CoatingAnalysisPanel::reproductionExportRequested,
            this, &CoatingAnalysisModuleController::exportAlgorithmReproduction);

        connect(&m_treePanel, &CoatingAnalysisTreePanel::waypointInfoRequested,
            this, &CoatingAnalysisModuleController::handleWaypointInfoRequested);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::modelVisibilityToggleRequested,
            this, &CoatingAnalysisModuleController::handleModelVisibilityToggleRequested);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::modelSetAsWorkpiece,
            this, &CoatingAnalysisModuleController::handleModelSetAsWorkpiece);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::thicknessClearRequested,
            this, &CoatingAnalysisModuleController::handleThicknessClearRequested);

        connect(&m_visibilityBar, &CoatingAnalysisVisibilityBar::showModelChanged,
            this, &CoatingAnalysisModuleController::setShowModel);
        connect(&m_visibilityBar, &CoatingAnalysisVisibilityBar::showTrajectoryChanged,
            this, &CoatingAnalysisModuleController::setShowTrajectory);
        connect(&m_visibilityBar, &CoatingAnalysisVisibilityBar::showSprayPointsChanged,
            this, &CoatingAnalysisModuleController::setShowSprayPoints);
        connect(&m_visibilityBar, &CoatingAnalysisVisibilityBar::showThicknessChanged,
            this, &CoatingAnalysisModuleController::setShowThickness);
        connect(&m_visibilityBar, &CoatingAnalysisVisibilityBar::thicknessPickChanged,
            this, &CoatingAnalysisModuleController::setThicknessPickEnabled);

        connect(m_predictionJob.get(), &ThicknessPredictionJobController::progressChanged,
            this, &CoatingAnalysisModuleController::handlePredictionProgress);
        connect(m_predictionJob.get(), &ThicknessPredictionJobController::predictionFinished,
            this, &CoatingAnalysisModuleController::handlePredictionFinished);
        connect(m_predictionJob.get(), &ThicknessPredictionJobController::predictionFailed,
            this, &CoatingAnalysisModuleController::handlePredictionFailed);
        connect(m_predictionJob.get(), &ThicknessPredictionJobController::runningChanged,
            this, [this](bool running) {
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->setGpuPredictionBusy(running);
                }
                refreshViewModel();
            });
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::progressChanged,
            this, &CoatingAnalysisModuleController::handleReproductionProgress);
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::reproductionFinished,
            this, &CoatingAnalysisModuleController::handleReproductionFinished);
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::reproductionFailed,
            this, &CoatingAnalysisModuleController::handleReproductionFailed);
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::runningChanged,
            this, [this](bool) { refreshViewModel(); });
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::resetReferenceResult()
    {
        m_referenceThickness = spraythickness::ThicknessField();
        m_referenceActiveVertexCount = 0;
        m_validationDetails = QStringLiteral("No reference result.");
    }

    void CoatingAnalysisModuleController::setCurrentResultAsReference()
    {
        if(anyPredictionRunning()) {
            return;
        }
        if(!m_session.hasResult || m_session.prediction.field.empty()) {
            m_status = QStringLiteral("Run a prediction before setting a reference.");
            refreshViewModel();
            return;
        }

        m_referenceThickness = m_session.prediction.field;
        m_referenceActiveVertexCount = activeThicknessCount(m_referenceThickness);
        m_validationDetails = QStringLiteral(
            "Reference ready\n"
            "Mode       : %1\n"
            "Vertices   : %2\n"
            "Active     : %3")
            .arg(predictionModeName(m_panel.predictionInputMode()))
            .arg(static_cast<qulonglong>(m_referenceThickness.results.size()))
            .arg(static_cast<qulonglong>(m_referenceActiveVertexCount));
        m_status = QStringLiteral("Current prediction stored as the reference result.");
        refreshViewModel();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::clearReferenceResult()
    {
        if(anyPredictionRunning() || m_referenceThickness.empty()) {
            return;
        }
        resetReferenceResult();
        if(m_session.showRelativeError) {
            m_session.overlay = m_session.thicknessOverlay;
            m_session.showRelativeError = false;
            QString error;
            applyCurrentSurfaceOverlay(&error);
        }
        m_status = QStringLiteral("Reference result cleared.");
        refreshViewModel();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::checkCurrentResultAgainstReference()
    {
        if(anyPredictionRunning()) {
            return;
        }
        if(m_referenceThickness.empty()) {
            m_status = QStringLiteral("Set a reference result before checking this prediction.");
            refreshViewModel();
            return;
        }
        if(!m_session.hasResult || m_session.prediction.field.empty()) {
            m_status = QStringLiteral("Run prediction before checking it against the reference.");
            refreshViewModel();
            return;
        }

        const spraythickness::ThicknessField& current = m_session.prediction.field;
        if(current.results.size() != m_referenceThickness.results.size()) {
            m_validationDetails = QStringLiteral(
                "Validation unavailable\n"
                "Reference vertices: %1\n"
                "Current vertices  : %2\n"
                "The optimized result was not mapped to the complete model.")
                .arg(static_cast<qulonglong>(m_referenceThickness.results.size()))
                .arg(static_cast<qulonglong>(current.results.size()));
            m_status = QStringLiteral("Validation stopped: complete-model vertex counts differ.");
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        std::vector<double> absoluteErrors;
        absoluteErrors.reserve(current.results.size());
        const PredictionInputMode comparisonMode = m_panel.predictionInputMode();
        const bool nonzeroOnly = usesNonzeroVertexComparison(comparisonMode);
        std::vector<std::uint8_t> comparisonMask(current.results.size(), 1U);
        if(nonzeroOnly) {
            for(std::size_t index = 0; index < current.results.size(); ++index) {
                comparisonMask[index] = std::abs(current.results[index].thickness)
                    > kValidationActiveThicknessMeters ? 1U : 0U;
            }
        }
        std::size_t referenceActive = 0;
        std::size_t currentActive = 0;
        std::size_t commonActive = 0;
        std::size_t missingReference = 0;
        std::size_t currentOnly = 0;
        double absoluteErrorSum = 0.0;
        double squaredErrorSum = 0.0;
        double maximumAbsoluteError = 0.0;
        double maximumRelativeError = 0.0;

        for(std::size_t index = 0; index < current.results.size(); ++index) {
            if(comparisonMask[index] == 0U) {
                continue;
            }
            const spraythickness::ThicknessSampleResult& reference =
                m_referenceThickness.results[index];
            const spraythickness::ThicknessSampleResult& candidate = current.results[index];
            if(reference.sampleIndex != candidate.sampleIndex) {
                m_validationDetails = QStringLiteral(
                    "Validation unavailable\n"
                    "Vertex index mapping differs at result index %1.")
                    .arg(static_cast<qulonglong>(index));
                m_status = QStringLiteral("Validation stopped: vertex index mapping differs.");
                refreshViewModel();
                emit statusMessageRequested(m_status, 5000);
                return;
            }

            const bool referenceHasThickness =
                std::abs(reference.thickness) > kValidationActiveThicknessMeters;
            const bool candidateHasThickness =
                std::abs(candidate.thickness) > kValidationActiveThicknessMeters;
            referenceActive += referenceHasThickness ? 1 : 0;
            currentActive += candidateHasThickness ? 1 : 0;
            if(!candidateHasThickness && referenceHasThickness) {
                ++missingReference;
            }

            currentOnly += !referenceHasThickness && candidateHasThickness ? 1 : 0;
            if(referenceHasThickness && candidateHasThickness) {
                ++commonActive;
            }

            const double absoluteError = std::abs(candidate.thickness - reference.thickness);
            absoluteErrors.push_back(absoluteError);
            absoluteErrorSum += absoluteError;
            squaredErrorSum += absoluteError * absoluteError;
            maximumAbsoluteError = std::max(maximumAbsoluteError, absoluteError);
            maximumRelativeError = std::max(
                maximumRelativeError,
                absoluteError / std::max(
                    std::abs(reference.thickness),
                    kValidationActiveThicknessMeters));
        }

        if(absoluteErrors.empty()) {
            m_validationDetails = QStringLiteral(
                "Validation unavailable\n"
                "The current mode produced no nonzero thickness vertices to compare.");
            m_status = QStringLiteral("Validation stopped: no active current vertices.");
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        std::sort(absoluteErrors.begin(), absoluteErrors.end());
        const std::size_t p95Index = static_cast<std::size_t>(std::ceil(
            static_cast<double>(absoluteErrors.size()) * 0.95)) - 1;
        const double p95AbsoluteError = absoluteErrors[p95Index];
        const std::size_t comparisonCount = absoluteErrors.size();
        const double meanAbsoluteError = absoluteErrorSum
            / static_cast<double>(comparisonCount);
        const double rootMeanSquareError = std::sqrt(
            squaredErrorSum / static_cast<double>(comparisonCount));
        m_validationDetails = QStringLiteral(
            "Validation complete\n"
            "Mode             : %1\n"
            "Compared vertices : %2 (%3)\n"
            "Excluded vertices: %4\n"
            "Reference active : %5\n"
            "Current active   : %6\n"
            "Common active    : %7\n"
            "Missing reference: %8\n"
            "Current-only     : %9\n"
            "MAE              : %10 um\n"
            "RMSE             : %11 um\n"
            "P95 abs error    : %12 um\n"
            "Max abs error    : %13 um\n"
            "Max relative err : %14%")
            .arg(predictionModeName(comparisonMode))
            .arg(static_cast<qulonglong>(comparisonCount))
            .arg(nonzeroOnly ? QStringLiteral("current nonzero") : QStringLiteral("all vertices"))
            .arg(static_cast<qulonglong>(current.results.size() - comparisonCount))
            .arg(static_cast<qulonglong>(referenceActive))
            .arg(static_cast<qulonglong>(currentActive))
            .arg(static_cast<qulonglong>(commonActive))
            .arg(static_cast<qulonglong>(missingReference))
            .arg(static_cast<qulonglong>(currentOnly))
            .arg(meanAbsoluteError * kMetersToMicrometers, 0, 'g', 6)
            .arg(rootMeanSquareError * kMetersToMicrometers, 0, 'g', 6)
            .arg(p95AbsoluteError * kMetersToMicrometers, 0, 'g', 6)
            .arg(maximumAbsoluteError * kMetersToMicrometers, 0, 'g', 6)
            .arg(maximumRelativeError * 100.0, 0, 'g', 6);
        m_status = QStringLiteral("Validation completed for %1 vertices.")
            .arg(static_cast<qulonglong>(comparisonCount));
        try {
            smrobot::visualization::SurfaceScalarOverlay errorOverlay =
                PaintingAnalysisMeshAdapter::makeRelativeErrorOverlay(
                    m_session.objectId.toStdString(),
                    m_session.binding,
                    m_referenceThickness,
                    current,
                    &comparisonMask);
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                QString applyError;
                const bool errorOverlayApplied = m_session.predictionDisplayModel
                    ? services->applySurfaceScalarOverlayModel(
                        errorOverlay, *m_session.predictionDisplayModel, &applyError)
                    : services->applySurfaceScalarOverlay(errorOverlay, &applyError);
                if(!errorOverlayApplied) {
                    m_status = applyError.isEmpty()
                        ? QStringLiteral("Validation completed, but the error cloud could not be displayed.")
                        : applyError;
                } else {
                    m_session.overlay = std::move(errorOverlay);
                    m_session.showRelativeError = true;
                    m_session.showThickness = true;
                    services->setCoatingModelVisible(m_session.objectId, true);
                    services->setSurfaceScalarOverlayVisible(m_session.objectId, true);
                }
            }
        } catch(const std::exception& exception) {
            m_status = QString::fromLocal8Bit(exception.what());
        }
        refreshViewModel();
        emit statusMessageRequested(m_status, 5000);
    }

    bool CoatingAnalysisModuleController::ensurePreviewWorkpieceLoaded()
    {
        if(!m_rotationPreviewWorkpiece.empty()) {
            return true;
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        if(object == nullptr || object->objectType != "workpiece") {
            return false;
        }
        const std::filesystem::path sourcePath =
            simulation_project::AssetResolver::resolveProjectPath(
                makeResolveContext(m_context.projectSession()),
                object->sourcePath);
        std::string loadError;
        const std::shared_ptr<assetcore::ModelDesc> model =
            assetcore::AssetManager::instance().tryLoadModel(
                sourcePath.generic_u8string(),
                static_cast<float>(object->visualScale),
                &loadError);
        if(!model) {
            return false;
        }
        const PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
            *model,
            object->name,
            sourcePath.generic_u8string(),
            makeTransform(object->transform));
        if(mesh.workpiece.empty()) {
            return false;
        }
        m_rotationPreviewWorkpiece = mesh.workpiece;
        return true;
    }

    bool CoatingAnalysisModuleController::hasEffectiveRotationAxis() const
    {
        return m_hasRotationAxis ||
            (!m_rotationPreviewWorkpiece.empty() && !m_session.objectId.isEmpty());
    }

    Eigen::Vector3d CoatingAnalysisModuleController::effectiveRotationAxisOrigin() const
    {
        if(m_hasRotationAxis) {
            return m_rotationAxisOrigin;
        }
        if(m_rotationPreviewWorkpiece.empty()) {
            return Eigen::Vector3d::Zero();
        }
        Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::max());
        Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::lowest());
        for(const auto& sample : m_rotationPreviewWorkpiece.samples) {
            minimum = minimum.cwiseMin(sample.position);
            maximum = maximum.cwiseMax(sample.position);
        }
        return (minimum + maximum) * 0.5;
    }

    Eigen::Vector3d CoatingAnalysisModuleController::effectiveRotationAxisDirection() const
    {
        return m_hasRotationAxis
            ? m_rotationAxisDirection.normalized()
            : m_panel.periodicAxisDirection();
    }

    QString CoatingAnalysisModuleController::rotationAxisSource() const
    {
        if(m_hasRotationAxis) {
            return QStringLiteral("Fitted from cylindrical surface");
        }
        if(!m_rotationPreviewWorkpiece.empty() && !m_session.objectId.isEmpty()) {
            return QStringLiteral("Manual axis");
        }
        return QStringLiteral("Not configured");
    }

    void CoatingAnalysisModuleController::applyLocalDebugVisibility()
    {
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            CoatingPredictionDebugVisibility visibility;
            visibility.cylindricalSurface = m_showCylindricalSurface;
            visibility.rotationAxis = m_showRotationAxis;
            visibility.localSector = m_showLocalSector;
            visibility.sprayPoints = m_showLocalSprayPoints;
            services->setCoatingPredictionDebugVisibility(visibility);
        }
    }

    CoatingAnalysisModuleController::~CoatingAnalysisModuleController() = default;

    const std::vector<spraytrajectory::SprayPathPoint>&
    CoatingAnalysisModuleController::waypoints() const
    {
        return m_session.waypoints;
    }

    const spraytrajectory::SprayPathPoint&
    CoatingAnalysisModuleController::waypointAt(std::size_t index) const
    {
        return m_session.waypoints[index];
    }

    double CoatingAnalysisModuleController::waypointDurationAt(std::size_t index) const
    {
        if(index + 1 < m_session.waypoints.size()) {
            return std::max(0.0,
                m_session.waypoints[index + 1].time - m_session.waypoints[index].time);
        }
        return -1.0;
    }

    void CoatingAnalysisModuleController::setThicknessDisplayRange(
        double minimumMicrometers,
        double maximumMicrometers)
    {
        if(anyPredictionRunning() || !m_session.hasResult
            || m_session.showRelativeError
            || !std::isfinite(minimumMicrometers)
            || !std::isfinite(maximumMicrometers)
            || minimumMicrometers < 0.0
            || minimumMicrometers >= maximumMicrometers) {
            return;
        }

        m_session.manualThicknessRange = true;
        m_session.minimumDisplayThicknessMeters =
            minimumMicrometers / kMetersToMicrometers;
        m_session.maximumDisplayThicknessMeters =
            maximumMicrometers / kMetersToMicrometers;
        updateThicknessRangeAndStatistics();

        QString error;
        if(!applyCurrentSurfaceOverlay(&error)) {
            m_status = error.isEmpty()
                ? QStringLiteral("Failed to apply the thickness display range.")
                : error;
        } else {
            m_status = QStringLiteral("Thickness display range updated.");
        }
        refreshViewModel();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::resetThicknessDisplayRange()
    {
        if(anyPredictionRunning() || !m_session.hasResult
            || m_session.showRelativeError) {
            return;
        }

        m_session.manualThicknessRange = false;
        updateThicknessRangeAndStatistics();

        QString error;
        if(!applyCurrentSurfaceOverlay(&error)) {
            m_status = error.isEmpty()
                ? QStringLiteral("Failed to restore the automatic thickness range.")
                : error;
        } else {
            m_status = QStringLiteral("Automatic thickness display range restored.");
        }
        refreshViewModel();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::activate()
    {
        m_active = true;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setCoatingAnalysisView(true);
        }
        ensureWorkpieceSelection();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            if(!m_session.objectId.isEmpty()) {
                services->focusCoatingObject(m_session.objectId, 0.3);
            }
        }
        applyVisualizationState();
        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                if(!services->setSurfaceScalarOverlayVisible(m_session.objectId, true)) {
                    QString error;
                    const bool applied = applyCurrentSurfaceOverlay(&error);
                    if(!applied) {
                        m_status = error;
                        m_session.clearResult();
                        refreshViewModel();
                        publishStateChanged();
                        return;
                    }
                }
                services->setSurfaceScalarProbeEnabled(
                    m_session.showThickness && m_session.thicknessPickEnabled,
                    m_session.objectId);
            }
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::deactivate()
    {
        if(simulationActive() && !anyPredictionRunning()) {
            exitSimulation();
        }
        m_active = false;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        clearModelVisibilityOverrides();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setCoatingAnalysisView(false);
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreviewVisible(false);
            if(m_session.hasResult) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        m_panel.setLanguageCode(m_languageCode);
        m_treePanel.setLanguageCode(m_languageCode);
        m_infoPanel.setLanguageCode(m_languageCode);
        m_visibilityBar.setLanguageCode(m_languageCode);
    }

    void CoatingAnalysisModuleController::populateViewportContextMenu(QMenu* menu)
    {
        if(menu == nullptr || !m_active) {
            return;
        }

        menu->addSeparator();
        QMenu* visualizationMenu = menu->addMenu(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Visualize")));

        QAction* modelAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Model")));
        modelAction->setCheckable(true);
        modelAction->setChecked(m_session.showModel);
        modelAction->setEnabled(!m_session.objectId.isEmpty());
        connect(modelAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowModel);

        QAction* trajectoryAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Trajectory")));
        trajectoryAction->setCheckable(true);
        trajectoryAction->setChecked(m_session.showTrajectory);
        trajectoryAction->setEnabled(!m_session.trajectory.empty());
        connect(trajectoryAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowTrajectory);

        QAction* sprayPointsAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Spray Points")));
        sprayPointsAction->setCheckable(true);
        sprayPointsAction->setChecked(m_session.showSprayPoints);
        sprayPointsAction->setEnabled(!m_session.trajectory.empty());
        connect(sprayPointsAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowSprayPoints);

        QAction* thicknessAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Thickness Cloud")));
        thicknessAction->setCheckable(true);
        thicknessAction->setChecked(m_session.showThickness);
        thicknessAction->setEnabled(m_session.hasResult);
        connect(thicknessAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowThickness);

        QAction* thicknessPickAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Thickness Pick")));
        thicknessPickAction->setCheckable(true);
        thicknessPickAction->setChecked(m_session.thicknessPickEnabled);
        thicknessPickAction->setEnabled(m_session.hasResult && m_session.showThickness);
        connect(thicknessPickAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setThicknessPickEnabled);
    }

    void CoatingAnalysisModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        if(event.kind == RobotQtViewerEventKind::UiLanguageChanged) {
            setLanguageCode(event.languageCode);
            refreshViewModel();
            emit statusMessageRequested(
                coatingAnalysisTranslate(m_languageCode, m_status), 3000);
            return;
        }
        if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            clearSession();
            ensureWorkpieceSelection();
        } else if(event.kind == RobotQtViewerEventKind::ViewportReloaded &&
            event.viewport.reloadSucceeded) {
            // The reload rebuilt the scene; re-apply the coating view mode so
            // robots stay hidden while the coating analysis workbench is active.
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingAnalysisView(m_active);
            }
            applyVisualizationState();
            if(simulationActive() && !anyPredictionRunning()) {
                rebuildSimulation(false);
            }
            if(m_active && !m_session.objectId.isEmpty()) {
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->focusCoatingObject(m_session.objectId, 0.3);
                }
            }
            applyOverlayAfterReload();
        } else if(event.kind == RobotQtViewerEventKind::ProjectDocumentChanged) {
            if(!m_session.objectId.isEmpty() &&
                findObject(m_context.document(), m_session.objectId) == nullptr) {
                clearSession();
            }
            ensureWorkpieceSelection();
            refreshViewModel();
        } else if(event.kind == RobotQtViewerEventKind::SelectionChanged) {
            selectWorkpieceFromTree(event.selection.objectId);
        }
    }

    void CoatingAnalysisModuleController::handleSurfaceScalarHover(
        const QString& objectId,
        double valueMeters,
        double worldX,
        double worldY,
        double worldZ,
        const QPoint& viewportPosition,
        bool hit)
    {
        if(!m_active || !m_session.hasResult || !m_session.showThickness ||
            !m_session.thicknessPickEnabled ||
            objectId != m_session.objectId || !hit) {
            if(!m_hasCurrentThickness) {
                return;
            }
            m_hasCurrentThickness = false;
            emit thicknessToolTipRequested(QString(), viewportPosition, false);
            refreshViewModel();
            return;
        }

        m_hasCurrentThickness = true;
        m_currentThicknessMeters = valueMeters;
        const QString text = m_session.showRelativeError
            ? QStringLiteral(
                "Vertex\nX: %1 mm\nY: %2 mm\nZ: %3 mm\nRelative error: %4%")
                .arg(worldX * 1000.0, 0, 'f', 3)
                .arg(worldY * 1000.0, 0, 'f', 3)
                .arg(worldZ * 1000.0, 0, 'f', 3)
                .arg(valueMeters, 0, 'f', 2)
            : QStringLiteral(
                "Vertex\nX: %1 mm\nY: %2 mm\nZ: %3 mm\nThickness: %4 um")
                .arg(worldX * 1000.0, 0, 'f', 3)
                .arg(worldY * 1000.0, 0, 'f', 3)
                .arg(worldZ * 1000.0, 0, 'f', 3)
                .arg(valueMeters * kMetersToMicrometers, 0, 'f', 2);
        emit thicknessToolTipRequested(text, viewportPosition, true);
        refreshViewModel();
    }

    bool CoatingAnalysisModuleController::loadModel(
        const QString& path,
        double scaleToMeters)
    {
        const std::filesystem::path sourcePath = normalizedPath(
            std::filesystem::path(path.toStdWString()));
        if(!std::filesystem::exists(sourcePath)) {
            m_status = QStringLiteral("Model file was not found: %1").arg(path);
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return false;
        }
        const double modelScale = scaleToMeters > 0.0 ? scaleToMeters : 1.0;

        for(const simulation_project::SceneObjectDesc& existing : m_context.document().objects) {
            if(existing.objectType != "workpiece" || existing.visualScale != modelScale) {
                continue;
            }
            const std::filesystem::path existingPath = normalizedPath(
                simulation_project::AssetResolver::resolveProjectPath(
                    makeResolveContext(m_context.projectSession()),
                    existing.sourcePath));
            if(existingPath == sourcePath) {
                const QString existingId = QString::fromStdString(existing.id);
                if(existingId == m_session.objectId) {
                    m_session.manualThicknessRange = false;
                    m_session.minimumDisplayThicknessMeters = 0.0;
                    m_session.maximumDisplayThicknessMeters = 0.0;
                    if(m_session.hasResult) {
                        updateThicknessRangeAndStatistics();
                        if(!m_session.showRelativeError) {
                            QString applyError;
                            applyCurrentSurfaceOverlay(&applyError);
                        }
                    }
                }
                selectWorkpiece(existingId);
                m_status = QStringLiteral("Existing model reused. Run thickness prediction.");
                refreshViewModel();
                emit statusMessageRequested(m_status, 3000);
                return true;
            }
        }

        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            clearModelVisibilityOverrides();
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_session.clear();
        resetReferenceResult();
        m_hasCurrentThickness = false;
        publishStateChanged();

        SceneEntityWorkflowController importWorkflow(m_context);
        const SceneEntityImportResult importResult = importWorkflow.importSceneObjectFromPath(
            sourcePath,
            "workpiece",
            modelScale);
        if(!importResult.success) {
            m_status = importResult.message;
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return false;
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
            return false;
        }

        m_session.clear();
        resetReferenceResult();
        m_hasRotationAxis = false;
        m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
        m_rotationSurfaceTriangleIndices.clear();
        m_rotationSeedTriangleIndex = 0;
        m_rotationFitElapsedMilliseconds = 0.0;
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        clearAxisymmetricProfileSelection();
        m_panel.setPeriodicLocalPredictionEnabled(false);
        m_session.objectId = importResult.entityId;
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        m_session.modelName = object != nullptr
            ? QString::fromStdString(object->name)
            : QString::fromWCharArray(sourcePath.stem().c_str());
        m_session.sourcePath = importResult.storedPath;
        const std::filesystem::path resolvedSourcePath =
            simulation_project::AssetResolver::resolveProjectPath(
                makeResolveContext(m_context.projectSession()),
                object != nullptr ? object->sourcePath : importResult.storedPath.toStdString());
        std::string modelLoadError;
        const std::shared_ptr<assetcore::ModelDesc> model =
            assetcore::AssetManager::instance().tryLoadModel(
                resolvedSourcePath.generic_u8string(),
                object != nullptr ? static_cast<float>(object->visualScale) : 1.0f,
                &modelLoadError);
        if(model) {
            m_session.modelInfo = makeModelInfo(
                *model,
                object != nullptr
                    ? makeTransform(object->transform)
                    : Eigen::Isometry3d::Identity());
        }
        m_context.selectionModel().selectSceneObject(
            m_session.objectId,
            QStringLiteral("coatingAnalysisOpenModel"));
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            if(m_active) {
                services->selectSceneObject(QString());
            } else {
                services->selectSceneObject(m_session.objectId);
            }
        }
        applyVisualizationState();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            if(m_active) {
                services->focusCoatingObject(m_session.objectId, 0.3);
            }
        }
        m_status = QStringLiteral("Model loaded. Load a trajectory and run thickness prediction.");
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 3000);
        return true;
    }

    void CoatingAnalysisModuleController::openModelFromDialog()
    {
        // This analysis workflow uses the fixed benchmark STL requested for
        // the current thickness-prediction validation run. The file is in mm.
        loadModel(kFixedModelPath, 0.001);
    }

    bool CoatingAnalysisModuleController::loadTrajectory(const QString& path)
    {
        const std::filesystem::path sourcePath(path.toStdWString());
        if(!std::filesystem::exists(sourcePath)) {
            m_status = QStringLiteral("Spray trajectory was not found: %1").arg(path);
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return false;
        }

        const spraytrajectory::SprayTrajectoryLoadResult loadResult =
            spraytrajectory::SprayTrajectoryIo::loadText(sourcePath);
        if(!loadResult.success) {
            m_status = loadResult.warnings.empty()
                ? QStringLiteral("Failed to load the spray trajectory.")
                : QString::fromStdString(loadResult.warnings.front());
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return false;
        }

        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearCoatingPredictionDebugState();
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.clearResult();
        resetReferenceResult();
        m_session.trajectory = loadResult.trajectory;
        m_session.waypoints = loadResult.trajectory.flattenedPoints();
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        m_session.trajectoryName = QString::fromStdString(loadResult.trajectory.name);
        m_session.trajectoryPath = path;
        m_session.trajectoryInfo = makeTrajectoryInfo(
            loadResult.trajectory,
            loadResult.warnings.size());
        m_treePanel.setWaypoints(&m_session.waypoints);
        submitTrajectoryPreview();
        if(m_panel.periodicLocalPredictionEnabled() && hasEffectiveRotationAxis()) {
            applyModelVisibilityOverrides();
            previewLocalInputs();
        }
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_status = QStringLiteral("Trajectory loaded. Ready for GPU thickness prediction.");
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 3000);
        return true;
    }

    void CoatingAnalysisModuleController::openTrajectoryFromDialog()
    {
        loadTrajectory(kFixedTrajectoryPath);
    }

    void CoatingAnalysisModuleController::selectModelFileFromDialog()
    {
        QString title = QStringLiteral("Open Coating Analysis Model");
        QString filter = QStringLiteral(
            "Mesh Models (*.stl *.obj *.dae *.ply);;All Files (*.*)");
        if(reproductionActive()) {
            const PublishedReproductionInputProfile profile =
                PublishedReproductionAdapter::inputProfile(
                    m_panel.reproductionAlgorithm());
            title = coatingAnalysisTranslate(m_languageCode,
                QString::fromStdString(profile.modelDialogTitle));
            filter = QString::fromStdString(profile.modelDialogFilter);
        }
        const QString path = PaintingAnalysisDialogService::selectModelFile(
            &m_panel, title, filter);
        if(path.isEmpty()) {
            return;
        }

        double scaleToMeters = 0.001;
        if(!PaintingAnalysisDialogService::selectModelUnitScale(
               &m_panel,
               path,
               scaleToMeters)) {
            return;
        }
        loadModel(path, scaleToMeters);
    }

    void CoatingAnalysisModuleController::selectTrajectoryFileFromDialog()
    {
        QString title = QStringLiteral("Open Spray Trajectory");
        if(reproductionActive()) {
            const PublishedReproductionInputProfile profile =
                PublishedReproductionAdapter::inputProfile(
                    m_panel.reproductionAlgorithm());
            title = coatingAnalysisTranslate(m_languageCode,
                QString::fromStdString(profile.trajectoryDialogTitle));
        }
        const QString path = PaintingAnalysisDialogService::selectTrajectoryFile(
            &m_panel, title);
        if(path.isEmpty()) {
            return;
        }
        loadTrajectory(path);
    }

    void CoatingAnalysisModuleController::predictThickness()
    {
        if(anyPredictionRunning()) {
            return;
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(object == nullptr || services == nullptr || m_session.trajectory.empty()) {
            m_status = QStringLiteral("The analysis model, trajectory, or viewport is unavailable.");
            refreshViewModel();
            return;
        }
        if(m_panel.rotationBasedPredictionEnabled() && !hasEffectiveRotationAxis()) {
            m_status = QStringLiteral(
                "Local prediction requires a fitted or manually selected rotation axis.");
            refreshViewModel();
            return;
        }
        if(m_panel.axisymmetricProfilePredictionEnabled()
            && !m_axisymmetricProfile->reduction.valid()) {
            m_status = QStringLiteral(
                "Axisymmetric profile prediction requires a selected profile region.");
            refreshViewModel();
            return;
        }
        if(m_panel.adaptiveMeshPredictionEnabled()
            && (!m_axisymmetricProfile->selection.enabled || !hasEffectiveRotationAxis())) {
            m_status = QStringLiteral(
                "Adaptive mesh prediction requires a fitted axis and a selected dense region.");
            refreshViewModel();
            return;
        }
        if(m_panel.localCandidateVertexPredictionEnabled()
            && (!m_axisymmetricProfile->selection.enabled || !hasEffectiveRotationAxis())) {
            m_status = QStringLiteral(
                "Local candidate prediction requires a fitted axis and a selected prediction region.");
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
            const Eigen::Isometry3d worldFromModel = makeTransform(object->transform);
            PaintingAnalysisMeshData mesh;
            const bool localCandidateMode =
                m_panel.localCandidateVertexPredictionEnabled();
            std::vector<std::uint32_t> restrictedPredictionVertices;
            if(m_panel.adaptiveMeshPredictionEnabled()) {
                AdaptiveMeshOptions adaptiveOptions;
                adaptiveOptions.axisOrigin = effectiveRotationAxisOrigin();
                adaptiveOptions.axisDirection = effectiveRotationAxisDirection();
                adaptiveOptions.selectionMinimum = m_axisymmetricProfile->selection.minimum;
                adaptiveOptions.selectionMaximum = m_axisymmetricProfile->selection.maximum;
                adaptiveOptions.selectionPolygon = m_axisymmetricProfile->selection.polygon;
                adaptiveOptions.simplificationPercent = m_panel.adaptiveMeshSimplificationPercent();
                mesh = PaintingAnalysisMeshAdapter::buildAdaptive(
                    *model, object->name, sourcePath.generic_u8string(), worldFromModel, adaptiveOptions);
            } else {
                mesh = PaintingAnalysisMeshAdapter::build(
                    *model, object->name, sourcePath.generic_u8string(), worldFromModel);
            }
            if(localCandidateMode) {
                AdaptiveMeshOptions regionOptions;
                regionOptions.axisOrigin = effectiveRotationAxisOrigin();
                regionOptions.axisDirection = effectiveRotationAxisDirection();
                regionOptions.selectionMinimum = m_axisymmetricProfile->selection.minimum;
                regionOptions.selectionMaximum = m_axisymmetricProfile->selection.maximum;
                regionOptions.selectionPolygon = m_axisymmetricProfile->selection.polygon;
                restrictedPredictionVertices =
                    PaintingAnalysisMeshAdapter::selectVerticesInRegion(
                        mesh.workpiece, regionOptions);
                if(restrictedPredictionVertices.empty()) {
                    throw std::runtime_error(
                        "The selected prediction region contains no model vertices.");
                }
                mesh.warnings.push_back(
                    "Local candidate prediction vertices: "
                    + std::to_string(restrictedPredictionVertices.size()) + "/"
                    + std::to_string(mesh.workpiece.samples.size()));
            }
            QString adaptiveMeshTiming;
            QString adaptiveMeshSummary;
            QString adaptiveMeshTopologyWarning;
            bool adaptiveMeshCacheHit = false;
            for(const std::string& warning : mesh.warnings) {
                LOG_DEBUG("rs2026") << "Painting analysis mesh: " << warning;
                if(m_panel.adaptiveMeshPredictionEnabled()
                    && (warning == "Adaptive mesh cache hit; simplification was skipped."
                        || warning == "Adaptive mesh disk cache hit; simplification was skipped.")) {
                    adaptiveMeshCacheHit = true;
                }
                if(m_panel.adaptiveMeshPredictionEnabled()
                    && warning.rfind("Adaptive mesh total:", 0) == 0) {
                    adaptiveMeshTiming = QString::fromStdString(warning);
                }
                if(m_panel.adaptiveMeshPredictionEnabled()
                    && warning.rfind("Adaptive mesh QEM:", 0) == 0) {
                    adaptiveMeshSummary = QString::fromStdString(warning);
                }
                if(m_panel.adaptiveMeshPredictionEnabled()
                    && warning.rfind("Adaptive mesh QEM output failed topology validation", 0) == 0) {
                    adaptiveMeshTopologyWarning = QString::fromStdString(warning);
                }
            }
            if(mesh.workpiece.empty()) {
                m_status = QStringLiteral("The model has no mesh vertices.");
                refreshViewModel();
                return;
            }

            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            m_session.clearResult();
            m_session.binding = std::move(mesh.binding);
            m_session.predictionDisplayModel = std::move(mesh.displayModel);
            if(m_panel.adaptiveMeshPredictionEnabled()) {
                QString displayError;
                if(!m_session.predictionDisplayModel
                    || !services->setCoatingPredictionModel(
                        QString::fromStdString(object->id),
                        *m_session.predictionDisplayModel,
                        &displayError)) {
                    throw std::runtime_error(
                        displayError.isEmpty()
                        ? "Failed to display the adaptive prediction mesh."
                        : displayError.toStdString());
                }
                std::size_t sourceVertexCount = 0;
                std::size_t displayVertexCount = 0;
                std::size_t displayTriangleCount = mesh.workpiece.triangleIndices.size() / 3;
                for(const auto& subMesh : model->subMeshes()) {
                    sourceVertexCount += subMesh.geometry.positions.size();
                }
                for(const auto& subMesh : m_session.predictionDisplayModel->subMeshes()) {
                    displayVertexCount += subMesh.geometry.positions.size();
                }
                m_status = QStringLiteral(
                    "Adaptive mesh ready: vertices %1 -> %2, triangles %3, dense region retained.")
                    .arg(static_cast<qulonglong>(sourceVertexCount))
                    .arg(static_cast<qulonglong>(displayVertexCount))
                    .arg(static_cast<qulonglong>(displayTriangleCount));
                if(!adaptiveMeshTiming.isEmpty()) {
                    m_status += QStringLiteral("\n") + adaptiveMeshTiming;
                }
                if(!adaptiveMeshSummary.isEmpty()) {
                    m_status += QStringLiteral("\n") + adaptiveMeshSummary;
                }
                if(!adaptiveMeshTopologyWarning.isEmpty()) {
                    m_status += QStringLiteral("\n") + adaptiveMeshTopologyWarning;
                }
                if(adaptiveMeshCacheHit) {
                    m_status += QStringLiteral("\nAdaptive mesh cache hit; simplification skipped.");
                }
            } else if(localCandidateMode) {
                m_status = QStringLiteral(
                    "Local candidate mode ready: %1/%2 vertices selected; complete-model BVH enabled.")
                    .arg(static_cast<qulonglong>(restrictedPredictionVertices.size()))
                    .arg(static_cast<qulonglong>(mesh.workpiece.samples.size()));
            }
            m_hasCurrentThickness = false;

            spraythickness::ThicknessPredictionTask task;
            task.model = m_panel.thicknessModel();
            task.workpiece = std::move(mesh.workpiece);
            task.trajectory = m_session.trajectory;
            task.tool.name = "Legacy spray gun";
            task.tool.sprayDirectionLocal = m_panel.sprayDirectionLocal();
            task.tool.powderFeedDirectionLocal = m_panel.powderFeedDirectionLocal();
            task.process.id = spraythickness::thicknessModelId(task.model);
            task.process.name = task.process.id;
            task.options.base.trajectorySamplingMode = m_panel.trajectorySamplingMode();
            task.options.base.timeStep = m_panel.timeStepSeconds();
            task.options.enableBvhOcclusion = m_panel.bvhOcclusionEnabled();
            task.options.enableHistoryCorrection = m_panel.historyCorrectionEnabled();
            task.options.periodicLocal.enabled = m_panel.periodicLocalPredictionEnabled();
            task.options.axisymmetricProfile.enabled =
                m_panel.axisymmetricProfilePredictionEnabled();
            task.options.spatialFiltering.enabled =
                m_panel.spatialInfluenceFilteringEnabled();
            task.options.spatialFiltering.filterCandidateVertices =
                m_panel.spatialCandidateVertexFilteringEnabled();
            task.options.spatialFiltering.overrideGridCellSize =
                m_panel.overrideSpatialGridCellSize();
            task.options.spatialFiltering.gridCellSizeMeters =
                task.options.spatialFiltering.overrideGridCellSize
                ? m_panel.spatialGridCellSizeMillimeters() * 1.0e-3
                : 0.0;
            if(localCandidateMode) {
                task.options.spatialFiltering.predictionVertexIndices =
                    std::move(restrictedPredictionVertices);
                task.options.enableBvhOcclusion = true;
                task.options.spatialFiltering.fallbackToFullPrediction = false;
            }
            if(m_panel.adaptiveMeshPredictionEnabled()) {
                task.options.spatialFiltering.enabled = true;
                task.options.spatialFiltering.filterCandidateVertices = true;
            }
            if(task.options.periodicLocal.enabled) {
                Eigen::Vector3d boundsMinimum = Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::max());
                Eigen::Vector3d boundsMaximum = Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::lowest());
                for(const auto& sample : task.workpiece.samples) {
                    boundsMinimum = boundsMinimum.cwiseMin(sample.position);
                    boundsMaximum = boundsMaximum.cwiseMax(sample.position);
                }
                const Eigen::Vector3d axis = effectiveRotationAxisDirection();
                task.options.periodicLocal.axisOrigin = m_hasRotationAxis
                    ? m_rotationAxisOrigin
                    : (boundsMinimum + boundsMaximum) * 0.5;
                task.options.periodicLocal.axisDirection = axis.normalized();
                task.options.periodicLocal.referenceDirection = std::abs(axis.x()) < 0.9
                    ? Eigen::Vector3d::UnitX()
                    : Eigen::Vector3d::UnitY();
                task.options.periodicLocal.sectorCount = m_panel.periodicSectorCount();
                task.options.periodicLocal.angularHaloRadians = 0.0;
                task.options.periodicLocal.reduceTrajectory = false;
                task.options.periodicLocal.fallbackToFullPrediction = false;
            }
            if(task.options.axisymmetricProfile.enabled) {
                task.options.axisymmetricProfile.predictionSamples =
                    m_axisymmetricProfile->reduction.predictionSamples;
                task.options.axisymmetricProfile.sampleSegments =
                    m_axisymmetricProfile->reduction.sampleSegments;
                task.options.axisymmetricProfile.axisOrigin =
                    m_axisymmetricProfile->slice.axisOrigin;
                task.options.axisymmetricProfile.axisDirection =
                    m_axisymmetricProfile->slice.axisDirection;
                task.options.axisymmetricProfile.radialDirection =
                    m_axisymmetricProfile->slice.radialDirection;
                task.options.axisymmetricProfile.selectionMinimum =
                    m_axisymmetricProfile->selection.minimum;
                task.options.axisymmetricProfile.selectionMaximum =
                    m_axisymmetricProfile->selection.maximum;
                task.options.axisymmetricProfile.selectionPolygon =
                    m_axisymmetricProfile->selection.polygon;
            }

            m_predictionObjectId = QString::fromStdString(object->id);
            m_predictionProgress = 0.0;
            m_status = QStringLiteral("Preparing GPU thickness prediction...");
            if(!adaptiveMeshTiming.isEmpty()) {
                m_status += QStringLiteral("\n") + adaptiveMeshTiming;
            }
            m_session.predictionElapsedSeconds = 0.0;
            refreshViewModel();
            publishStateChanged();

            m_predictionStartedAt = std::chrono::steady_clock::now();
            m_predictionTimerActive = true;
            if(!m_predictionJob->start(std::move(task))) {
                m_predictionObjectId.clear();
                m_predictionProgress = 0.0;
                m_predictionTimerActive = false;
                m_session.clearResult();
                m_status = QStringLiteral("Failed to start the GPU thickness prediction task.");
                refreshViewModel();
                return;
            }
        } catch(const std::exception& exception) {
            m_predictionObjectId.clear();
            m_predictionProgress = 0.0;
            m_predictionTimerActive = false;
            services->clearSurfaceScalarOverlay(m_session.objectId);
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            refreshViewModel();
            publishStateChanged();
        }
    }

    bool CoatingAnalysisModuleController::simulationActive() const
    {
        return m_mode == CoatingAnalysisMode::Simulation;
    }

    bool CoatingAnalysisModuleController::reproductionActive() const
    {
        return m_mode == CoatingAnalysisMode::Reproduction;
    }

    bool CoatingAnalysisModuleController::anyPredictionRunning() const
    {
        return m_predictionJob->isRunning() || m_reproductionJob->isRunning();
    }

    void CoatingAnalysisModuleController::enterSimulation()
    {
        if(anyPredictionRunning()) {
            return;
        }
        m_savedSimulationSession = m_session;
        m_savedSimulationModelVisibility = m_modelVisibility;
        m_session.clear();
        m_session.objectId = QString::fromLatin1(kSimulationPlateObjectId);
        m_session.showModel = true;
        m_session.showTrajectory = true;
        m_session.showSprayPoints = true;
        m_mode = CoatingAnalysisMode::Simulation;
        m_simulationStatus = QStringLiteral("Simulation mode active.");
        rebuildSimulation(false, true);
    }

    void CoatingAnalysisModuleController::exitSimulation()
    {
        if(anyPredictionRunning() || !simulationActive()) {
            return;
        }
        const QString simulationObjectId =
            QString::fromLatin1(kSimulationPlateObjectId);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            // Remove every simulation-owned render resource before restoring
            // the saved prediction session. The grid is rebuilt from the
            // plate visibility state and therefore disappears here as well.
            services->clearSurfaceScalarOverlay(simulationObjectId);
            services->clearCoatingPredictionModel(simulationObjectId);
            services->setCoatingModelVisible(simulationObjectId, false);
            services->setCoatingTrajectoryPreview({}, false);
        }
        m_mode = CoatingAnalysisMode::Prediction;
        m_simulationReady = false;
        m_simulation = SimulationExperimentData();
        m_simulationReadyStatus.clear();
        m_simulationStatus.clear();
        m_session = std::move(m_savedSimulationSession);
        m_modelVisibility = std::move(m_savedSimulationModelVisibility);
        m_status = QStringLiteral("Exited spray simulation mode.");
        // The workbench remains active while switching tabs. Re-assert the
        // analysis view and restore the prediction object's camera target so
        // camera/projection/rotation interaction does not depend on the tab
        // that was active immediately before the switch.
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setCoatingAnalysisView(true);
            if(!m_session.objectId.isEmpty()) {
                services->focusCoatingObject(m_session.objectId, 0.3);
            }
        }
        applyModelVisibilityOverrides();
        updateTrajectoryPreviewVisibility();
        applyOverlayAfterReload();
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::rebuildSimulation(bool runAfterBuild, bool focusView)
    {
        if(!simulationActive() || anyPredictionRunning()) {
            return;
        }
        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setSurfaceScalarProbeEnabled(false, QString());
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            m_session.clearResult();
            m_hasCurrentThickness = false;
        }
        QString error;
        SimulationExperimentData experiment;
        if(!SimulationExperiment::build(m_panel.simulationParameters(), experiment, &error)) {
            m_simulationReady = false;
            m_simulationStatus = error;
            m_status = m_simulationStatus;
            refreshViewModel();
            return;
        }

        // Publish the rebuilt simulation data before refreshing any view state.
        // The preview, session and prediction task must all observe the same
        // trajectory after an incidence/azimuth change.
        m_simulation = std::move(experiment);
        m_simulationReady = true;
        m_session.modelName = QStringLiteral("Simulation plate");
        m_session.sourcePath = QStringLiteral("simulation://plate");
        m_session.trajectoryName = QString::fromStdString(m_simulation.trajectory.name);
        m_session.trajectoryPath = QStringLiteral("simulation://generated");
        m_session.trajectoryInfo = makeTrajectoryInfo(m_simulation.trajectory, 0);
        m_session.trajectory = m_simulation.trajectory;
        m_session.waypoints = m_simulation.trajectory.flattenedPoints();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->clearSurfaceScalarOverlay(m_session.objectId);
            services->clearCoatingPredictionModel(m_session.objectId);
            QString displayError;
            if(!services->setCoatingPredictionModel(
                    m_session.objectId, *m_simulation.displayModel, &displayError)) {
                m_simulationReady = false;
                m_simulationStatus = displayError;
                m_status = m_simulationStatus;
                refreshViewModel();
                return;
            }
            if(focusView) {
                services->focusCoatingObject(m_session.objectId, 0.3);
            }
            QHash<QString, bool> simulationVisibility;
            for(const simulation_project::SceneObjectDesc& object : m_context.document().objects) {
                if(object.objectType == "workpiece") {
                    simulationVisibility.insert(QString::fromStdString(object.id), false);
                }
            }
            services->setCoatingModelVisibilities(simulationVisibility);
            std::vector<CoatingTrajectoryPreviewPoint> preview;
            for(const auto& segment : m_simulation.trajectory.segments) {
                bool first = true;
                for(const auto& point : segment.points) {
                    CoatingTrajectoryPreviewPoint item;
                    const Eigen::Vector3d position = point.tcpPose.translation();
                    const Eigen::Matrix3d& rotation = point.tcpPose.linear();
                    const Eigen::Vector3d direction = rotation * Eigen::Vector3d::UnitZ();
                    item.positionX = position.x(); item.positionY = position.y(); item.positionZ = position.z();
                    item.directionX = direction.x(); item.directionY = direction.y(); item.directionZ = direction.z();
                    const Eigen::Vector3d frameX = rotation.col(0);
                    const Eigen::Vector3d frameY = rotation.col(1);
                    const Eigen::Vector3d frameZ = rotation.col(2);
                    item.frameXAxisX = frameX.x(); item.frameXAxisY = frameX.y(); item.frameXAxisZ = frameX.z();
                    item.frameYAxisX = frameY.x(); item.frameYAxisY = frameY.y(); item.frameYAxisZ = frameY.z();
                    item.frameZAxisX = frameZ.x(); item.frameZAxisY = frameZ.y(); item.frameZAxisZ = frameZ.z();
                    item.sprayEnabled = point.sprayEnabled;
                    item.startsNewSegment = first;
                    first = false;
                    preview.push_back(item);
                }
            }
            services->setCoatingTrajectoryPreview(
                preview,
                m_session.showTrajectory || m_session.showSprayPoints,
                m_session.showTrajectory,
                m_session.showSprayPoints);
        }
        m_session.modelInfo = makeModelInfo(*m_simulation.displayModel,
            Eigen::Isometry3d::Identity());
        m_simulationReadyStatus = QStringLiteral("Plate ready: %1 x %2 cells, actual cell %3 mm\n"
            "Trajectory points: %4, interval: %5 s\n"
            "Side: %6 mm\nDistance: %7 mm\nIncidence: %8 deg\nAzimuth: %9 deg\n"
            "Tool roll: %10 deg")
            .arg(static_cast<qulonglong>(m_simulation.rowCount))
            .arg(static_cast<qulonglong>(m_simulation.columnCount))
            .arg(m_simulation.actualCellSizeMeters * 1000.0, 0, 'f', 3)
            .arg(static_cast<qulonglong>(m_simulation.trajectory.flattenedPoints().size()))
            .arg(m_simulation.parameters.trajectoryPointIntervalSeconds, 0, 'f', 3)
            .arg(m_simulation.parameters.plateSideMillimeters, 0, 'f', 2)
            .arg(m_simulation.parameters.sprayDistanceMillimeters, 0, 'f', 2)
            .arg(m_simulation.parameters.incidenceAngleDegrees, 0, 'f', 1)
            .arg(m_simulation.parameters.azimuthDegrees, 0, 'f', 1)
            .arg(m_simulation.parameters.toolRollDegrees, 0, 'f', 1);
        if(m_simulation.parameters.kind == SimulationExperimentKind::LineScan) {
            m_simulationReadyStatus += QStringLiteral(
                "\nScan passes (round trips): %1")
                .arg(m_simulation.parameters.scanPassCount);
        }
        m_simulationStatus = m_simulationReadyStatus;
        m_status = m_simulationStatus;
        refreshViewModel();
        publishStateChanged();
        if(runAfterBuild) {
            runSimulationPrediction();
        }
    }

    void CoatingAnalysisModuleController::runSimulationPrediction()
    {
        QString runError;
        const std::size_t simulationVertexCount = m_simulation.workpiece.samples.size();
        const std::size_t simulationTriangleIndexCount =
            m_simulation.workpiece.triangleIndices.size();
        const std::size_t activeSprayIntervals = countActiveSprayIntervals(
            m_simulation.trajectory);
        if(!simulationActive()) {
            runError = QStringLiteral("Enter simulation mode before running it.");
        } else if(!m_simulationReady) {
            runError = QStringLiteral("Build the simulation plate before running it.");
        } else if(anyPredictionRunning()) {
            runError = QStringLiteral("A thickness prediction is already running.");
        } else if(m_simulation.trajectory.empty()) {
            runError = QStringLiteral(
                "The simulation trajectory is empty. Rebuild the plate.");
        } else if(simulationVertexCount == 0 || simulationTriangleIndexCount < 3) {
            runError = QStringLiteral(
                "The simulation plate has no valid vertices or triangles.");
        } else if(activeSprayIntervals == 0) {
            runError = QStringLiteral(
                "The simulation trajectory has no active spray interval.");
        }
        if(!runError.isEmpty()) {
            m_status = runError;
            m_simulationStatus = runError;
            refreshViewModel();
            publishStateChanged();
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearSurfaceScalarOverlay(m_session.objectId);
        }
        m_session.clearResult();
        m_simulation.thicknessVolumeCubicMillimeters = 0.0;
        m_session.predictionDisplayModel = m_simulation.displayModel;
        m_session.binding.sampleIndicesBySubMesh = { {} };
        m_session.binding.sampleIndicesBySubMesh.front().resize(
            m_simulation.workpiece.samples.size());
        std::iota(m_session.binding.sampleIndicesBySubMesh.front().begin(),
            m_session.binding.sampleIndicesBySubMesh.front().end(), std::size_t{ 0 });
        spraythickness::ThicknessPredictionTask task;
        task.model = m_panel.thicknessModel();
        task.workpiece = m_simulation.workpiece;
        task.trajectory = m_simulation.trajectory;
        task.tool.name = "Simulation spray gun";
        task.tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
        task.tool.powderFeedDirectionLocal = Eigen::Vector3d::UnitX();
        // Simulation points use the simulation segment's process ID.  The
        // GPU backend filters trajectory samples by this ID; using the
        // Gaussian model ID here would discard every simulation sample
        // before dispatch.
        task.process.id = "simulation";
        task.process.name = "simulation";
        task.options.base.trajectorySamplingMode =
            spraythickness::TrajectorySamplingMode::OriginalPoints;
        task.options.enableBvhOcclusion = m_panel.bvhOcclusionEnabled();
        task.options.enableHistoryCorrection = m_panel.historyCorrectionEnabled();
        m_predictionObjectId = m_session.objectId;
        m_predictionProgress = 0.0;
        m_status = QStringLiteral(
            "Running spray simulation with GPU deposition shader...\n"
            "Plate vertices: %1, triangle indices: %2, active spray intervals: %3")
            .arg(static_cast<qulonglong>(simulationVertexCount))
            .arg(static_cast<qulonglong>(simulationTriangleIndexCount))
            .arg(static_cast<qulonglong>(activeSprayIntervals));
        m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
        m_predictionStartedAt = std::chrono::steady_clock::now();
        m_predictionTimerActive = true;
        refreshViewModel();
        publishStateChanged();
        QString startError;
        if(!m_predictionJob->start(std::move(task), &startError)) {
            m_predictionObjectId.clear();
            m_predictionTimerActive = false;
            m_status = QStringLiteral("Failed to start spray simulation: %1")
                .arg(startError.isEmpty()
                    ? QStringLiteral("unknown worker error")
                    : startError);
            m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            refreshViewModel();
            publishStateChanged();
        }
    }

    void CoatingAnalysisModuleController::exportSimulationResult()
    {
        if(!simulationActive() || !m_session.hasResult) {
            return;
        }
        QSettings settings;
        const QString exportDirectory = settings.value(
            QString::fromLatin1(kSimulationExportDirectorySettingsKey)).toString();
        const double activeDuration = activeSprayDurationSeconds(m_simulation.trajectory);
        const double volume = m_simulation.thicknessVolumeCubicMillimeters;
        const QString fileName = simulationExportFileName(
            m_simulation.parameters, activeDuration, volume);
        const QString initialPath = exportDirectory.isEmpty()
            ? fileName
            : QDir(exportDirectory).filePath(fileName);
        const QString path = QFileDialog::getSaveFileName(
            &m_panel,
            QStringLiteral("Export simulation thickness"),
            initialPath,
            QStringLiteral("CSV files (*.csv)"));
        if(path.isEmpty()) {
            return;
        }
        QString error;
        if(!SimulationExperiment::exportContourCsv(
                path, m_simulation, m_session.prediction, &error)) {
            m_status = QStringLiteral("Simulation export failed: %1").arg(error);
        } else {
            settings.setValue(
                QString::fromLatin1(kSimulationExportDirectorySettingsKey),
                QFileInfo(path).absolutePath());
            m_status = QStringLiteral("Simulation thickness exported to %1.").arg(path);
        }
        refreshViewModel();
        emit statusMessageRequested(m_status, 5000);
    }

    void CoatingAnalysisModuleController::enterReproduction()
    {
        if(anyPredictionRunning()) {
            return;
        }
        m_mode = CoatingAnalysisMode::Reproduction;
        m_reproductionStatus = m_session.hasReproductionResult
            ? m_reproductionStatus
            : QStringLiteral("Select a published algorithm and run its reproduction.");
        m_status = QStringLiteral("Algorithm reproduction mode active.");
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::exitReproduction()
    {
        if(anyPredictionRunning() || !reproductionActive()) {
            return;
        }
        m_mode = CoatingAnalysisMode::Prediction;
        m_status = QStringLiteral("Exited algorithm reproduction mode.");
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::runAlgorithmReproduction()
    {
        if(!reproductionActive() || anyPredictionRunning()) {
            return;
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(object == nullptr || services == nullptr || m_session.trajectory.empty()) {
            m_status = QStringLiteral(
                "The analysis model, trajectory, or viewport is unavailable.");
            m_reproductionStatus = m_status;
            refreshViewModel();
            return;
        }

        const std::filesystem::path sourcePath =
            simulation_project::AssetResolver::resolveProjectPath(
                makeResolveContext(m_context.projectSession()), object->sourcePath);
        std::string loadError;
        const std::shared_ptr<assetcore::ModelDesc> model =
            assetcore::AssetManager::instance().tryLoadModel(
                sourcePath.generic_u8string(), static_cast<float>(object->visualScale),
                &loadError);
        if(!model) {
            m_status = QString::fromStdString(loadError.empty()
                ? "Failed to load the analysis mesh."
                : loadError);
            m_reproductionStatus = m_status;
            refreshViewModel();
            return;
        }

        try {
            PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
                *model, object->name, sourcePath.generic_u8string(),
                makeTransform(object->transform));
            if(mesh.workpiece.empty()) {
                throw std::runtime_error("The model has no mesh vertices.");
            }
            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            m_session.clearResult();
            m_session.binding = std::move(mesh.binding);
            m_session.predictionDisplayModel = std::move(mesh.displayModel);
            m_reproductionWorkpiece = mesh.workpiece;
            m_predictionObjectId = QString::fromStdString(object->id);
            m_predictionProgress = 0.0;
            m_predictionStartedAt = std::chrono::steady_clock::now();
            m_predictionTimerActive = true;

            const spraythickness::ReproductionAlgorithmKind algorithm =
                m_panel.reproductionAlgorithm();
            m_status = QStringLiteral("Running %1...")
                .arg(QString::fromLatin1(
                    spraythickness::reproductionAlgorithmName(algorithm)));
            m_reproductionStatus = m_status;
            refreshViewModel();
            publishStateChanged();

            QString startError;
            if(algorithm
                == spraythickness::ReproductionAlgorithmKind::CurrentMethod) {
                spraythickness::ThicknessPredictionTask task;
                task.model = m_panel.thicknessModel();
                task.workpiece = std::move(mesh.workpiece);
                task.trajectory = m_session.trajectory;
                task.tool.name = "Legacy spray gun";
                task.tool.sprayDirectionLocal = m_panel.sprayDirectionLocal();
                task.tool.powderFeedDirectionLocal =
                    m_panel.powderFeedDirectionLocal();
                task.process.id = spraythickness::thicknessModelId(task.model);
                task.process.name = task.process.id;
                task.options.base.trajectorySamplingMode =
                    m_panel.trajectorySamplingMode();
                task.options.base.timeStep = m_panel.timeStepSeconds();
                task.options.enableBvhOcclusion = m_panel.bvhOcclusionEnabled();
                task.options.enableHistoryCorrection =
                    m_panel.historyCorrectionEnabled();
                m_reproductionGpuRun = true;
                if(!m_predictionJob->start(std::move(task), &startError)) {
                    m_reproductionGpuRun = false;
                    throw std::runtime_error(startError.toStdString());
                }
            } else {
                spraycore::SprayTool tool;
                tool.name = "Legacy spray gun";
                tool.sprayDirectionLocal = m_panel.sprayDirectionLocal();
                tool.powderFeedDirectionLocal = m_panel.powderFeedDirectionLocal();
                PublishedReproductionRuntimeInputs runtimeInputs =
                    m_panel.reproductionRuntimeInputs();
                runtimeInputs.modelSourcePath = sourcePath;
                spraythickness::AlgorithmReproductionTask task =
                    PublishedReproductionAdapter::buildTask(
                        algorithm,
                        mesh.workpiece,
                        m_session.trajectory,
                        tool,
                        m_panel.trajectorySamplingMode(),
                        m_panel.timeStepSeconds(),
                        std::filesystem::path(
                            m_panel.reproductionConfigurationPath().toStdWString()),
                        runtimeInputs);
                if(!m_reproductionJob->start(std::move(task), &startError)) {
                    throw std::runtime_error(startError.toStdString());
                }
            }
        } catch(const std::exception& exception) {
            m_predictionObjectId.clear();
            m_predictionProgress = 0.0;
            m_predictionTimerActive = false;
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            m_reproductionStatus = m_status;
            refreshViewModel();
            publishStateChanged();
        }
    }

    void CoatingAnalysisModuleController::createReproductionTemplate()
    {
        const auto algorithm = m_panel.reproductionAlgorithm();
        if(algorithm == spraythickness::ReproductionAlgorithmKind::CurrentMethod
            || anyPredictionRunning()) {
            return;
        }
        QSettings settings;
        const QString directory = settings.value(
            QString::fromLatin1(kReproductionExportDirectorySettingsKey)).toString();
        const QString fileName = QString::fromLatin1(
            spraythickness::reproductionAlgorithmId(algorithm)) + QStringLiteral(".json");
        const QString initialPath = directory.isEmpty()
            ? fileName : QDir(directory).filePath(fileName);
        const QString path = QFileDialog::getSaveFileName(
            &m_panel, QStringLiteral("Create calibration template"),
            initialPath, QStringLiteral("JSON files (*.json)"));
        if(path.isEmpty()) {
            return;
        }
        try {
            PublishedReproductionAdapter::writeConfigurationTemplate(
                algorithm, std::filesystem::path(path.toStdWString()));
            m_panel.setReproductionConfigurationPath(path);
            settings.setValue(
                QString::fromLatin1(kReproductionExportDirectorySettingsKey),
                QFileInfo(path).absolutePath());
            m_status = QStringLiteral(
                "Calibration template created. Replace every null value before running.");
            m_reproductionStatus = m_status;
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
        } catch(const std::exception& exception) {
            m_status = QString::fromLocal8Bit(exception.what());
            m_reproductionStatus = m_status;
            refreshViewModel();
        }
    }

    void CoatingAnalysisModuleController::cancelAlgorithmReproduction()
    {
        if(!reproductionActive()) {
            return;
        }
        m_predictionJob->cancel();
        m_reproductionJob->cancel();
        m_status = QStringLiteral("Canceling algorithm reproduction...");
        m_reproductionStatus = m_status;
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handleReproductionProgress(
        double progress,
        const QString& message)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        m_predictionProgress = std::clamp(progress, 0.0, 1.0);
        m_status = message;
        m_reproductionStatus = message;
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handleReproductionFinished(
        const spraythickness::AlgorithmReproductionResult& result)
    {
        if(spraythickness::reproductionCanceled(result)) {
            m_predictionObjectId.clear();
            m_predictionProgress = 0.0;
            m_predictionTimerActive = false;
            m_session.clearResult();
            m_status = QStringLiteral("Algorithm reproduction canceled.");
            m_reproductionStatus = m_status;
            refreshViewModel();
            publishStateChanged();
            return;
        }
        PublishedReproductionDisplayData display;
        try {
            display = PublishedReproductionDisplayAdapter::build(
                result, m_reproductionWorkpiece, m_session.binding);
        } catch(const std::exception& exception) {
            handleReproductionFailed(QStringLiteral(
                "Reproduction display conversion failed: %1")
                    .arg(QString::fromLocal8Bit(exception.what())));
            return;
        }
        m_session.reproduction = result;
        m_session.hasReproductionResult = true;
        m_session.binding = display.binding;
        m_session.predictionDisplayModel = display.displayModel;
        handlePredictionFinished(display.scalarField);
        if(!m_session.hasResult) {
            return;
        }
        const auto& statistics = spraythickness::reproductionStatistics(result);
        const QString domain = QString::fromStdString(display.domainLabel);
        m_reproductionStatus = QStringLiteral(
            "%1 completed in %2 ms.\n"
            "Output: %3; elements: %4; candidates: %5; "
            "occlusion queries: %6; occluded: %7.")
            .arg(QString::fromLatin1(
                spraythickness::reproductionAlgorithmName(result.algorithm)))
            .arg(statistics.elapsedMilliseconds, 0, 'f', 3)
            .arg(domain)
            .arg(static_cast<qulonglong>(statistics.evaluatedElementCount))
            .arg(static_cast<qulonglong>(statistics.candidatePairCount))
            .arg(static_cast<qulonglong>(statistics.visibilityQueryCount))
            .arg(static_cast<qulonglong>(statistics.hiddenElementCount));
        for(const std::string& note :
            spraythickness::reproductionImplementationNotes(result)) {
            m_reproductionStatus += QStringLiteral("\n")
                + QString::fromStdString(note);
        }
        m_status = m_reproductionStatus;
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::handleReproductionFailed(
        const QString& message)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        m_session.clearResult();
        m_status = message.isEmpty()
            ? QStringLiteral("Algorithm reproduction failed.")
            : message;
        m_reproductionStatus = m_status;
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 5000);
    }

    void CoatingAnalysisModuleController::exportAlgorithmReproduction()
    {
        if(!reproductionActive() || !m_session.hasReproductionResult
            || !m_session.hasResult) {
            return;
        }
        QSettings settings;
        const QString exportDirectory = settings.value(
            QString::fromLatin1(kReproductionExportDirectorySettingsKey)).toString();
        const QString algorithmId = QString::fromLatin1(
            spraythickness::reproductionAlgorithmId(
                m_session.reproduction.algorithm));
        const bool surfaceGeometry = m_session.reproduction.algorithm
                == spraythickness::ReproductionAlgorithmKind::Vanerio2021
            || m_session.reproduction.algorithm
                == spraythickness::ReproductionAlgorithmKind::DynamicSurface2026;
        const QString extension = surfaceGeometry
            ? QStringLiteral(".stl") : QStringLiteral(".csv");
        const QString initialPath = exportDirectory.isEmpty()
            ? algorithmId + extension
            : QDir(exportDirectory).filePath(algorithmId + extension);
        const QString path = QFileDialog::getSaveFileName(
            &m_panel,
            QStringLiteral("Export algorithm reproduction"),
            initialPath,
            surfaceGeometry ? QStringLiteral("STL files (*.stl)")
                            : QStringLiteral("CSV files (*.csv)"));
        if(path.isEmpty()) {
            return;
        }

        QFile file(path);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            m_status = QStringLiteral("Reproduction export failed: %1")
                .arg(file.errorString());
            m_reproductionStatus = m_status;
            refreshViewModel();
            return;
        }
        QTextStream stream(&file);
        stream.setCodec("UTF-8");
        if(surfaceGeometry) {
            const spraythickness::published::TriangleMesh* mesh = nullptr;
            if(const auto* vanerio = std::get_if<
                    spraythickness::published::VanerioResult>(
                    &m_session.reproduction.nativeResult)) {
                mesh = &vanerio->evolvedStlSurface;
            } else if(const auto* dynamic = std::get_if<
                    spraythickness::published::DynamicSurfaceResult>(
                    &m_session.reproduction.nativeResult)) {
                mesh = &dynamic->evolvedStlSurface;
            }
            if(mesh == nullptr) {
                m_status = QStringLiteral(
                    "Reproduction export failed: dynamic result has no STL surface.");
                m_reproductionStatus = m_status;
                refreshViewModel();
                return;
            }
            stream << "solid " << algorithmId << '\n';
            for(std::size_t faceIndex = 0;
                faceIndex < mesh->faces.size(); ++faceIndex) {
                const Eigen::Vector3d normal =
                    spraythickness::published::faceNormal(*mesh, faceIndex);
                const auto& face = mesh->faces[faceIndex];
                stream << "  facet normal " << normal.x() << ' ' << normal.y()
                       << ' ' << normal.z() << '\n';
                stream << "    outer loop\n";
                for(const std::uint32_t vertex : face) {
                    const Eigen::Vector3d& point = mesh->vertices[vertex];
                    stream << "      vertex " << point.x() << ' ' << point.y()
                           << ' ' << point.z() << '\n';
                }
                stream << "    endloop\n  endfacet\n";
            }
            stream << "endsolid " << algorithmId << '\n';
        } else {
            const auto& statistics = spraythickness::reproductionStatistics(
                m_session.reproduction);
            stream << "algorithm," << algorithmId << '\n';
            stream << "elapsed_ms," << statistics.elapsedMilliseconds << '\n';
            std::visit([&](const auto& native) {
                using Result = std::decay_t<decltype(native)>;
                if constexpr(std::is_same_v<Result,
                    spraythickness::CurrentMethodReproductionResult>) {
                    stream << "domain,index,x_m,y_m,z_m,thickness_m\n";
                    for(const auto& value : native.prediction.field.results) {
                        if(value.sampleIndex >= m_reproductionWorkpiece.samples.size()) {
                            continue;
                        }
                        const Eigen::Vector3d& point =
                            m_reproductionWorkpiece.samples[value.sampleIndex].position;
                        stream << "vertex," << value.sampleIndex << ',' << point.x()
                               << ',' << point.y() << ',' << point.z() << ','
                               << value.thickness << '\n';
                    }
                } else if constexpr(std::is_same_v<Result,
                    spraythickness::published::TanakaResult>) {
                    stream << "domain,index,x_m,y_m,z_m,thickness_m\n";
                    for(std::size_t index = 0;
                        index < native.pointThicknessMeters.size(); ++index) {
                        const Eigen::Vector3d& point =
                            m_reproductionWorkpiece.samples[index].position;
                        stream << "target_point," << index << ',' << point.x() << ','
                               << point.y() << ',' << point.z() << ','
                               << native.pointThicknessMeters[index] << '\n';
                    }
                } else if constexpr(std::is_same_v<Result,
                    spraythickness::published::TzinavaResult>) {
                    stream << "domain,index,x_m,y_m,z_m,thickness_m\n";
                    for(std::size_t index = 0;
                        index < native.faceThicknessMeters.size(); ++index) {
                        const Eigen::Vector3d& point = native.faceCentroids[index];
                        stream << "face," << index << ',' << point.x() << ','
                               << point.y() << ',' << point.z() << ','
                               << native.faceThicknessMeters[index] << '\n';
                    }
                } else if constexpr(std::is_same_v<Result,
                    spraythickness::published::FukeResult>) {
                    stream << "domain,index,x_m,y_m,z_m,thickness_m\n";
                    for(std::size_t index = 0;
                        index < native.polygonThicknessMeters.size(); ++index) {
                        const Eigen::Vector3d& point = native.polygonCentroids[index];
                        stream << "polygon," << index << ',' << point.x() << ','
                               << point.y() << ',' << point.z() << ','
                               << native.polygonThicknessMeters[index] << '\n';
                    }
                } else if constexpr(std::is_same_v<Result,
                    spraythickness::published::WuResult>) {
                    stream << "index,base_x_m,base_y_m,base_z_m,direction_x,"
                              "direction_y,direction_z,radius_m,height_m\n";
                    for(std::size_t index = 0;
                        index < native.depositedCylinders.size(); ++index) {
                        const auto& cylinder = native.depositedCylinders[index];
                        stream << index << ',' << cylinder.baseCenter.x() << ','
                               << cylinder.baseCenter.y() << ','
                               << cylinder.baseCenter.z() << ','
                               << cylinder.growthDirection.x() << ','
                               << cylinder.growthDirection.y() << ','
                               << cylinder.growthDirection.z() << ','
                               << cylinder.radiusMeters << ','
                               << cylinder.heightMeters << '\n';
                    }
                }
            }, m_session.reproduction.nativeResult);
        }
        file.close();
        settings.setValue(
            QString::fromLatin1(kReproductionExportDirectorySettingsKey),
            QFileInfo(path).absolutePath());
        m_status = QStringLiteral("Algorithm reproduction exported to %1.").arg(path);
        m_reproductionStatus = m_status;
        refreshViewModel();
        emit statusMessageRequested(m_status, 5000);
    }

    void CoatingAnalysisModuleController::previewLocalInputs()
    {
        const auto previewStart = std::chrono::steady_clock::now();
        if(anyPredictionRunning() || m_session.trajectory.empty() ||
            !ensurePreviewWorkpieceLoaded() || !hasEffectiveRotationAxis()) {
            m_status = QStringLiteral(
                "Configure a fitted or manual rotation axis and load a trajectory before previewing local inputs.");
            refreshViewModel();
            return;
        }

        spraythickness::ThicknessPredictionTask task;
        task.trajectory = m_session.trajectory;
        task.tool.name = "Legacy spray gun";
        task.tool.sprayDirectionLocal = m_panel.sprayDirectionLocal();
        task.tool.powderFeedDirectionLocal = m_panel.powderFeedDirectionLocal();
        task.process.id = spraythickness::thicknessModelId(m_panel.thicknessModel());
        task.options.base.trajectorySamplingMode = m_panel.trajectorySamplingMode();
        task.options.base.timeStep = m_panel.timeStepSeconds();
        task.options.periodicLocal.enabled = true;
        task.options.periodicLocal.axisOrigin = effectiveRotationAxisOrigin();
        task.options.periodicLocal.axisDirection = effectiveRotationAxisDirection();
        task.options.periodicLocal.referenceDirection =
            std::abs(task.options.periodicLocal.axisDirection.x()) < 0.9
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY();
        task.options.periodicLocal.sectorCount = m_panel.periodicSectorCount();
        task.options.periodicLocal.angularHaloRadians = 0.0;
        task.options.periodicLocal.reduceTrajectory = false;

        const spraythickness::opengl::PeriodicSectorReduction reduction =
            spraythickness::opengl::buildPeriodicSectorReduction(
                m_rotationPreviewWorkpiece,
                task.options.periodicLocal,
                false);
        const double sectorSelectionMilliseconds = elapsedMilliseconds(previewStart);
        if(!reduction.localSelectionValid()) {
            m_hasLocalPreview = false;
            m_localPreviewDetails.clear();
            m_status = QStringLiteral("Local input preview failed: %1")
                .arg(QString::fromStdString(reduction.failureReason));
            refreshViewModel();
            return;
        }

        const std::vector<spraythickness::opengl::PeriodicSpraySample> samples =
            spraythickness::opengl::makePeriodicSpraySamples(task);
        std::vector<std::size_t> selectedSprayIndices(samples.size());
        std::iota(selectedSprayIndices.begin(), selectedSprayIndices.end(), 0U);

        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }

        CoatingPredictionDebugState state;
        state.visible = true;
        state.objectId = m_session.objectId;
        configureAxisDebugState(
            state,
            m_rotationPreviewWorkpiece,
            task.options.periodicLocal.axisOrigin,
            task.options.periodicLocal.axisDirection);

        state.cylindricalTriangles.reserve(m_rotationSurfaceTriangleIndices.size());
        for(const std::size_t triangleIndex : m_rotationSurfaceTriangleIndices) {
            state.cylindricalTriangles.push_back(
                makeDebugTriangle(m_rotationPreviewWorkpiece, triangleIndex));
        }
        if(m_hasRotationAxis) {
            state.seedTriangles.push_back(
                makeDebugTriangle(m_rotationPreviewWorkpiece, m_rotationSeedTriangleIndex));
        }

        state.localSectorTriangles.reserve(reduction.predictionTriangleIndices.size());
        state.localSectorVertexIndices = reduction.predictionVertexIndices;
        for(const std::uint32_t triangleIndex : reduction.predictionTriangleIndices) {
            state.localSectorTriangles.push_back(
                makeDebugTriangle(m_rotationPreviewWorkpiece, triangleIndex));
        }
        state.sprayPoints.reserve(selectedSprayIndices.size());
        for(const std::size_t index : selectedSprayIndices) {
            if(index >= samples.size()) {
                continue;
            }
            const auto& sample = samples[index];
            CoatingPredictionDebugPoint point;
            point.positionX = sample.position.x();
            point.positionY = sample.position.y();
            point.positionZ = sample.position.z();
            point.directionX = sample.direction.x();
            point.directionY = sample.direction.y();
            point.directionZ = sample.direction.z();
            state.sprayPoints.push_back(point);
        }
        services->setCoatingPredictionDebugState(state);
        applyLocalDebugVisibility();
        const double previewMilliseconds = elapsedMilliseconds(previewStart);
        LOG_DEBUG("rs2026") << "Coating local preview: sectorSelectionMs="
            << sectorSelectionMilliseconds
            << ", totalMs=" << previewMilliseconds
            << ", localTriangles=" << reduction.predictionTriangleIndices.size()
            << ", localVertices=" << reduction.predictionVertexIndices.size()
            << ", sprayPoints=" << selectedSprayIndices.size();

        const double sectorAngleDegrees =
            360.0 / static_cast<double>(task.options.periodicLocal.sectorCount);
        m_localPreviewDetails = QStringLiteral(
            "Local preview: vertices %1, triangles %2, spray points %3/%4, base angle %5 deg, exact triangle-sector selection, fit %6 ms, preview %7 ms")
            .arg(static_cast<qulonglong>(reduction.predictionVertexIndices.size()))
            .arg(static_cast<qulonglong>(reduction.predictionTriangleIndices.size()))
            .arg(static_cast<qulonglong>(selectedSprayIndices.size()))
            .arg(static_cast<qulonglong>(samples.size()))
            .arg(sectorAngleDegrees, 0, 'f', 2)
            .arg(m_rotationFitElapsedMilliseconds, 0, 'f', 1)
            .arg(previewMilliseconds, 0, 'f', 1);
        m_hasLocalPreview = true;
        m_status = m_localPreviewDetails;
        refreshViewModel();
        emit statusMessageRequested(m_status, 5000);
    }

    bool CoatingAnalysisModuleController::ensureAxisymmetricProfileSlice()
    {
        if(!m_axisymmetricProfile->slice.valid()) {
            const Eigen::Vector3d axis = effectiveRotationAxisDirection();
            const Eigen::Vector3d reference = std::abs(axis.x()) < 0.9
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY();
            m_axisymmetricProfile->slice =
                spraythickness::opengl::buildAxisymmetricProfileSlice(
                    m_rotationPreviewWorkpiece,
                    effectiveRotationAxisOrigin(),
                    axis,
                    reference);
        }
        if(!m_axisymmetricProfile->slice.valid()) {
            m_status = QStringLiteral("Profile extraction failed: %1").arg(
                QString::fromStdString(m_axisymmetricProfile->slice.failureReason));
            return false;
        }
        return true;
    }

    void CoatingAnalysisModuleController::clearAxisymmetricProfileSelection()
    {
        m_axisymmetricProfile->selection =
            spraythickness::opengl::AxisymmetricProfileSelection();
        m_axisymmetricProfile->reduction =
            spraythickness::opengl::AxisymmetricProfileReduction();
        m_axisymmetricProfile->slice =
            spraythickness::opengl::AxisymmetricProfileSlice();
        m_axisymmetricProfile->details.clear();
    }

    void CoatingAnalysisModuleController::updateAxisymmetricProfileDebugState()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || m_rotationPreviewWorkpiece.empty()) {
            return;
        }
        CoatingPredictionDebugState state;
        state.visible = true;
        state.objectId = m_session.objectId;
        configureAxisDebugState(
            state,
            m_rotationPreviewWorkpiece,
            effectiveRotationAxisOrigin(),
            effectiveRotationAxisDirection());
        state.cylindricalTriangles.reserve(m_rotationSurfaceTriangleIndices.size());
        for(const std::size_t triangleIndex : m_rotationSurfaceTriangleIndices) {
            state.cylindricalTriangles.push_back(
                makeDebugTriangle(m_rotationPreviewWorkpiece, triangleIndex));
        }
        if(m_hasRotationAxis) {
            state.seedTriangles.push_back(
                makeDebugTriangle(m_rotationPreviewWorkpiece, m_rotationSeedTriangleIndex));
        }
        std::size_t completeLinePointCount = 0;
        for(const auto& contour : m_axisymmetricProfile->slice.contours) {
            if(contour.points.size() >= 2) {
                completeLinePointCount += (contour.points.size() - 1) * 2;
                if(contour.closed) {
                    completeLinePointCount += 2;
                }
            }
        }
        state.profileLinePoints.reserve(completeLinePointCount);
        const auto appendProfilePoint = [](const auto& point,
            std::vector<CoatingPredictionDebugPoint>& output) {
            CoatingPredictionDebugPoint debugPoint;
            debugPoint.positionX = point.position.x();
            debugPoint.positionY = point.position.y();
            debugPoint.positionZ = point.position.z();
            output.push_back(debugPoint);
        };
        for(const auto& contour : m_axisymmetricProfile->slice.contours) {
            for(std::size_t index = 0; index + 1 < contour.points.size(); ++index) {
                appendProfilePoint(contour.points[index], state.profileLinePoints);
                appendProfilePoint(contour.points[index + 1], state.profileLinePoints);
            }
            if(contour.closed && contour.points.size() >= 2) {
                appendProfilePoint(contour.points.back(), state.profileLinePoints);
                appendProfilePoint(contour.points.front(), state.profileLinePoints);
            }
        }
        state.selectedProfileLinePoints.reserve(
            m_axisymmetricProfile->reduction.displayLineSegments.size());
        for(const auto& point : m_axisymmetricProfile->reduction.displayLineSegments) {
            appendProfilePoint(point, state.selectedProfileLinePoints);
        }
        services->setCoatingPredictionDebugState(state);
        applyLocalDebugVisibility();
    }

    bool CoatingAnalysisModuleController::rebuildAxisymmetricProfileReduction()
    {
        const auto reduction =
            spraythickness::opengl::buildAxisymmetricProfileReduction(
                m_axisymmetricProfile->slice,
                m_axisymmetricProfile->selection,
                m_panel.axisymmetricProfileSampleCount());
        if(!reduction.valid()) {
            m_status = QStringLiteral("Profile region preparation failed: %1").arg(
                QString::fromStdString(reduction.failureReason));
            refreshViewModel();
            return false;
        }
        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setSurfaceScalarProbeEnabled(false, QString());
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            m_session.clearResult();
            m_hasCurrentThickness = false;
        }
        m_axisymmetricProfile->reduction = reduction;
        m_axisymmetricProfile->details = QStringLiteral(
            "Profile region: %1/%2 uniform samples, %3 segments, arc length %4 mm")
            .arg(static_cast<qulonglong>(reduction.actualSampleCount))
            .arg(static_cast<qulonglong>(reduction.requestedSampleCount))
            .arg(static_cast<qulonglong>(reduction.sampleSegments.size()))
            .arg(reduction.selectedArcLengthMeters * 1000.0, 0, 'f', 3);
        m_status = m_axisymmetricProfile->details;
        updateAxisymmetricProfileDebugState();
        applyModelVisibilityOverrides();
        refreshViewModel();
        publishStateChanged();
        return true;
    }

    void CoatingAnalysisModuleController::selectAxisymmetricProfileRegion()
    {
        if(anyPredictionRunning() || !ensurePreviewWorkpieceLoaded()
            || !hasEffectiveRotationAxis() || !ensureAxisymmetricProfileSlice()) {
            refreshViewModel();
            return;
        }
        AxisymmetricProfileSelectionDialog dialog(
            m_axisymmetricProfile->slice, &m_panel);
        dialog.setLanguageCode(m_languageCode);
        if(dialog.exec() != QDialog::Accepted) {
            return;
        }
        const auto selection = dialog.selection();
        if(!selection.enabled) {
            m_status = QStringLiteral("Draw a closed freeform profile region before accepting.");
            refreshViewModel();
            return;
        }
        m_axisymmetricProfile->selection = selection;
        if(m_panel.adaptiveMeshPredictionEnabled()
            || m_panel.localCandidateVertexPredictionEnabled()) {
            // Build the profile reduction for visualization only. These two
            // modes still predict from their own full-model vertex selection;
            // the reduction supplies the green selected contour line and must
            // not replace the prediction vertex list.
            if(!rebuildAxisymmetricProfileReduction()) {
                return;
            }
            m_status = m_panel.localCandidateVertexPredictionEnabled()
                ? QStringLiteral(
                    "Local prediction region selected. Outside vertices will remain zero; complete-model BVH will be used.")
                : QStringLiteral(
                    "Adaptive dense region selected. Outside target vertex ratio: %1%%.")
                    .arg(m_panel.adaptiveMeshSimplificationPercent(), 0, 'f', 1);
            refreshViewModel();
            publishStateChanged();
            return;
        }
        rebuildAxisymmetricProfileReduction();
    }

    void CoatingAnalysisModuleController::cancelPrediction()
    {
        if(!m_predictionJob->isRunning()) {
            return;
        }
        m_predictionJob->cancel();
        m_status = QStringLiteral("Canceling GPU thickness prediction...");
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handlePredictionProgress(
        double progress,
        const QString& message)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        m_predictionProgress = std::clamp(progress, 0.0, 1.0);
        m_status = message;
        if(message.contains(QStringLiteral("Spatial grid ready:"))
            || message.contains(QStringLiteral("Spatial grid cache hit:"))
            || message.contains(QStringLiteral("Axisymmetric mapping built"))
            || message.contains(QStringLiteral("Axisymmetric mapping cache hit"))) {
            LOG_DEBUG("rs2026") << message.toStdString();
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handlePredictionFinished(
        const spraythickness::ThicknessPredictionResult& prediction)
    {
        const bool reproductionGpu = m_reproductionGpuRun;
        const QString objectId = m_predictionObjectId;
        if(objectId.isEmpty()) {
            return;
        }
        const double elapsedSeconds = m_predictionTimerActive
            ? std::chrono::duration<double>(
                std::chrono::steady_clock::now() - m_predictionStartedAt).count()
            : 0.0;
        m_predictionTimerActive = false;
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;

        if(prediction.field.empty()) {
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = prediction.warnings.empty()
                ? QStringLiteral("GPU thickness prediction produced no result.")
                : QString::fromStdString(prediction.warnings.front());
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
            return;
        }

        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || (objectId != QString::fromLatin1(kSimulationPlateObjectId)
                && findObject(m_context.document(), objectId) == nullptr)) {
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = QStringLiteral("The analysis model or viewport is no longer available.");
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
            return;
        }

        try {
            smrobot::visualization::SurfaceScalarOverlay overlay =
                PaintingAnalysisMeshAdapter::makeOverlay(
                    objectId.toStdString(),
                    m_session.binding,
                    prediction);
            if(m_session.manualThicknessRange) {
                overlay.range.minimum = m_session.minimumDisplayThicknessMeters;
                overlay.range.maximum = m_session.maximumDisplayThicknessMeters;
            }
            // Restore the original workpiece before creating the thickness
            // model. Otherwise deferred local-debug cleanup can replace the
            // freshly uploaded thickness overlay on the next viewport frame.
            services->clearCoatingPredictionDebugState();
            QString applyError;
            const bool customDisplay = m_session.predictionDisplayModel != nullptr;
            const bool overlayApplied = customDisplay
                ? services->applySurfaceScalarOverlayModel(
                    overlay, *m_session.predictionDisplayModel, &applyError, true)
                : services->applySurfaceScalarOverlay(overlay, &applyError, true);
            if(!overlayApplied) {
                m_reproductionGpuRun = false;
                m_session.clearResult();
                m_status = applyError.isEmpty()
                    ? QStringLiteral("Failed to display the thickness result.")
                    : applyError;
                if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                    m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
                }
                refreshViewModel();
                publishStateChanged();
                return;
            }

            m_session.prediction = prediction;
            m_session.thicknessOverlay = overlay;
            m_session.overlay = std::move(overlay);
            m_session.showRelativeError = false;
            m_session.predictionElapsedSeconds = elapsedSeconds;
            m_session.hasResult = true;
            m_session.showThickness = true;
            updateThicknessRangeAndStatistics();
            if(reproductionGpu) {
                m_session.reproduction = spraythickness::AlgorithmReproductionResult();
                m_session.reproduction.algorithm =
                    spraythickness::ReproductionAlgorithmKind::CurrentMethod;
                spraythickness::CurrentMethodReproductionResult current;
                current.prediction = prediction;
                current.thicknessModel = m_panel.thicknessModel();
                current.bvhOcclusion = m_panel.bvhOcclusionEnabled();
                current.historyCorrection = m_panel.historyCorrectionEnabled();
                current.statistics.elapsedMilliseconds = elapsedSeconds * 1000.0;
                current.statistics.evaluatedElementCount =
                    prediction.field.results.size();
                current.statistics.trajectorySampleCount =
                    prediction.timing.sprayPointCount;
                m_session.reproduction.nativeResult = std::move(current);
                m_session.hasReproductionResult = true;
            }
            // Ensure the prediction workpiece node is visible so the scalar
            // overlay model actually renders.
            services->setCoatingModelVisible(objectId, true);
            services->setSurfaceScalarOverlayVisible(objectId, true);
            services->setSurfaceScalarProbeEnabled(false, QString());
            m_status = QStringLiteral("GPU thickness prediction completed in %1 s.")
                .arg(elapsedSeconds, 0, 'f', 3);
            if(reproductionGpu) {
                m_reproductionStatus = QStringLiteral(
                    "Current method (GPU) completed in %1 s.\n"
                    "Output: vertices; elements: %2; spray samples: %3.")
                    .arg(elapsedSeconds, 0, 'f', 3)
                    .arg(static_cast<qulonglong>(prediction.field.results.size()))
                    .arg(static_cast<qulonglong>(prediction.timing.sprayPointCount));
                m_status = m_reproductionStatus;
            }
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                const double activeDuration = activeSprayDurationSeconds(m_simulation.trajectory);
                const double volume = thicknessVolumeCubicMillimeters(
                    m_simulation.workpiece, m_session.prediction.field);
                m_simulation.thicknessVolumeCubicMillimeters = volume;
                m_simulationStatus = QStringLiteral(
                    "Simulation completed in %1 s.\n"
                    "Effective spray duration: %2 s\n"
                    "Thickness volume: %3 mm3\n")
                    .arg(elapsedSeconds, 0, 'f', 3)
                    .arg(activeDuration, 0, 'f', 3)
                    .arg(volume, 0, 'f', 6) + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
            emit statusMessageRequested(m_status, 3000);
            m_reproductionGpuRun = false;
        } catch(const std::exception& exception) {
            m_reproductionGpuRun = false;
            services->clearSurfaceScalarOverlay(objectId);
            services->clearCoatingPredictionModel(objectId);
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
        }
    }

    void CoatingAnalysisModuleController::handlePredictionFailed(const QString& message)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        const bool simulation =
            m_predictionObjectId == QString::fromLatin1(kSimulationPlateObjectId);
        const bool reproductionGpu = m_reproductionGpuRun;
        m_reproductionGpuRun = false;
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->clearSurfaceScalarOverlay(m_session.objectId);
            services->clearCoatingPredictionModel(m_session.objectId);
        }
        m_session.clearResult();
        m_status = message.isEmpty()
            ? QStringLiteral("GPU thickness prediction failed.")
            : message;
        if(simulation) {
            m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
        } else if(reproductionGpu) {
            m_reproductionStatus = m_status;
        }
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 5000);
    }

    void CoatingAnalysisModuleController::setShowModel(bool enabled)
    {
        m_session.showModel = enabled;
        applyModelVisibilityOverrides();
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::setShowTrajectory(bool enabled)
    {
        m_session.showTrajectory = enabled;
        updateTrajectoryPreviewVisibility();
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::setShowSprayPoints(bool enabled)
    {
        m_session.showSprayPoints = enabled;
        updateTrajectoryPreviewVisibility();
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::setLocalDebugVisibility(
        bool cylindricalSurface,
        bool rotationAxis,
        bool localSector,
        bool sprayPoints)
    {
        m_showCylindricalSurface = cylindricalSurface;
        m_showRotationAxis = rotationAxis;
        m_showLocalSector = localSector;
        m_showLocalSprayPoints = sprayPoints;
        applyLocalDebugVisibility();
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::setShowThickness(bool enabled)
    {
        m_session.showThickness = enabled && m_session.hasResult;
        if(!m_session.showThickness) {
            m_session.thicknessPickEnabled = false;
        }
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(
                m_active && m_session.showThickness && m_session.thicknessPickEnabled,
                m_session.objectId);
            if(m_session.hasResult) {
                if(!services->setSurfaceScalarOverlayVisible(
                        m_session.objectId,
                        m_session.showThickness)
                    && m_session.showThickness) {
                    QString error;
                    const bool applied = applyCurrentSurfaceOverlay(&error);
                    if(!applied) {
                        m_session.clearResult();
                        m_status = error.isEmpty()
                            ? QStringLiteral("Failed to re-apply the thickness overlay.")
                            : error;
                    }
                }
            }
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::setThicknessPickEnabled(bool enabled)
    {
        m_session.thicknessPickEnabled = enabled && m_active &&
            m_session.hasResult && m_session.showThickness;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(
                m_session.thicknessPickEnabled,
                m_session.thicknessPickEnabled ? m_session.objectId : QString());
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::selectWorkpiece(const QString& objectId)
    {
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), objectId);
        if(object == nullptr || object->objectType != "workpiece" ||
            objectId == m_session.objectId || anyPredictionRunning()) {
            refreshViewModel();
            return;
        }

        clearModelVisibilityOverrides();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearCoatingPredictionDebugState();
            if(!m_session.objectId.isEmpty() && m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            if(!m_session.objectId.isEmpty()) {
                services->clearCoatingPredictionModel(m_session.objectId);
            }
        }
        m_session.clearResult();
        m_session.manualThicknessRange = false;
        m_session.minimumDisplayThicknessMeters = 0.0;
        m_session.maximumDisplayThicknessMeters = 0.0;
        resetReferenceResult();
        m_session.objectId = objectId;
        m_session.modelName = QString::fromStdString(object->name);
        m_session.sourcePath = QString::fromStdString(object->sourcePath);
        m_session.modelInfo = CoatingAnalysisModelInfo();
        m_hasRotationAxis = false;
        m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
        m_rotationSurfaceTriangleIndices.clear();
        m_rotationSeedTriangleIndex = 0;
        m_rotationFitElapsedMilliseconds = 0.0;
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        clearAxisymmetricProfileSelection();
        m_panel.setPeriodicLocalPredictionEnabled(false);
        m_hasCurrentThickness = false;
        updateSelectedModelInfo();
        applyModelVisibilityOverrides();
        updateTrajectoryPreviewVisibility();
        m_context.selectionModel().selectSceneObject(
            objectId,
            QStringLiteral("coatingAnalysisWorkpiece"));
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            if(m_active) {
                services->selectSceneObject(QString());
                services->focusCoatingObject(objectId, 0.3);
            } else {
                services->selectSceneObject(objectId);
            }
        }
        m_status = QStringLiteral("Prediction workpiece selected: %1")
            .arg(m_session.modelName);
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::selectWorkpieceFromTree(const QString& objectId)
    {
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), objectId);
        if(object != nullptr && object->objectType == "workpiece") {
            selectWorkpiece(objectId);
        }
    }

    void CoatingAnalysisModuleController::ensureWorkpieceSelection()
    {
        const simulation_project::ProjectDocument& document = m_context.document();
        const simulation_project::SceneObjectDesc* selected =
            findObject(document, m_session.objectId);
        if(selected != nullptr && selected->objectType == "workpiece") {
            return;
        }

        const QString treeObjectId = m_context.selectionModel().state().objectId;
        const simulation_project::SceneObjectDesc* treeObject =
            findObject(document, treeObjectId);
        if(treeObject != nullptr && treeObject->objectType == "workpiece") {
            selectWorkpiece(treeObjectId);
            return;
        }
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.objectType == "workpiece") {
                selectWorkpiece(QString::fromStdString(object.id));
                return;
            }
        }
    }

    void CoatingAnalysisModuleController::updateSelectedModelInfo()
    {
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), m_session.objectId);
        if(object == nullptr) {
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
        if(model) {
            m_session.modelInfo = makeModelInfo(*model, makeTransform(object->transform));
        }
    }

    void CoatingAnalysisModuleController::clearModelVisibilityOverrides()
    {
        if(!m_modelVisibilityOverrideIds.empty()) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingModelVisibilities({});
            }
            m_modelVisibilityOverrideIds.clear();
        }
    }

    void CoatingAnalysisModuleController::applyModelVisibilityOverrides()
    {
        if(!m_active || m_session.objectId.isEmpty()) {
            clearModelVisibilityOverrides();
            return;
        }
        m_modelVisibilityOverrideIds.clear();
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }

        QHash<QString, bool> visibility;
        for(const simulation_project::SceneObjectDesc& object : m_context.document().objects) {
            if(object.objectType != "workpiece") {
                continue;
            }
            const QString objectId = QString::fromStdString(object.id);
            const bool requestedVisibility = m_modelVisibility.contains(objectId)
                ? m_modelVisibility.value(objectId)
                : (objectId == m_session.objectId && m_session.showModel);
            // Keep the original STL visible during local preview. The local
            // prediction sector is rendered separately as a green overlay;
            // keeping the source object visible preserves exact picking and
            // rotation-center behavior without changing prediction inputs.
            visibility[objectId] = requestedVisibility;
        }
        services->setCoatingModelVisibilities(visibility);
        for(const QString& objectId : visibility.keys()) {
            m_modelVisibilityOverrideIds.push_back(objectId);
        }
    }

    void CoatingAnalysisModuleController::clearSession()
    {
        m_predictionJob->cancel();
        m_reproductionJob->cancel();
        if(simulationActive()) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                if(!m_session.objectId.isEmpty()) {
                    services->clearCoatingPredictionModel(m_session.objectId);
                    services->setCoatingModelVisible(m_session.objectId, false);
                }
                services->setCoatingTrajectoryPreview({}, false);
            }
            m_mode = CoatingAnalysisMode::Prediction;
            m_simulationReady = false;
            m_simulation = SimulationExperimentData();
        }
        resetReferenceResult();
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        clearModelVisibilityOverrides();
        m_modelVisibility.clear();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.clear();
        m_reproductionWorkpiece = sprayworkpiece::WorkpieceModel();
        m_reproductionStatus = QStringLiteral("No reproduction result yet.");
        m_reproductionGpuRun = false;
        m_hasRotationAxis = false;
        m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
        m_rotationSurfaceTriangleIndices.clear();
        m_rotationSeedTriangleIndex = 0;
        m_rotationFitElapsedMilliseconds = 0.0;
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        clearAxisymmetricProfileSelection();
        m_panel.setPeriodicLocalPredictionEnabled(false);
        m_treePanel.setWaypoints(nullptr);
        m_hasCurrentThickness = false;
        m_status = QStringLiteral("Open a mesh model to begin.");
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::applyVisualizationState()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }
        applyModelVisibilityOverrides();
        updateTrajectoryPreviewVisibility();
    }

    void CoatingAnalysisModuleController::updateTrajectoryPreviewVisibility()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }
        const bool visible = m_active && !m_session.waypoints.empty()
            && (m_session.showTrajectory || m_session.showSprayPoints);
        if(visible && !services->setCoatingTrajectoryPreviewVisible(
                true,
                m_session.showTrajectory,
                m_session.showSprayPoints)) {
            submitTrajectoryPreview();
            return;
        }
        if(!visible) {
            services->setCoatingTrajectoryPreviewVisible(
                false,
                m_session.showTrajectory,
                m_session.showSprayPoints);
        }
    }

    void CoatingAnalysisModuleController::submitTrajectoryPreview()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }

        std::size_t totalPointCount = 0;
        for(const spraytrajectory::SpraySegment& segment : m_session.trajectory.segments) {
            totalPointCount += segment.points.size();
        }
        std::vector<CoatingTrajectoryPreviewPoint> previewPoints;
        previewPoints.reserve(totalPointCount);
        for(const spraytrajectory::SpraySegment& segment : m_session.trajectory.segments) {
            bool startsNewSegment = true;
            for(std::size_t pointIndex = 0; pointIndex < segment.points.size();
                ++pointIndex) {
                const spraytrajectory::SprayPathPoint& point = segment.points[pointIndex];
                const Eigen::Vector3d position = point.tcpPose.translation();
                const Eigen::Matrix3d& rotation = point.tcpPose.linear();
                // Simulation trajectories define the nozzle along local +Z;
                // legacy imported trajectories retain the original +X axis.
                const Eigen::Vector3d direction = rotation *
                    (simulationActive() ? Eigen::Vector3d::UnitZ()
                                         : Eigen::Vector3d::UnitX());
                CoatingTrajectoryPreviewPoint previewPoint;
                previewPoint.positionX = position.x();
                previewPoint.positionY = position.y();
                previewPoint.positionZ = position.z();
                previewPoint.directionX = direction.x();
                previewPoint.directionY = direction.y();
                previewPoint.directionZ = direction.z();
                const Eigen::Vector3d frameX = rotation.col(0);
                const Eigen::Vector3d frameY = rotation.col(1);
                const Eigen::Vector3d frameZ = rotation.col(2);
                previewPoint.frameXAxisX = frameX.x();
                previewPoint.frameXAxisY = frameX.y();
                previewPoint.frameXAxisZ = frameX.z();
                previewPoint.frameYAxisX = frameY.x();
                previewPoint.frameYAxisY = frameY.y();
                previewPoint.frameYAxisZ = frameY.z();
                previewPoint.frameZAxisX = frameZ.x();
                previewPoint.frameZAxisY = frameZ.y();
                previewPoint.frameZAxisZ = frameZ.z();
                previewPoint.sprayEnabled = point.sprayEnabled && segment.sprayEnabled;
                previewPoint.startsNewSegment = startsNewSegment;
                previewPoints.push_back(previewPoint);
                startsNewSegment = false;
            }
        }
        services->setCoatingTrajectoryPreview(
            previewPoints,
            m_active && !previewPoints.empty()
                && (m_session.showTrajectory || m_session.showSprayPoints),
            m_session.showTrajectory,
            m_session.showSprayPoints);
    }

    void CoatingAnalysisModuleController::applyOverlayAfterReload()
    {
        if(!m_session.hasResult || !m_active || !m_session.showThickness ||
            (m_session.objectId != QString::fromLatin1(kSimulationPlateObjectId) &&
             findObject(m_context.document(), m_session.objectId) == nullptr)) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            QString error;
            const bool applied = applyCurrentSurfaceOverlay(&error);
            if(applied) {
                services->setSurfaceScalarProbeEnabled(
                    m_session.showThickness && m_session.thicknessPickEnabled,
                    m_session.objectId);
            } else {
                m_status = error;
                m_session.clearResult();
                refreshViewModel();
                publishStateChanged();
            }
        }
    }

    bool CoatingAnalysisModuleController::applyCurrentSurfaceOverlay(
        QString* errorMessage)
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || !m_session.hasResult) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("The thickness overlay is unavailable.");
            }
            return false;
        }

        const bool blackOutsideRange = !m_session.showRelativeError;
        if(m_session.predictionDisplayModel) {
            return services->applySurfaceScalarOverlayModel(
                m_session.overlay,
                *m_session.predictionDisplayModel,
                errorMessage,
                blackOutsideRange);
        }
        return services->applySurfaceScalarOverlay(
            m_session.overlay,
            errorMessage,
            blackOutsideRange);
    }

    void CoatingAnalysisModuleController::updateThicknessRangeAndStatistics()
    {
        if(!m_session.hasResult || m_session.prediction.field.empty()) {
            m_session.uniformityStatistics = ThicknessUniformityStatistics();
            return;
        }

        const double automaticMinimum = m_session.prediction.metrics.minThickness;
        const double automaticMaximum = m_session.prediction.metrics.maxThickness;
        const bool manualRangeValid = m_session.manualThicknessRange
            && std::isfinite(m_session.minimumDisplayThicknessMeters)
            && std::isfinite(m_session.maximumDisplayThicknessMeters)
            && m_session.minimumDisplayThicknessMeters >= 0.0
            && m_session.minimumDisplayThicknessMeters
                < m_session.maximumDisplayThicknessMeters;
        if(!manualRangeValid) {
            m_session.manualThicknessRange = false;
            m_session.minimumDisplayThicknessMeters = automaticMinimum;
            m_session.maximumDisplayThicknessMeters = automaticMaximum;
        }

        m_session.thicknessOverlay.range.minimum =
            m_session.minimumDisplayThicknessMeters;
        m_session.thicknessOverlay.range.maximum =
            m_session.maximumDisplayThicknessMeters;
        m_session.uniformityStatistics = calculateThicknessUniformityStatistics(
            m_session.prediction.field,
            m_session.minimumDisplayThicknessMeters,
            m_session.maximumDisplayThicknessMeters);
        if(!m_session.showRelativeError) {
            m_session.overlay = m_session.thicknessOverlay;
        }
    }

    void CoatingAnalysisModuleController::refreshViewModel()
    {
        CoatingAnalysisViewModel viewModel;
        for(const simulation_project::SceneObjectDesc& object : m_context.document().objects) {
            if(object.objectType != "workpiece") {
                continue;
            }
            CoatingAnalysisWorkpieceItem item;
            item.id = QString::fromStdString(object.id);
            item.name = QString::fromStdString(
                object.name.empty() ? object.id : object.name);
            viewModel.workpieces.push_back(std::move(item));
        }
        viewModel.selectedWorkpieceId = m_session.objectId;
        viewModel.hasModel = !m_session.objectId.isEmpty();
        viewModel.modelName = viewModel.hasModel
            ? m_session.modelName
            : QStringLiteral("No model loaded");
        viewModel.modelPath = m_session.sourcePath;
        viewModel.hasTrajectory = !m_session.trajectory.empty();
        viewModel.trajectoryName = viewModel.hasTrajectory
            ? m_session.trajectoryName
            : QStringLiteral("No trajectory loaded");
        viewModel.trajectoryPath = m_session.trajectoryPath;
        viewModel.status = m_status;
        viewModel.hasResult = m_session.hasResult;
        viewModel.predictionRunning = anyPredictionRunning();
        viewModel.localMode = m_panel.periodicLocalPredictionEnabled();
        viewModel.axisymmetricProfileMode =
            m_panel.axisymmetricProfilePredictionEnabled();
        viewModel.adaptiveMeshMode = m_panel.adaptiveMeshPredictionEnabled();
        viewModel.localCandidateVertexMode =
            m_panel.localCandidateVertexPredictionEnabled();
        viewModel.hasEffectiveRotationAxis = hasEffectiveRotationAxis();
        viewModel.hasAxisymmetricProfileSelection =
            m_axisymmetricProfile->reduction.valid();
        viewModel.rotationAxisSource = rotationAxisSource();
        viewModel.canStartPrediction = viewModel.hasModel && viewModel.hasTrajectory
            && (!viewModel.localMode || viewModel.hasEffectiveRotationAxis)
            && (!viewModel.adaptiveMeshMode
                || (viewModel.hasEffectiveRotationAxis
                    && m_axisymmetricProfile->selection.enabled))
            && (!viewModel.localCandidateVertexMode
                || (viewModel.hasEffectiveRotationAxis
                    && m_axisymmetricProfile->selection.enabled))
            && (!viewModel.axisymmetricProfileMode
                || (viewModel.hasEffectiveRotationAxis
                    && viewModel.hasAxisymmetricProfileSelection));
        viewModel.canPreviewLocalInputs = viewModel.hasModel && viewModel.hasTrajectory
            && viewModel.hasEffectiveRotationAxis;
        viewModel.canSelectProfileRegion = viewModel.hasModel
            && viewModel.hasEffectiveRotationAxis
            && viewModel.axisymmetricProfileMode;
        viewModel.hasLocalPreview = m_hasLocalPreview;
        viewModel.localPreviewDetails = m_localPreviewDetails;
        viewModel.axisymmetricProfileDetails = m_axisymmetricProfile->details;
        viewModel.showCylindricalSurface = m_showCylindricalSurface;
        viewModel.showRotationAxis = m_showRotationAxis;
        viewModel.showLocalSector = m_showLocalSector;
        viewModel.showLocalSprayPoints = m_showLocalSprayPoints;
        viewModel.progress = m_predictionProgress;
        viewModel.showModel = m_session.showModel;
        viewModel.showTrajectory = m_session.showTrajectory;
        viewModel.showSprayPoints = m_session.showSprayPoints;
        viewModel.showThickness = m_session.showThickness;
        viewModel.thicknessPickEnabled = m_session.thicknessPickEnabled;
        viewModel.hasCurrentThickness = m_hasCurrentThickness;
        viewModel.currentMicrometers = m_currentThicknessMeters * kMetersToMicrometers;
        viewModel.referenceAvailable = !m_referenceThickness.empty();
        viewModel.canSetReference = viewModel.hasResult;
        viewModel.canClearReference = viewModel.referenceAvailable;
        viewModel.canCheckReference = viewModel.hasResult && viewModel.referenceAvailable;
        viewModel.referenceStatus = viewModel.referenceAvailable
            ? QStringLiteral("Reference: ready (%1 active vertices)")
                  .arg(static_cast<qulonglong>(m_referenceActiveVertexCount))
            : QStringLiteral("Reference: not set");
        viewModel.mode = m_mode;
        viewModel.simulationActive = simulationActive();
        viewModel.simulationRunning = simulationActive() && m_predictionJob->isRunning();
        viewModel.canRunSimulation = simulationActive() && m_simulationReady
            && viewModel.hasModel && !m_session.trajectory.empty();
        viewModel.canExportSimulation = simulationActive() && m_session.hasResult;
        viewModel.simulationDetails = m_simulationStatus;
        viewModel.reproductionRunning = reproductionActive()
            && anyPredictionRunning();
        viewModel.canRunReproduction = reproductionActive()
            && viewModel.hasModel && viewModel.hasTrajectory;
        viewModel.canExportReproduction = reproductionActive()
            && m_session.hasReproductionResult;
        viewModel.reproductionDetails = m_reproductionStatus;
        if(simulationActive() || reproductionActive()) {
            viewModel.canStartPrediction = false;
        }
        if(m_session.hasResult) {
            viewModel.minimumMicrometers =
                m_session.showRelativeError
                ? m_session.overlay.range.minimum
                : m_session.thicknessOverlay.range.minimum
                    * kMetersToMicrometers;
            viewModel.maximumMicrometers =
                m_session.showRelativeError
                ? m_session.overlay.range.maximum
                : m_session.thicknessOverlay.range.maximum
                    * kMetersToMicrometers;
            viewModel.midpointMicrometers =
                (viewModel.minimumMicrometers + viewModel.maximumMicrometers) * 0.5;
            viewModel.averageMicrometers =
                m_session.prediction.metrics.averageThickness * kMetersToMicrometers;
        }
        m_panel.applyViewModel(viewModel);

        CoatingAnalysisTreeView treeView;
        treeView.trajectoryName = m_session.trajectoryName;
        treeView.trajectoryInfo = m_session.trajectoryInfo;
        treeView.workpieces = viewModel.workpieces;
        treeView.selectedWorkpieceId = m_session.objectId;
        treeView.showModel = m_session.showModel;
        treeView.hasThickness = m_session.hasResult;
        if(m_session.hasResult) {
            treeView.thicknessMetrics = m_session.prediction.metrics;
        }
        for(const CoatingAnalysisWorkpieceItem& item : viewModel.workpieces) {
            const bool visible = m_modelVisibility.contains(item.id)
                ? m_modelVisibility.value(item.id)
                : (item.id == m_session.objectId && m_session.showModel);
            treeView.modelVisibility.insert(item.id, visible);
        }
        m_treePanel.applyTreeView(treeView);

        CoatingAnalysisInfoView infoView;
        infoView.hasModel = viewModel.hasModel;
        infoView.modelName = m_session.modelName;
        infoView.modelPath = m_session.sourcePath;
        infoView.modelInfo = m_session.modelInfo;
        infoView.hasTrajectory = viewModel.hasTrajectory;
        infoView.trajectoryName = m_session.trajectoryName;
        infoView.trajectoryInfo = m_session.trajectoryInfo;
        infoView.hasThickness = m_session.hasResult;
        infoView.predictionElapsedSeconds = m_session.predictionElapsedSeconds;
        infoView.validationDetails = m_validationDetails;
        infoView.simulationActive = simulationActive();
        infoView.simulationDetails = m_simulationStatus;
        infoView.simulationThicknessVolumeCubicMillimeters =
            simulationActive() && m_session.hasResult
                ? m_simulation.thicknessVolumeCubicMillimeters
                : 0.0;
        if(m_session.hasResult) {
            infoView.thicknessMetrics = m_session.prediction.metrics;
            infoView.predictionTiming = m_session.prediction.timing;
            infoView.manualThicknessRange = m_session.manualThicknessRange;
            infoView.minimumDisplayThicknessMicrometers =
                m_session.thicknessOverlay.range.minimum * kMetersToMicrometers;
            infoView.maximumDisplayThicknessMicrometers =
                m_session.thicknessOverlay.range.maximum * kMetersToMicrometers;
            infoView.uniformityStatistics = m_session.uniformityStatistics;
        }
        m_infoPanel.applyInfo(infoView);

        CoatingAnalysisVisibilityView visibilityView;
        visibilityView.hasModel = viewModel.hasModel;
        visibilityView.hasTrajectory = viewModel.hasTrajectory;
        visibilityView.hasThickness = m_session.hasResult;
        visibilityView.showModel = m_session.showModel;
        visibilityView.showTrajectory = m_session.showTrajectory;
        visibilityView.showSprayPoints = m_session.showSprayPoints;
        visibilityView.showThickness = m_session.showThickness;
        visibilityView.thicknessPickEnabled = m_session.thicknessPickEnabled;
        m_visibilityBar.applyVisibility(visibilityView);

        emit thicknessLegendChanged(
            m_active && viewModel.hasResult,
            viewModel.minimumMicrometers,
            viewModel.maximumMicrometers,
            m_session.showRelativeError,
            !anyPredictionRunning() && !m_session.showRelativeError);
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

    void CoatingAnalysisModuleController::handleWaypointInfoRequested(int index)
    {
        if(index < 0 || index >= static_cast<int>(m_session.waypoints.size())) {
            return;
        }
        CoatingAnalysisWaypointDialog dialog(&m_treePanel);
        dialog.setLanguageCode(m_languageCode);
        dialog.setWaypoint(
            m_session.waypoints[static_cast<std::size_t>(index)],
            static_cast<std::size_t>(index),
            waypointDurationAt(static_cast<std::size_t>(index)));
        dialog.exec();
    }

    void CoatingAnalysisModuleController::handleModelVisibilityToggleRequested(
        const QString& objectId)
    {
        if(objectId.isEmpty()) {
            return;
        }
        const bool defaultVisible =
            objectId == m_session.objectId && m_session.showModel;
        const bool current = m_modelVisibility.contains(objectId)
            ? m_modelVisibility.value(objectId)
            : defaultVisible;
        const bool newVisible = !current;
        m_modelVisibility[objectId] = newVisible;
        applyModelVisibilityOverrides();

        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), objectId);
        const QString name = object != nullptr && !object->name.empty()
            ? QString::fromStdString(object->name)
            : objectId;
        m_status = newVisible
            ? QStringLiteral("Model shown: %1").arg(name)
            : QStringLiteral("Model hidden: %1").arg(name);
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handleModelSetAsWorkpiece(const QString& objectId)
    {
        selectWorkpiece(objectId);
    }

    void CoatingAnalysisModuleController::handleThicknessClearRequested()
    {
        if(!m_session.hasResult) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearSurfaceScalarOverlay(m_session.objectId);
        }
        m_session.clearResult();
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_status = QStringLiteral("Thickness result cleared.");
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::handleRotationSurfacePicked(
        const QString& objectId,
        std::uint32_t triangleIndex,
        double hitX,
        double hitY,
        double hitZ,
        double normalX,
        double normalY,
        double normalZ)
    {
        const auto pickStartedAt = std::chrono::steady_clock::now();
        (void)hitX;
        (void)hitY;
        (void)hitZ;
        (void)normalX;
        (void)normalY;
        (void)normalZ;
        if(anyPredictionRunning()) {
            return;
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), objectId);
        if(object == nullptr || object->objectType != "workpiece") {
            m_status = QStringLiteral("The picked object is not a workpiece.");
            refreshViewModel();
            return;
        }
        if(objectId != m_session.objectId) {
            selectWorkpiece(objectId);
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
                ? "Failed to load the selected workpiece mesh."
                : loadError);
            refreshViewModel();
            return;
        }

        try {
            const auto meshBuildStartedAt = std::chrono::steady_clock::now();
            const PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
                *model,
                object->name,
                sourcePath.generic_u8string(),
                makeTransform(object->transform));
            const double meshBuildMilliseconds = elapsedMilliseconds(meshBuildStartedAt);

            // Show the exact picked face immediately. This keeps the selection
            // step observable even when the subsequent cylindrical fit fails.
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingPredictionDebugState(
                    makeSeedDebugState(mesh, static_cast<std::size_t>(triangleIndex)));
            }
            const auto fitStartedAt = std::chrono::steady_clock::now();
            const spraythickness::CylindricalSurfaceFitResult fit =
                spraythickness::fitCylindricalSurface(mesh.workpiece, triangleIndex);
            const double fitMilliseconds = elapsedMilliseconds(fitStartedAt);
            LOG_DEBUG("rs2026") << "Rotation surface fit: object="
                << objectId.toStdString()
                << ", triangle=" << triangleIndex
                << ", meshBuildMs=" << meshBuildMilliseconds
                << ", fitMs=" << fitMilliseconds
                << ", totalMs=" << elapsedMilliseconds(pickStartedAt)
                << ", selectedTriangles=" << fit.triangleCount
                << ", valid=" << fit.valid()
                << ", failure=" << fit.failureReason;
            if(!fit.valid()) {
                m_hasRotationAxis = false;
                m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
                m_rotationSurfaceTriangleIndices.clear();
                m_rotationSeedTriangleIndex = 0;
                m_rotationFitElapsedMilliseconds = fitMilliseconds;
                m_hasLocalPreview = false;
                m_localPreviewDetails.clear();
                clearAxisymmetricProfileSelection();
                m_status = QStringLiteral("Cylindrical fit failed: %1")
                    .arg(QString::fromStdString(fit.failureReason));
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->clearCoatingPredictionDebugState();
                }
                applyModelVisibilityOverrides();
                refreshViewModel();
                return;
            }
            m_hasRotationAxis = true;
            m_rotationAxisOrigin = fit.axisOrigin;
            m_rotationAxisDirection = fit.axisDirection;
            m_rotationPreviewWorkpiece = mesh.workpiece;
            m_rotationSurfaceTriangleIndices = fit.selectedTriangleIndices;
            m_rotationSeedTriangleIndex = fit.seedTriangleIndex;
            m_rotationFitElapsedMilliseconds = fitMilliseconds;
            m_hasLocalPreview = false;
            m_localPreviewDetails.clear();
            clearAxisymmetricProfileSelection();
            const bool modeWasAlreadyLocal = m_panel.periodicLocalPredictionEnabled();
            const bool modeWasAlreadyProfile =
                m_panel.axisymmetricProfilePredictionEnabled();
            const bool modeWasAlreadyAdaptive =
                m_panel.adaptiveMeshPredictionEnabled();
            const bool modeWasAlreadyLocalCandidate =
                m_panel.localCandidateVertexPredictionEnabled();
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingPredictionDebugState(
                    makeRotationFitDebugState(mesh, fit));
            }
            if(!modeWasAlreadyLocal && !modeWasAlreadyProfile && !modeWasAlreadyAdaptive
                && !modeWasAlreadyLocalCandidate) {
                m_panel.setPeriodicLocalPredictionEnabled(true);
            }
            if(modeWasAlreadyLocal || modeWasAlreadyProfile || modeWasAlreadyAdaptive
                || modeWasAlreadyLocalCandidate) {
                applyModelVisibilityOverrides();
            }
            if(modeWasAlreadyLocal && !m_session.trajectory.empty()) {
                previewLocalInputs();
            } else if(modeWasAlreadyProfile) {
                if(ensureAxisymmetricProfileSlice()) {
                    updateAxisymmetricProfileDebugState();
                    m_status = QStringLiteral(
                        "Rotation axis updated. Select the profile prediction region again.");
                }
                refreshViewModel();
            } else if(modeWasAlreadyAdaptive) {
                if(ensureAxisymmetricProfileSlice()) {
                    updateAxisymmetricProfileDebugState();
                    m_status = QStringLiteral(
                        "Rotation axis updated. Select the dense prediction region again.");
                }
                refreshViewModel();
            } else if(modeWasAlreadyLocalCandidate) {
                if(ensureAxisymmetricProfileSlice()) {
                    updateAxisymmetricProfileDebugState();
                    m_status = QStringLiteral(
                        "Rotation axis updated. Select the local prediction region again.");
                }
                refreshViewModel();
            } else if(m_session.trajectory.empty()) {
                m_status = QStringLiteral(
                    "Axis fitted: dir=(%1, %2, %3), R=%4 mm, faces=%5, RMS=%6 mm, fit=%7 ms")
                    .arg(fit.axisDirection.x(), 0, 'f', 4)
                    .arg(fit.axisDirection.y(), 0, 'f', 4)
                    .arg(fit.axisDirection.z(), 0, 'f', 4)
                    .arg(fit.radius * 1000.0, 0, 'f', 3)
                    .arg(static_cast<qulonglong>(fit.triangleCount))
                    .arg(fit.rmsRadialError * 1000.0, 0, 'f', 3)
                    .arg(fitMilliseconds, 0, 'f', 1);
                refreshViewModel();
            }
        } catch(const std::exception& exception) {
            m_hasRotationAxis = false;
            m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
            m_rotationSurfaceTriangleIndices.clear();
            m_rotationSeedTriangleIndex = 0;
            m_rotationFitElapsedMilliseconds = 0.0;
            m_hasLocalPreview = false;
            m_localPreviewDetails.clear();
            clearAxisymmetricProfileSelection();
            m_status = QString::fromLocal8Bit(exception.what());
            refreshViewModel();
        }
    }
}
