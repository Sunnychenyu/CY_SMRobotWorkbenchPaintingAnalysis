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
#include "OnlineThicknessPredictionJobController.h"
#include "OnlineVirtualMotion.h"
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
#include <RotationBodyTrajectoryPlanning/Persistence/PublishedTrajectoryPlanContract.h>
#include <RotationBodyTrajectoryPlanning/Persistence/PublishedTrajectoryPlanSprayTrajectoryAdapter.h>
#include <RotationBodyTrajectoryPlanning/Persistence/PublishedTrajectoryPlanStore.h>
#include <RotationBodyTrajectoryOptimization/RotationBodyTrajectoryOptimization.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/ProjectSession.h>
#include <SimulationProject/RuntimePaths.h>
#include <SprayTrajectoryCore/LegacySprayTrajectoryIo.h>
#include <SprayTrajectoryCore/SprayTrajectoryIo.h>
#include <QDir>
#include <QDateTime>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QMenu>
#include <QMessageBox>
#include <QRandomGenerator>
#include <QSettings>
#include <QScreen>
#include <QTextStream>
#include <QTimer>
#include <QWindow>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>
#include <optional>
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
    const QString kSavedRotationBodyTrajectoryPath =
        QStringLiteral("project://rotation-body-trajectory-plan");
    const QString kDualOptimizationBaselineRotationBodyTrajectoryPath =
        QStringLiteral("project://rotation-body-dual-optimization-baseline-trajectory");
    const QString kDualOptimizedRotationBodyTrajectoryPath =
        QStringLiteral("project://rotation-body-dual-optimized-trajectory");
    const QString kThreeOptimizationBaselineRotationBodyTrajectoryPath =
        QStringLiteral("project://rotation-body-three-optimization-baseline-trajectory");
    const QString kThreeOptimizedRotationBodyTrajectoryPath =
        QStringLiteral("project://rotation-body-three-optimized-trajectory");
    constexpr const char* kReproductionExportDirectorySettingsKey =
        "PaintingAnalysis/ReproductionExportDirectory";
    constexpr std::array<spraythickness::ReproductionAlgorithmKind, 5>
        kBenchmarkAlgorithms{
            spraythickness::ReproductionAlgorithmKind::Tzinava2020,
            spraythickness::ReproductionAlgorithmKind::Wu2020,
            spraythickness::ReproductionAlgorithmKind::Fuke2005,
            spraythickness::ReproductionAlgorithmKind::Vanerio2021,
            spraythickness::ReproductionAlgorithmKind::CurrentMethod
        };

    QString csvCell(QString value)
    {
        value.replace(QLatin1Char('"'), QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QStringLiteral("\"");
    }

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

    QString thicknessFieldDiagnostic(const spraythickness::ThicknessField& field)
    {
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        std::size_t finiteCount = 0;
        std::size_t nonzeroCount = 0;
        for(const auto& sample : field.results) {
            if(!std::isfinite(sample.thickness)) {
                continue;
            }
            ++finiteCount;
            nonzeroCount += sample.thickness != 0.0;
            minimum = std::min(minimum, sample.thickness);
            maximum = std::max(maximum, sample.thickness);
        }
        return QStringLiteral("samples=%1, finite=%2, nonzero=%3, min=%4 um, max=%5 um")
            .arg(static_cast<qulonglong>(field.results.size()))
            .arg(static_cast<qulonglong>(finiteCount))
            .arg(static_cast<qulonglong>(nonzeroCount))
            .arg(finiteCount ? QString::number(minimum * kMetersToMicrometers, 'g', 9)
                             : QStringLiteral("N/A"))
            .arg(finiteCount ? QString::number(maximum * kMetersToMicrometers, 'g', 9)
                             : QStringLiteral("N/A"));
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

    double activeSprayDurationSeconds(
        const std::vector<spraytrajectory::SprayTrajectorySample>& samples)
    {
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

    simulation_project::TransformDesc makeTransformDesc(
        const Eigen::Isometry3d& transform)
    {
        simulation_project::TransformDesc desc;
        desc.x = transform.translation().x();
        desc.y = transform.translation().y();
        desc.z = transform.translation().z();
        const Eigen::Vector3d angles = transform.linear().eulerAngles(2, 1, 0);
        desc.yaw = angles.x();
        desc.pitch = angles.y();
        desc.roll = angles.z();
        return desc;
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
        constexpr const char* kReproductionPlateStackObjectId =
            "__coating_reproduction_plate_stack__";
    }
    struct CoatingAnalysisModuleController::PendingOnlineDisplay
    {
        std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> result;
        ThicknessUniformityStatistics uniformity;
        OnlinePredictionFrame frame;
    };

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
        , m_onlineJob(std::make_unique<OnlineThicknessPredictionJobController>())
        , m_onlineDiagnostics(std::make_unique<OnlinePredictionDiagnostics>())
        , m_reproductionJob(
              std::make_unique<AlgorithmReproductionJobController>())
        , m_axisymmetricProfile(std::make_unique<AxisymmetricProfileState>())
    {
        m_onlinePoseWatchdog = new QTimer(this);
        m_onlinePoseWatchdog->setInterval(200);
        connect(m_onlinePoseWatchdog, &QTimer::timeout, this, [this]() {
            if(!m_onlineSpraying) {
                return;
            }
            const double now = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if(now - m_liveSampleTimeSeconds <= 0.5) {
                return;
            }
            finishOnlineSpray(false);
            m_status = QStringLiteral(
                "Live RWS pose stream timed out; online accumulation paused.");
            m_onlineStatus = m_status;
            m_panel.setOnlinePredictionState(true, false, m_status, m_onlineFinishing);
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
        });
        m_onlineVirtualTimer = new QTimer(this);
        m_onlineVirtualTimer->setTimerType(Qt::PreciseTimer);
        m_onlineVirtualTimer->setInterval(8);
        connect(m_onlineVirtualTimer, &QTimer::timeout,
            this, &CoatingAnalysisModuleController::advanceVirtualOnlineSpray);
        m_onlineRefreshTimer = new QTimer(this);
        m_onlineRefreshTimer->setInterval(1000);
        connect(m_onlineRefreshTimer, &QTimer::timeout,
            this, &CoatingAnalysisModuleController::updateOnlineRefreshStatistics);
        m_onlineDiagnosticTimer = new QTimer(this);
        m_onlineDiagnosticTimer->setTimerType(Qt::PreciseTimer);
        m_onlineDiagnosticTimer->setInterval(20);
        connect(m_onlineDiagnosticTimer, &QTimer::timeout, this,
            [this]() { m_onlineDiagnostics->heartbeat(); });
        connect(&m_panel, &CoatingAnalysisPanel::openModelRequested,
            this, &CoatingAnalysisModuleController::openModelFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::openTrajectoryRequested,
            this, &CoatingAnalysisModuleController::openTrajectoryFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::loadSavedTrajectoryRequested,
            this, [this]() { loadSelectedSavedTrajectory(true); });
        connect(&m_panel, &CoatingAnalysisPanel::selectModelFileRequested,
            this, &CoatingAnalysisModuleController::selectModelFileFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::selectTrajectoryFileRequested,
            this, &CoatingAnalysisModuleController::selectTrajectoryFileFromDialog);
        connect(&m_panel, &CoatingAnalysisPanel::savedTrajectorySourceChanged,
            this, &CoatingAnalysisModuleController::savedTrajectorySourceChanged);
        connect(&m_panel, &CoatingAnalysisPanel::trajectorySamplingParametersChanged,
            this,
            &CoatingAnalysisModuleController::handleTrajectorySamplingParametersChanged);
        connect(&m_panel, &CoatingAnalysisPanel::trajectorySamplingApplyRequested,
            this, &CoatingAnalysisModuleController::applyTrajectorySampling);
        connect(&m_panel, &CoatingAnalysisPanel::predictionRequested,
            this, &CoatingAnalysisModuleController::predictThickness);
        connect(&m_panel, &CoatingAnalysisPanel::onlineSprayStartRequested,
            this, &CoatingAnalysisModuleController::startOnlineSpray);
        connect(&m_panel, &CoatingAnalysisPanel::onlineSprayStopRequested,
            this, &CoatingAnalysisModuleController::stopOnlineSpray);
        connect(&m_panel, &CoatingAnalysisPanel::onlineGunMovementStartRequested,
            this, [this]() { setOnlineVirtualMotion(true, true); });
        connect(&m_panel, &CoatingAnalysisPanel::onlineGunMovementStopRequested,
            this, [this]() { setOnlineVirtualMotion(true, false); });
        connect(&m_panel, &CoatingAnalysisPanel::onlineRotationStartRequested,
            this, [this]() { setOnlineVirtualMotion(false, true); });
        connect(&m_panel, &CoatingAnalysisPanel::onlineRotationStopRequested,
            this, [this]() { setOnlineVirtualMotion(false, false); });
        connect(&m_panel, &CoatingAnalysisPanel::onlineResetRequested,
            this, &CoatingAnalysisModuleController::resetOnlinePrediction);
        connect(&m_panel, &CoatingAnalysisPanel::onlineToolDirectionsChanged,
            this, &CoatingAnalysisModuleController::applyOnlineToolDirections);
        connect(&m_panel, &CoatingAnalysisPanel::onlineInfluenceDisplayChanged,
            this, [this]() {
                if(!onlineModeActive()) return;
                updateOnlineInfluenceDisplay();
            });
        connect(&m_panel, &CoatingAnalysisPanel::onlineInputChanged,
            this, [this]() {
                if(onlineModeActive() && !m_onlineActive) {
                    m_onlineStatus = QStringLiteral("Ready to start online prediction.");
                    m_status = m_onlineStatus;
                    m_panel.setOnlinePredictionState(false, false, m_status);
                    refreshViewModel();
                } else if(onlineModeActive() && m_onlineVirtualSource) {
                    applyOnlineMotionParameters();
                }
            });
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
        connect(&m_panel, &CoatingAnalysisPanel::enterOnlineRequested,
            this, &CoatingAnalysisModuleController::enterOnline);
        connect(&m_panel, &CoatingAnalysisPanel::exitOnlineRequested,
            this, &CoatingAnalysisModuleController::exitOnline);
        connect(&m_panel,
            &CoatingAnalysisPanel::reproductionSceneParametersChanged,
            this, [this]() {
                if(!reproductionActive() || anyPredictionRunning()) {
                    return;
                }
                invalidateReproductionResult();
                m_reproductionSceneReady = false;
                m_reproductionPlateStack = PlateStackData();
                m_reproductionSceneDetails = QStringLiteral(
                    "Scene settings changed. Generate the scene again.");
                clearReproductionGeneratedPreview();
                applyModelVisibilityOverrides();
                if(generatedReproductionTrajectoryActive()) {
                    m_reproductionTrajectoryReady = false;
                    m_reproductionGeneratedTrajectory =
                        spraytrajectory::SprayTrajectory();
                    m_reproductionTrajectoryDetails = QStringLiteral(
                        "The generated trajectory depends on the scene. "
                        "Generate the trajectory again.");
                    if(RobotQtViewerViewportServices* services =
                            m_context.viewportServices()) {
                        services->setCoatingTrajectoryPreview({}, false);
                    }
                }
                updateReproductionPreparationStatus();
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel,
            &CoatingAnalysisPanel::reproductionTrajectoryParametersChanged,
            this, [this]() {
                if(!reproductionActive() || anyPredictionRunning()) {
                    return;
                }
                invalidateReproductionResult();
                m_reproductionTrajectoryReady = false;
                m_reproductionGeneratedTrajectory =
                    spraytrajectory::SprayTrajectory();
                m_reproductionTrajectoryDetails = QStringLiteral(
                    "Trajectory settings changed. Generate the trajectory again.");
                if(RobotQtViewerViewportServices* services =
                        m_context.viewportServices()) {
                    services->setCoatingTrajectoryPreview({}, false);
                }
                updateReproductionPreparationStatus();
                refreshViewModel();
                publishStateChanged();
            });
        connect(&m_panel, &CoatingAnalysisPanel::reproductionSceneApplyRequested,
            this, [this]() { generateReproductionScene(true); });
        connect(&m_panel,
            &CoatingAnalysisPanel::reproductionTrajectoryApplyRequested,
            this, &CoatingAnalysisModuleController::generateReproductionTrajectory);
        connect(&m_panel,
            &CoatingAnalysisPanel::reproductionRecommendedSetupRequested,
            this, [this]() {
                generateReproductionScene(true);
                if(m_reproductionSceneReady) {
                    generateReproductionTrajectory();
                }
            });
        connect(&m_panel, &CoatingAnalysisPanel::reproductionRequested,
            this, &CoatingAnalysisModuleController::runAlgorithmReproduction);
        connect(&m_panel, &CoatingAnalysisPanel::reproductionBenchmarkRequested,
            this, &CoatingAnalysisModuleController::startReproductionBenchmark);
        connect(&m_panel, &CoatingAnalysisPanel::reproductionInputsChanged,
            this, [this]() {
                if(!reproductionActive() || anyPredictionRunning()) {
                    return;
                }
                invalidateReproductionResult();
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
        connect(&m_panel, &CoatingAnalysisPanel::wuDisplayScaleApplyRequested,
            this, &CoatingAnalysisModuleController::applyWuDisplayScale);

        connect(&m_treePanel, &CoatingAnalysisTreePanel::waypointInfoRequested,
            this, &CoatingAnalysisModuleController::handleWaypointInfoRequested);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::modelVisibilityToggleRequested,
            this, &CoatingAnalysisModuleController::handleModelVisibilityToggleRequested);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::modelSetAsWorkpiece,
            this, &CoatingAnalysisModuleController::handleModelSetAsWorkpiece);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::modelDeleteRequested,
            this, &CoatingAnalysisModuleController::handleModelDeleteRequested);
        connect(&m_treePanel, &CoatingAnalysisTreePanel::trajectoryDeleteRequested,
            this, &CoatingAnalysisModuleController::handleTrajectoryDeleteRequested);
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
        connect(m_onlineJob.get(), &OnlineThicknessPredictionJobController::fieldReady,
            this, &CoatingAnalysisModuleController::handleOnlineField);
        connect(m_onlineJob.get(), &OnlineThicknessPredictionJobController::prepared,
            this, [this](const ViewportSprayInfluenceGeometry& geometry) {
                if(m_onlineInfluencePreview) m_onlineInfluencePreview->geometry = geometry;
                m_onlineBackendReady = true;
                updateOnlineInfluenceDisplay();
                m_onlineIntegrationSampling.setSurfaceDistanceQuery(m_onlineJob->surfaceDistanceQuery());
                if(m_onlineStartAfterPreparation) {
                    startOnlineSpray();
                } else {
                    resumeOnlineVirtualClock();
                }
            });
        connect(m_onlineJob.get(), &OnlineThicknessPredictionJobController::inputCapacityAvailable,
            this, [this](std::uint64_t completedFrameId) {
                if(completedFrameId != m_onlineLastSubmittedFrameId) return;
                m_onlineVirtualBatchPending = false;
                if(m_onlineVirtualSource && onlineModeActive() && !m_onlineFinishing
                    && (m_onlineSpraying || m_onlineVirtualMovingGun || m_onlineVirtualRotating)) {
                    m_onlineVirtualTimer->start(0);
                }
            });
        connect(m_onlineJob.get(), &OnlineThicknessPredictionJobController::predictionFailed,
            this, [this](const QString& error) {
                m_onlineDiagnostics->event(QStringLiteral("FAILED"), 0.0, error);
                resetOnlinePrediction();
                m_onlineStatus = QStringLiteral("Online prediction failed: ") + error;
                if(onlineModeActive()) {
                    m_status = m_onlineStatus;
                    m_panel.setOnlinePredictionState(false, false, m_status);
                    refreshViewModel();
                }
                emit statusMessageRequested(m_onlineStatus, 6000);
            });
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::progressChanged,
            this, &CoatingAnalysisModuleController::handleReproductionProgress);
        connect(m_reproductionJob.get(),
            &AlgorithmReproductionJobController::diagnosticChanged,
            this, &CoatingAnalysisModuleController::handleReproductionDiagnostic);
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

    CoatingAnalysisModuleController::~CoatingAnalysisModuleController()
    {
        m_onlineDiagnostics->finish(QStringLiteral("application shutdown"));
    }

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
        if(onlineModeActive() || anyPredictionRunning() || !m_session.hasResult
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
        if(onlineModeActive() || anyPredictionRunning() || !m_session.hasResult
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
        loadSavedTrajectoryPlanIfAvailable();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            const QString objectId = activeCoatingObjectId();
            if(!objectId.isEmpty()) {
                services->focusCoatingObject(objectId, 0.3);
            }
        }
        applyVisualizationState();
        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                const QString objectId = activeCoatingObjectId();
                if(!services->setSurfaceScalarOverlayVisible(objectId, true)) {
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
                    objectId);
            }
        }
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::deactivate()
    {
        if(onlineModeActive()) {
            exitOnline();
        } else if(simulationActive() && !anyPredictionRunning()) {
            exitSimulation();
        } else if(reproductionActive() && !anyPredictionRunning()) {
            exitReproduction();
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
                services->setSurfaceScalarOverlayVisible(
                    activeCoatingObjectId(), false);
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
        modelAction->setEnabled(!activeCoatingObjectId().isEmpty());
        connect(modelAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowModel);

        QAction* trajectoryAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Trajectory")));
        trajectoryAction->setCheckable(true);
        trajectoryAction->setChecked(m_session.showTrajectory);
        const bool hasGeneratedTrajectory = generatedReproductionTrajectoryActive()
            && m_reproductionTrajectoryReady
            && !m_reproductionGeneratedTrajectory.empty();
        trajectoryAction->setEnabled(
            !m_session.trajectory.empty() || hasGeneratedTrajectory);
        connect(trajectoryAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowTrajectory);

        QAction* sprayPointsAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Spray Points")));
        sprayPointsAction->setCheckable(true);
        sprayPointsAction->setChecked(m_session.showSprayPoints);
        sprayPointsAction->setEnabled(
            !m_session.trajectory.empty() || hasGeneratedTrajectory);
        connect(sprayPointsAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowSprayPoints);

        QAction* thicknessAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Thickness Cloud")));
        thicknessAction->setCheckable(true);
        thicknessAction->setChecked(onlineModeActive()
            ? m_onlineShowThickness : m_session.showThickness);
        thicknessAction->setEnabled(onlineModeActive()
            ? (m_onlineResult && !m_onlineResult->empty()) : m_session.hasResult);
        connect(thicknessAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setShowThickness);

        QAction* thicknessPickAction = visualizationMenu->addAction(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Thickness Pick")));
        thicknessPickAction->setCheckable(true);
        thicknessPickAction->setChecked(onlineModeActive()
            ? m_onlinePickEnabled : m_session.thicknessPickEnabled);
        thicknessPickAction->setEnabled(onlineModeActive()
            ? ((m_onlineResult && !m_onlineResult->empty()) && m_onlineShowThickness)
            : (m_session.hasResult && m_session.showThickness));
        connect(thicknessPickAction, &QAction::toggled,
            this, &CoatingAnalysisModuleController::setThicknessPickEnabled);
    }

    void CoatingAnalysisModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        if(event.kind == RobotQtViewerEventKind::RobotRuntimeChanged
            && event.sourceId == QStringLiteral("digitalTwinContinuousSync")) {
            handleLiveRobotPose(event);
            return;
        }
        if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            resetOnlinePrediction();
            clearSession();
            ensureWorkpieceSelection();
            loadSavedTrajectoryPlanIfAvailable();
        } else if(event.kind == RobotQtViewerEventKind::ViewportReloaded &&
            event.viewport.reloadSucceeded) {
            resetOnlinePrediction();
            // The reload rebuilt the scene; re-apply the coating view mode so
            // robots stay hidden while the coating analysis workbench is active.
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingAnalysisView(m_active);
            }
            applyVisualizationState();
            if(simulationActive() && !anyPredictionRunning()) {
                rebuildSimulation(false);
            } else if(reproductionActive() && !anyPredictionRunning()
                && (m_reproductionSceneReady
                    || m_reproductionTrajectoryReady)) {
                restoreReproductionPreviews(false);
            }
            const QString objectId = activeCoatingObjectId();
            if(m_active && !objectId.isEmpty()) {
                if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                    services->focusCoatingObject(objectId, 0.3);
                }
            }
            applyOverlayAfterReload();
        } else if(event.kind == RobotQtViewerEventKind::ProjectDocumentChanged) {
            if(m_onlineActive) {
                resetOnlinePrediction();
            }
            if(!m_session.objectId.isEmpty()
                && m_session.objectId
                    != QString::fromLatin1(kSimulationPlateObjectId)
                && m_session.objectId
                    != QString::fromLatin1(kReproductionPlateStackObjectId)
                && findObject(m_context.document(), m_session.objectId) == nullptr) {
                clearSession();
            }
            if(m_mode == CoatingAnalysisMode::Prediction) {
                ensureWorkpieceSelection();
                loadSavedTrajectoryPlanIfAvailable(true);
            }
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
        if(onlineModeActive()) {
            const bool valid = m_active && m_onlineShowThickness
                && m_onlinePickEnabled && objectId == m_session.objectId && hit;
            m_hasCurrentThickness = valid;
            if(valid) {
                m_currentThicknessMeters = valueMeters;
                emit thicknessToolTipRequested(
                    QStringLiteral("%1 um").arg(valueMeters * kMetersToMicrometers,
                        0, 'f', 3), viewportPosition, true);
            } else {
                emit thicknessToolTipRequested(QString(), viewportPosition, false);
            }
            refreshViewModel();
            return;
        }
        if(!m_active || !m_session.hasResult || !m_session.showThickness ||
            !m_session.thicknessPickEnabled ||
            objectId != activeCoatingObjectId() || !hit) {
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
                if(reproductionActive()) {
                    m_reproductionSceneReady = false;
                    m_reproductionSceneDetails = QStringLiteral(
                        "Scene model changed. Generate the scene again.");
                    updateReproductionPreparationStatus();
                }
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
        if(m_mode == CoatingAnalysisMode::Prediction) {
            resetReferenceResult();
        }
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
        if(m_mode == CoatingAnalysisMode::Prediction) {
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
        }
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
        if(reproductionActive()) {
            m_reproductionSceneReady = false;
            m_reproductionSceneDetails = QStringLiteral(
                "Scene model changed. Generate the scene again.");
            updateReproductionPreparationStatus();
        }
        m_status = QStringLiteral("Model loaded. Load a trajectory and run thickness prediction.");
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 3000);
        if(onlineModeActive() && model && object && !m_onlineActive) {
            m_onlineJob->prepareModel(model, makeTransform(object->transform));
        }
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

        spraytrajectory::LegacySprayTrajectoryLoadOptions loadOptions;
        loadOptions.disableSprayAcrossTransitionGaps = true;
        const spraytrajectory::LegacySprayTrajectoryLoadResult loadResult =
            spraytrajectory::LegacySprayTrajectoryIo::loadMatrixText(sourcePath, loadOptions);
        if(!loadResult.success) {
            m_status = loadResult.warnings.empty()
                ? QStringLiteral("Failed to load the spray trajectory.")
                : QString::fromStdString(loadResult.warnings.front());
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return false;
        }

        applyLoadedTrajectory(
            loadResult.trajectory,
            path,
            loadResult.warnings.size());
        return true;
    }

    void CoatingAnalysisModuleController::openTrajectoryFromDialog()
    {
        loadTrajectory(kFixedTrajectoryPath);
    }

    void CoatingAnalysisModuleController::savedTrajectorySourceChanged()
    {
        if(m_predictionJob->isRunning()) {
            return;
        }
        clearLoadedTrajectory();
        loadSelectedSavedTrajectory(true);
    }

    bool CoatingAnalysisModuleController::loadSelectedSavedTrajectory(bool reportFailure)
    {
        switch(m_panel.savedTrajectorySource()) {
        case SavedTrajectorySource::Planning:
            return loadSavedTrajectoryPlan(reportFailure);
        case SavedTrajectorySource::DualOptimizationBaseline:
            return loadOptimizationBaselineTrajectory(2, reportFailure);
        case SavedTrajectorySource::DualOptimization:
            return loadOptimizedTrajectory(2, reportFailure);
        case SavedTrajectorySource::ThreeOptimizationBaseline:
            return loadOptimizationBaselineTrajectory(3, reportFailure);
        case SavedTrajectorySource::ThreeOptimization:
            return loadOptimizedTrajectory(3, reportFailure);
        }
        return false;
    }

    bool CoatingAnalysisModuleController::loadSavedTrajectoryPlan(bool reportFailure)
    {
        const auto fail = [this, reportFailure](const QString& message) {
            if(reportFailure) {
                m_status = message;
                refreshViewModel();
                emit statusMessageRequested(m_status, 5000);
            }
            return false;
        };
        if(m_session.objectId.isEmpty()) {
            return fail(QStringLiteral("Select a workpiece before loading its saved trajectory."));
        }

        const simulation_project::ProjectExtensionDesc* extension = nullptr;
        for(const simulation_project::ProjectExtensionDesc& candidate :
            m_context.document().extensions) {
            if(candidate.key !=
                smrobot::spray::rotationbody::kPublishedTrajectoryPlanExtensionKey) {
                continue;
            }
            if(extension != nullptr) {
                return fail(QStringLiteral(
                    "The project contains duplicate saved rotation-body trajectory plans."));
            }
            extension = &candidate;
        }
        if(extension == nullptr) {
            return fail(QStringLiteral("No saved rotation-body trajectory is available for this project."));
        }
        if(extension->version <
                smrobot::spray::rotationbody::kPublishedTrajectoryPlanMinimumSchemaVersion ||
            extension->version >
                smrobot::spray::rotationbody::kPublishedTrajectoryPlanSchemaVersion) {
            return fail(QStringLiteral("The saved rotation-body trajectory uses an unsupported version."));
        }

        std::string parseError;
        const std::optional<smrobot::spray::rotationbody::PublishedTrajectoryPlan> plan =
            smrobot::spray::rotationbody::PublishedTrajectoryPlanStore::read(
                extension->serializedPayload,
                &parseError);
        if(!plan.has_value() || plan->schemaVersion != extension->version) {
            return fail(QStringLiteral("The saved rotation-body trajectory is invalid: %1")
                .arg(QString::fromStdString(parseError)));
        }
        if(QString::fromStdString(plan->objectId) != m_session.objectId) {
            return fail(QStringLiteral(
                "The saved rotation-body trajectory belongs to a different workpiece."));
        }

        std::string conversionError;
        const std::optional<spraytrajectory::SprayTrajectory> trajectory =
            smrobot::spray::rotationbody::PublishedTrajectoryPlanSprayTrajectoryAdapter::convert(
                *plan,
                &conversionError);
        if(!trajectory.has_value()) {
            return fail(QStringLiteral("The saved rotation-body trajectory cannot be loaded: %1")
                .arg(QString::fromStdString(conversionError)));
        }
        applyLoadedTrajectory(*trajectory, kSavedRotationBodyTrajectoryPath);
        return true;
    }

    bool CoatingAnalysisModuleController::loadOptimizationBaselineTrajectory(
        std::size_t trajectoryCount,
        bool reportFailure)
    {
        const auto fail = [this, reportFailure](const QString& message) {
            if(reportFailure) {
                m_status = message;
                refreshViewModel();
                emit statusMessageRequested(m_status, 5000);
            }
            return false;
        };
        if(m_session.objectId.isEmpty()) {
            return fail(QStringLiteral(
                "Select a workpiece before loading its optimization baseline trajectory."));
        }

        const auto record = rotationbodytrajectoryoptimization::OptimizedTrajectoryMemoryStore::read(
            m_session.objectId.toStdString(),
            trajectoryCount == 3
                ? rotationbodytrajectoryoptimization::TrajectoryOptimizationMode::TripleTrajectory
                : rotationbodytrajectoryoptimization::TrajectoryOptimizationMode::DualTrajectory);
        if(!record.has_value() || record->initialTrajectory.empty()) {
            return fail(QStringLiteral(
                "No %1-trajectory optimization baseline is available for this workpiece. Run that optimization mode first.")
                .arg(static_cast<qulonglong>(trajectoryCount)));
        }
        const QString& sourcePath = trajectoryCount == 3
            ? kThreeOptimizationBaselineRotationBodyTrajectoryPath
            : kDualOptimizationBaselineRotationBodyTrajectoryPath;
        applyLoadedTrajectory(
            record->initialTrajectory, sourcePath);
        return true;
    }

    bool CoatingAnalysisModuleController::loadOptimizedTrajectory(
        std::size_t trajectoryCount,
        bool reportFailure)
    {
        const auto fail = [this, reportFailure](const QString& message) {
            if(reportFailure) {
                m_status = message;
                refreshViewModel();
                emit statusMessageRequested(m_status, 5000);
            }
            return false;
        };
        if(m_session.objectId.isEmpty()) {
            return fail(QStringLiteral("Select a workpiece before loading its optimized trajectory."));
        }

        const auto record = rotationbodytrajectoryoptimization::OptimizedTrajectoryMemoryStore::read(
            m_session.objectId.toStdString(),
            trajectoryCount == 3
                ? rotationbodytrajectoryoptimization::TrajectoryOptimizationMode::TripleTrajectory
                : rotationbodytrajectoryoptimization::TrajectoryOptimizationMode::DualTrajectory);
        if(!record.has_value() || record->trajectory.empty()) {
            return fail(QStringLiteral(
                "No selected %1-trajectory optimization candidate is available for this workpiece. Run that optimization mode first.")
                .arg(static_cast<qulonglong>(trajectoryCount)));
        }
        const QString& sourcePath = trajectoryCount == 3
            ? kThreeOptimizedRotationBodyTrajectoryPath
            : kDualOptimizedRotationBodyTrajectoryPath;
        applyLoadedTrajectory(record->trajectory, sourcePath);
        return true;
    }

    void CoatingAnalysisModuleController::loadSavedTrajectoryPlanIfAvailable(
        bool refreshSavedTrajectory)
    {
        if(m_panel.savedTrajectorySource() != SavedTrajectorySource::Planning) {
            return;
        }
        if(m_session.trajectory.empty() ||
            (refreshSavedTrajectory && usesSavedTrajectoryPlan())) {
            loadSavedTrajectoryPlan(false);
        }
    }

    bool CoatingAnalysisModuleController::usesSavedTrajectoryPlan() const
    {
        return m_session.trajectoryPath == kSavedRotationBodyTrajectoryPath;
    }

    bool CoatingAnalysisModuleController::usesOptimizationBaselineTrajectory() const
    {
        return m_session.trajectoryPath == kDualOptimizationBaselineRotationBodyTrajectoryPath
            || m_session.trajectoryPath == kThreeOptimizationBaselineRotationBodyTrajectoryPath;
    }

    bool CoatingAnalysisModuleController::usesOptimizedTrajectory() const
    {
        return m_session.trajectoryPath == kDualOptimizedRotationBodyTrajectoryPath
            || m_session.trajectoryPath == kThreeOptimizedRotationBodyTrajectoryPath;
    }

    void CoatingAnalysisModuleController::clearLoadedTrajectory()
    {
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearCoatingPredictionDebugState();
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            services->setCoatingTrajectoryPreview({}, false);
        }
        m_session.clearResult();
        resetReferenceResult();
        m_session.trajectory = spraytrajectory::SprayTrajectory();
        m_session.waypoints.clear();
        m_session.trajectoryName.clear();
        m_session.trajectoryPath.clear();
        m_session.trajectoryInfo = CoatingAnalysisTrajectoryInfo();
        m_session.clearTrajectorySampling();
        m_treePanel.setWaypoints(nullptr);
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
    }

    void CoatingAnalysisModuleController::applyLoadedTrajectory(
        spraytrajectory::SprayTrajectory trajectory,
        const QString& sourcePath,
        std::size_t warningCount)
    {
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearCoatingPredictionDebugState();
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.clearResult();
        if(m_mode == CoatingAnalysisMode::Prediction) {
            resetReferenceResult();
        }
        m_session.trajectoryName = QString::fromStdString(trajectory.name);
        m_session.trajectoryInfo = makeTrajectoryInfo(trajectory, warningCount);
        m_session.waypoints = trajectory.flattenedPoints();
        m_session.trajectory = std::move(trajectory);
        m_session.trajectoryPath = sourcePath;
        m_session.clearTrajectorySampling();
        if(m_mode == CoatingAnalysisMode::Prediction) {
            m_session.appliedTrajectoryTimeStepSeconds = m_panel.timeStepSeconds();
        }
        m_session.trajectoryControlPointCount = m_session.waypoints.size();
        m_session.trajectoryEffectiveSprayDurationSeconds =
            activeSprayDurationSeconds(m_session.trajectory);
        m_session.trajectorySamplingDirty =
            m_mode == CoatingAnalysisMode::Prediction
            && m_panel.trajectorySamplingMode()
                == spraythickness::TrajectorySamplingMode::ResampleByTimeStep;
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        m_treePanel.setWaypoints(&m_session.waypoints);
        submitTrajectoryPreview();
        if(m_mode == CoatingAnalysisMode::Prediction
            && m_panel.periodicLocalPredictionEnabled()
            && hasEffectiveRotationAxis()) {
            applyModelVisibilityOverrides();
            previewLocalInputs();
        }
        m_hasCurrentThickness = false;
        if(reproductionActive()) {
            m_reproductionTrajectoryReady = false;
            m_reproductionTrajectoryDetails = QStringLiteral(
                "Trajectory data changed. Generate the trajectory again.");
            updateReproductionPreparationStatus();
        }
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_status = reproductionActive()
            ? QStringLiteral("Trajectory loaded. Generate the reproduction trajectory.")
            : (m_session.trajectorySamplingDirty
                ? QStringLiteral(
                    "Trajectory loaded. Click Apply Sampling to build the resampled preview.")
                : QStringLiteral("Trajectory loaded. Ready for GPU thickness prediction."));
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 3000);
    }

    void CoatingAnalysisModuleController::handleTrajectorySamplingParametersChanged()
    {
        if(anyPredictionRunning() || m_mode != CoatingAnalysisMode::Prediction) {
            return;
        }

        if(m_panel.trajectorySamplingMode()
            == spraythickness::TrajectorySamplingMode::OriginalPoints) {
            if(m_session.hasResult
                && m_session.appliedTrajectorySamplingMode
                    != spraythickness::TrajectorySamplingMode::OriginalPoints) {
                if(RobotQtViewerViewportServices* services =
                        m_context.viewportServices()) {
                    services->setSurfaceScalarProbeEnabled(false, QString());
                    services->clearSurfaceScalarOverlay(m_session.objectId);
                }
                m_session.clearResult();
                m_hasCurrentThickness = false;
                emit thicknessToolTipRequested(QString(), QPoint(), false);
            }
            m_session.trajectoryPreviewSamples.clear();
            m_session.appliedTrajectorySamplingMode =
                spraythickness::TrajectorySamplingMode::OriginalPoints;
            m_session.appliedTrajectoryTimeStepSeconds = m_panel.timeStepSeconds();
            m_session.trajectoryControlPointCount = m_session.waypoints.size();
            m_session.trajectoryInterpolatedPointCount = 0;
            m_session.trajectoryEffectiveSprayDurationSeconds =
                activeSprayDurationSeconds(m_session.trajectory);
            m_session.trajectorySamplingDirty = false;
            m_session.trajectorySamplingApplied = false;
            submitTrajectoryPreview();
            m_status = QStringLiteral(
                "Original trajectory points selected for prediction.");
        } else {
            m_session.trajectorySamplingDirty = true;
            m_status = QStringLiteral(
                "Sampling parameters changed. Click Apply Sampling to update the trajectory preview.");
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::applyTrajectorySampling()
    {
        if(anyPredictionRunning() || m_session.trajectory.empty()) {
            return;
        }

        if(m_panel.trajectorySamplingMode()
            != spraythickness::TrajectorySamplingMode::ResampleByTimeStep) {
            handleTrajectorySamplingParametersChanged();
            return;
        }

        const double timeStepSeconds = m_panel.timeStepSeconds();
        std::vector<spraytrajectory::SprayTrajectorySample> samples =
            spraytrajectory::SprayTrajectorySampler::sample(
                m_session.trajectory, timeStepSeconds);
        if(samples.empty()) {
            m_status = QStringLiteral("Failed to resample the trajectory.");
            refreshViewModel();
            emit statusMessageRequested(m_status, 4000);
            return;
        }

        if(m_session.hasResult) {
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setSurfaceScalarProbeEnabled(false, QString());
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
            m_session.clearResult();
            m_hasCurrentThickness = false;
            emit thicknessToolTipRequested(QString(), QPoint(), false);
        }

        m_session.trajectoryPreviewSamples = std::move(samples);
        m_session.appliedTrajectorySamplingMode =
            spraythickness::TrajectorySamplingMode::ResampleByTimeStep;
        m_session.appliedTrajectoryTimeStepSeconds = timeStepSeconds;
        m_session.trajectoryControlPointCount = m_session.waypoints.size();
        m_session.trajectoryInterpolatedPointCount =
            m_session.trajectoryPreviewSamples.size()
                > m_session.trajectoryControlPointCount
            ? m_session.trajectoryPreviewSamples.size()
                - m_session.trajectoryControlPointCount
            : 0;
        m_session.trajectoryEffectiveSprayDurationSeconds =
            activeSprayDurationSeconds(m_session.trajectoryPreviewSamples);
        m_session.trajectorySamplingDirty = false;
        m_session.trajectorySamplingApplied = true;
        m_session.showTrajectory = true;
        m_session.showSprayPoints = true;
        submitTrajectoryPreview();
        m_status = QStringLiteral(
            "Trajectory sampling applied.\n"
            "Control points: %1\nInterpolated points: %2\n"
            "Total samples: %3\nEffective spray duration: %4 s")
            .arg(static_cast<qulonglong>(m_session.trajectoryControlPointCount))
            .arg(static_cast<qulonglong>(m_session.trajectoryInterpolatedPointCount))
            .arg(static_cast<qulonglong>(m_session.trajectoryPreviewSamples.size()))
            .arg(m_session.trajectoryEffectiveSprayDurationSeconds, 0, 'f', 6);
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 4000);
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
        if(onlineModeActive() && m_onlineActive) {
            resetOnlinePrediction();
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

    bool CoatingAnalysisModuleController::prepareOnlineSession()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        const bool virtualSource = m_panel.onlinePoseSource()
            == OnlinePoseSource::Virtual;
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if(services == nullptr || (!virtualSource
            && (m_liveGunRobotId.isEmpty() || m_liveTableRobotId.isEmpty()
                || now - m_liveSampleTimeSeconds > 0.5))) {
            m_onlineStartAfterPreparation = false;
            m_status = virtualSource
                ? QStringLiteral("The online viewport is unavailable.")
                : QStringLiteral("Start the RWS digital twin with complete ROB_1 and STN1 mappings first.");
            m_onlineStatus = m_status;
            m_panel.setOnlinePredictionState(m_onlineActive, false, m_status);
            emit statusMessageRequested(m_status, 5000);
            return false;
        }
        if(m_onlineActive && (m_onlineVirtualSource != virtualSource
            || (!virtualSource && (m_liveGunRobotId != m_onlineGunRobotId
                || m_liveTableRobotId != m_onlineTableRobotId)))) {
            resetOnlinePrediction();
        }
        if(!m_onlineActive) {
            const simulation_project::SceneObjectDesc* selected =
                findObject(m_context.document(), m_session.objectId);
            if(selected == nullptr || selected->objectType != "workpiece") {
                m_status = QStringLiteral("Select a workpiece for online prediction.");
                m_onlineStatus = m_status;
                m_panel.setOnlinePredictionState(false, false, m_status);
                m_onlineDiagnostics->finish(QStringLiteral("no selected workpiece"));
                m_onlineDiagnosticTimer->stop();
                m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
                return false;
            }
            try {
                spraythickness::ThicknessPredictionTask task;
                task.model = spraythickness::ThicknessModelKind::PaperGaussian;
                task.workpiece.name = virtualSource
                    ? "Virtual online workpiece" : "Online rotating workpiece and fixtures";
                task.tool.name = virtualSource
                    ? "Virtual spray gun" : "Live RWS spray gun";
                task.tool.sprayDirectionLocal = m_panel.onlineSprayDirectionLocal();
                task.tool.powderFeedDirectionLocal = m_panel.onlinePowderFeedDirectionLocal();
                m_onlineTool = task.tool;
                m_onlineDisplayedTool = task.tool;
                task.process.id = "online";
                task.process.name = "online";
                task.options.base.trajectorySamplingMode =
                    spraythickness::TrajectorySamplingMode::OriginalPoints;
                task.options.enableBvhOcclusion = m_panel.onlineBvhOcclusionEnabled();
                task.options.enableHistoryCorrection =
                    m_panel.onlineHistoryCorrectionEnabled();
                m_onlineInitialTablePose = virtualSource
                    ? Eigen::Isometry3d::Identity() : m_liveTablePose;
                std::vector<OnlineObject> objects;
                for(const auto& object : m_context.document().objects) {
                    if(object.id != selected->id
                        && (virtualSource || object.objectType != "fixture")) {
                        continue;
                    }
                    const std::filesystem::path sourcePath =
                        simulation_project::AssetResolver::resolveProjectPath(
                            makeResolveContext(m_context.projectSession()),
                            object.sourcePath);
                    std::string loadError;
                    const auto model = assetcore::AssetManager::instance().tryLoadModel(
                        sourcePath.generic_u8string(),
                        static_cast<float>(object.visualScale), &loadError);
                    if(!model) {
                        throw std::runtime_error(loadError.empty()
                            ? "Failed to load an online workpiece or fixture mesh."
                            : loadError);
                    }
                    OnlineObject onlineObject;
                    onlineObject.id = QString::fromStdString(object.id);
                    onlineObject.worldFromObject = makeTransform(object.transform);
                    PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
                        *model, object.name, sourcePath.generic_u8string(),
                        m_onlineInitialTablePose.inverse()
                            * onlineObject.worldFromObject);
                    const std::size_t offset = task.workpiece.samples.size();
                    for(auto& subMesh : mesh.binding.sampleIndicesBySubMesh) {
                        for(std::size_t& index : subMesh) {
                            index += offset;
                        }
                    }
                    onlineObject.binding = std::move(mesh.binding);
                    task.workpiece.samples.insert(task.workpiece.samples.end(),
                        mesh.workpiece.samples.begin(), mesh.workpiece.samples.end());
                    for(std::uint32_t index : mesh.workpiece.triangleIndices) {
                        task.workpiece.triangleIndices.push_back(
                            static_cast<std::uint32_t>(offset + index));
                    }
                    objects.push_back(std::move(onlineObject));
                }
                if(task.workpiece.samples.empty()
                    || task.workpiece.triangleIndices.empty()) {
                    throw std::runtime_error(
                        "The online workpiece and fixtures have no triangle mesh.");
                }
                Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::max());
                Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::lowest());
                for(const auto& sample : task.workpiece.samples) {
                    minimum = minimum.cwiseMin(sample.position);
                    maximum = maximum.cwiseMax(sample.position);
                }
                m_onlineVirtualCenter = 0.5 * (minimum + maximum);
                m_onlineModelRadiusMeters = 0.5 * (maximum - minimum).norm();
                m_onlineInfluencePreview = std::make_unique<ViewportSprayInfluencePreview>();
                const auto& deposition = task.options.deposition;
                m_onlineInfluencePreview->angularPattern = Eigen::Vector4f(
                    static_cast<float>(deposition.sigmaPhiRadians), static_cast<float>(deposition.sigmaPsiRadians),
                    static_cast<float>(deposition.phiOffsetRadians), static_cast<float>(deposition.psiOffsetRadians));
                m_onlineInfluencePreview->patternRotation = static_cast<float>(deposition.rotationRadians);
                m_onlineInfluencePreview->shadowBiasMillimeters = static_cast<float>(task.options.shadowBiasMeters * 1000.0);
                m_onlineInfluencePreview->occlusionEnabled = task.options.enableBvhOcclusion;
                for(const auto& object : objects) m_onlineInfluencePreview->objectIds.push_back(object.id);
                m_onlineIntegrationSampling.configure(minimum, maximum,
                    std::min(task.options.deposition.sigmaPhiRadians,
                        task.options.deposition.sigmaPsiRadians));
                m_onlineVirtualSource = virtualSource;
                m_onlineVirtualRotating = false;
                m_onlineVirtualMovingGun = false;
                m_onlineMotionClock.reset();
                m_panel.setOnlineMotionState(false, false);
                m_onlineVirtualAxis = m_panel.onlineRotationAxis();
                m_onlineConfiguredRotationAxis = m_onlineVirtualAxis;
                m_onlineRotationBasePose = Eigen::Isometry3d::Identity();
                m_onlineRotationReferenceSeconds = 0.0;
                m_onlineVirtualRpm = m_panel.onlineRotationRpm();
                m_onlineVirtualRandomAxis = virtualSource
                    && m_panel.onlineRandomRotationAxisEnabled();
                if(m_onlineVirtualRandomAxis) {
                    m_onlineRandomRotation.reset(m_onlineVirtualCenter,
                        m_onlineVirtualRpm, QRandomGenerator::global()->generate());
                    m_onlineVirtualAxis = m_onlineRandomRotation.axis();
                    LOG_INFO("rs2026") << "Online random rotation: seed="
                        << m_onlineRandomRotation.seed() << ", rpm=" << m_onlineVirtualRpm
                        << ", center=" << m_onlineVirtualCenter.transpose();
                }
                m_onlineDisplayedRotationAxis = m_onlineVirtualAxis;
                const Eigen::Vector3d gunStart = m_panel.onlineGunStartOffsetMeters();
                const Eigen::Vector3d gunEnd = m_panel.onlineGunEndOffsetMeters();
                const double gunSpeed = m_panel.onlineGunSpeedMetersPerSecond();
                m_onlineGunMotion.reset(m_onlineVirtualCenter + gunStart,
                    m_onlineVirtualCenter + gunEnd, gunSpeed);
                m_onlineMotionParametersPending = false;
                m_onlineVirtualTimeSeconds = 0.0;
                m_onlineCurrentTablePose = virtualSource
                    ? Eigen::Isometry3d::Identity() : m_liveTablePose;
                m_onlineCurrentGunPose = virtualSource
                    ? onlineVirtualGunPose(m_onlineVirtualCenter,
                        gunStart, gunEnd, gunSpeed, 0.0) : m_liveGunPose;
                m_onlineDisplayedTablePose = m_onlineCurrentTablePose;
                m_onlineDisplayedGunPose = m_onlineCurrentGunPose;
                m_onlineObjects = std::move(objects);
                m_onlineGunRobotId = m_liveGunRobotId;
                m_onlineTableRobotId = m_liveTableRobotId;
                m_onlineStartTimeSeconds = now;
                m_onlineActive = true;
                m_onlineShowThickness = true;
                m_onlinePickEnabled = false;
                std::vector<OnlinePredictionObjectBinding> displayBindings;
                displayBindings.reserve(m_onlineObjects.size());
                for(OnlineObject& object : m_onlineObjects) {
                    displayBindings.push_back({ object.id.toStdString(), std::move(object.binding) });
                }
                m_onlineDisplayOverlays.reset();
                m_onlineGpuDisplayFields.reset();
                m_onlineJob->begin(std::move(task), std::move(displayBindings));
                m_onlineBackendReady = false;
                services->setCoatingAnalysisView(true);
                applyModelVisibilityOverrides();
                services->setSurfaceScalarProbeEnabled(false, QString());
                services->setCoatingTrajectoryPreview({}, false);
                for(const OnlineObject& object : m_onlineObjects) {
                    services->clearSurfaceScalarOverlay(object.id);
                }
                if(m_session.predictionDisplayModel) {
                    services->clearCoatingPredictionModel(m_session.objectId);
                }
            } catch(const std::exception& exception) {
                m_onlineDiagnostics->event(QStringLiteral("FAILED"), 0.0,
                    QString::fromLocal8Bit(exception.what()));
                resetOnlinePrediction();
                m_status = QString::fromLocal8Bit(exception.what());
                m_onlineStatus = m_status;
                m_panel.setOnlinePredictionState(false, false, m_status);
                emit statusMessageRequested(m_status, 5000);
                return false;
            }
        }
        return true;
    }

    void CoatingAnalysisModuleController::setOnlineVirtualMotion(bool gunMovement, bool running)
    {
        if(!onlineModeActive() || m_panel.onlinePoseSource() != OnlinePoseSource::Virtual) return;
        if(running && (anyPredictionRunning() || m_onlineFinishing || !prepareOnlineSession())) return;
        if(!m_onlineActive) return;
        // Change motion at the last integrated boundary, not at wall time. This
        // keeps the submitted interval intact and makes high-RPM stop immediate.
        if(gunMovement) m_onlineVirtualMovingGun = running;
        else m_onlineVirtualRotating = running;
        m_panel.setOnlineMotionState(m_onlineVirtualMovingGun, m_onlineVirtualRotating);
        resumeOnlineVirtualClock();
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::applyOnlineToolDirections()
    {
        if(!m_onlineActive) return;
        const Eigen::Vector3d spray = m_panel.onlineSprayDirectionLocal();
        const Eigen::Vector3d powder = m_panel.onlinePowderFeedDirectionLocal();
        if(spray.isApprox(m_onlineTool.sprayDirectionLocal)
            && powder.isApprox(m_onlineTool.powderFeedDirectionLocal)) return;
        // Complete any collected old-direction intervals before changing the
        // input snapshot. Already queued batches keep their own directions.
        flushOnlineTrajectory();
        m_onlineTool.sprayDirectionLocal = spray;
        m_onlineTool.powderFeedDirectionLocal = powder;
        if(!m_onlineSpraying && !m_onlineFinishing
            && m_onlineLastSubmittedFrameId <= m_onlineLastAcknowledgedFrameId) {
            m_onlineDisplayedTool = m_onlineTool;
            updateOnlinePoseDisplay();
        }
    }

    void CoatingAnalysisModuleController::applyOnlineMotionParameters()
    {
        if(!m_onlineActive || !m_onlineVirtualSource) return;
        if(m_onlineFinishing) {
            m_onlineMotionParametersPending = true;
            return;
        }
        m_onlineMotionParametersPending = false;
        const Eigen::Vector3d axis = m_panel.onlineRotationAxis();
        const double rpm = m_panel.onlineRotationRpm();
        const bool randomAxis = m_panel.onlineRandomRotationAxisEnabled();
        const bool rotationChanged = !axis.isApprox(m_onlineConfiguredRotationAxis)
            || rpm != m_onlineVirtualRpm || randomAxis != m_onlineVirtualRandomAxis;
        if(rotationChanged) {
            // Apply the new angular velocity to the current orientation, never
            // retroactively to all elapsed time (which would jump the model).
            m_onlineRotationBasePose = m_onlineCurrentTablePose;
            m_onlineRotationReferenceSeconds = m_onlineMotionClock.rotationSeconds();
            if(randomAxis) {
                const auto seed = m_onlineVirtualRandomAxis ? m_onlineRandomRotation.seed()
                    : QRandomGenerator::global()->generate();
                m_onlineRandomRotation.reset(m_onlineVirtualCenter, rpm, seed, m_onlineVirtualAxis);
                m_onlineVirtualAxis = m_onlineRandomRotation.axis();
            } else {
                m_onlineVirtualAxis = axis;
            }
            m_onlineConfiguredRotationAxis = axis;
            m_onlineVirtualRpm = rpm;
            m_onlineVirtualRandomAxis = randomAxis;
        }
        const Eigen::Vector3d start = m_onlineVirtualCenter + m_panel.onlineGunStartOffsetMeters();
        const Eigen::Vector3d end = m_onlineVirtualCenter + m_panel.onlineGunEndOffsetMeters();
        const double speed = m_panel.onlineGunSpeedMetersPerSecond();
        const bool startChanged = !start.isApprox(m_onlineGunMotion.startPosition(), 1.0e-12);
        const bool gunChanged = startChanged || !end.isApprox(m_onlineGunMotion.endPosition(), 1.0e-12)
            || speed != m_onlineGunMotion.speedMetersPerSecond();
        if(!rotationChanged && !gunChanged) return;
        if(startChanged) {
            // Editing coordinates explicitly repositions the gun. The next
            // interval starts at the new pose; the jump has no time contribution.
            m_onlineGunMotion.reset(start, end, speed, m_onlineMotionClock.gunSeconds());
        } else if(gunChanged) {
            m_onlineGunMotion.reconfigure(m_onlineMotionClock.gunSeconds(), start, end, speed);
        }
        updateOnlineVirtualPoses(m_onlineVirtualTimeSeconds);
        if(m_onlineSpraying && !m_onlinePendingPoints.empty()) {
            m_onlinePendingPoints.back().tcpPose = m_onlineCurrentTablePose.inverse() * m_onlineCurrentGunPose;
        }
        if(!m_onlineSpraying) {
            m_onlineDisplayedTablePose = m_onlineCurrentTablePose;
            m_onlineDisplayedGunPose = m_onlineCurrentGunPose;
            m_onlineDisplayedRotationAxis = m_onlineVirtualAxis;
            updateOnlinePoseDisplay();
        }
        resumeOnlineVirtualClock();
    }

    void CoatingAnalysisModuleController::resumeOnlineVirtualClock()
    {
        if(!m_onlineActive || !m_onlineVirtualSource || !onlineModeActive() || m_onlineFinishing) return;
        m_onlineVirtualTimer->stop();
        m_onlineVirtualRunBaseSeconds = m_onlineVirtualTimeSeconds;
        m_onlineVirtualRunStartedAt = std::chrono::steady_clock::now();
        if(!m_onlineFinishing) m_onlineVirtualStopping = false;
        m_onlineRefreshCadence.setFramesPerSecondLimit(m_panel.onlineFramesPerSecondLimit());
        if(!m_onlineSpraying && !m_onlineStartAfterPreparation && !m_onlineFinishing) {
            m_onlineStatus = m_onlineVirtualMovingGun || m_onlineVirtualRotating
                ? QStringLiteral("Virtual motion active; spraying is stopped.")
                : QStringLiteral("Virtual motion paused; current poses retained.");
            if(m_onlineLastSubmittedFrameId != 0) m_onlineStatus = onlineStoppedStatus();
            if(!m_onlineBackendReady) m_onlineStatus = QStringLiteral("Preparing online model and GPU resources...");
            m_status = m_onlineStatus;
            m_panel.setOnlinePredictionState(true, false, m_onlineStatus);
        }
        if(m_onlineBackendReady && !m_onlineFinishing && !m_onlineVirtualBatchPending
            && (m_onlineSpraying || m_onlineVirtualMovingGun || m_onlineVirtualRotating)) {
            m_onlineVirtualTimer->setSingleShot(true);
            m_onlineVirtualTimer->start(0);
        }
        if(!m_onlineResult) {
            m_onlineDisplayedTablePose = m_onlineCurrentTablePose;
            m_onlineDisplayedGunPose = m_onlineCurrentGunPose;
            updateOnlinePoseDisplay();
        }
    }

    void CoatingAnalysisModuleController::updateOnlineVirtualPoses(double timeSeconds)
    {
        m_onlineMotionClock.advanceTo(timeSeconds, m_onlineVirtualRotating, m_onlineVirtualMovingGun);
        const double rotationTime = m_onlineMotionClock.rotationSeconds() - m_onlineRotationReferenceSeconds;
        if(m_onlineVirtualRandomAxis) {
            m_onlineCurrentTablePose = m_onlineRandomRotation.poseAt(rotationTime) * m_onlineRotationBasePose;
            m_onlineVirtualAxis = m_onlineRandomRotation.axis();
        } else {
            m_onlineCurrentTablePose = onlineRotatingWorkpiecePose(m_onlineVirtualCenter,
                m_onlineVirtualAxis, m_onlineVirtualRpm, rotationTime) * m_onlineRotationBasePose;
        }
        m_onlineCurrentGunPose.translation() = m_onlineGunMotion.positionAt(m_onlineMotionClock.gunSeconds());
        m_onlineVirtualTimeSeconds = m_onlineLastPoseTimeSeconds = timeSeconds;
    }

    void CoatingAnalysisModuleController::startOnlineSpray()
    {
        const auto guiPreparationStartedAt = std::chrono::steady_clock::now();
        if(!onlineModeActive() || anyPredictionRunning()
            || m_onlineSpraying || m_onlineFinishing) return;
        const bool virtualSource = m_panel.onlinePoseSource() == OnlinePoseSource::Virtual;
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if(!m_onlineStartAfterPreparation) {
            m_onlineSprayRequestedAt = std::chrono::steady_clock::now();
            m_onlineDiagnosticStartedAt = m_onlineSprayRequestedAt;
            m_onlineLastInputSubmittedAt = {};
            m_onlineGuiStartupMilliseconds = 0.0;
            m_onlineDiagnosticFrameEligible = false;
            m_onlineDiagnostics->begin(QStringLiteral("source=%1; occlusion=%2; thermal_history=%3; screen_hz=%4; fps_limit=%5")
                .arg(virtualSource ? QStringLiteral("virtual") : QStringLiteral("RWS"))
                .arg(m_panel.onlineBvhOcclusionEnabled()).arg(m_panel.onlineHistoryCorrectionEnabled())
                .arg(m_onlineScreenRefreshRate).arg(m_panel.onlineFramesPerSecondLimit()));
            m_onlineDiagnosticTimer->start();
            m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
        }
        if(!prepareOnlineSession()) return;
        if(!m_onlineBackendReady) {
            // Do not accumulate seconds of virtual motion while shaders/BVH are
            // still initializing. Start the physical clock only when ready.
            m_onlineStartAfterPreparation = true;
            m_onlineGuiStartupMilliseconds += elapsedMilliseconds(guiPreparationStartedAt);
            m_panel.setOnlinePredictionState(true, true,
                QStringLiteral("Preparing online model and GPU resources..."));
            return;
        }
        m_onlineVirtualStopping = false;
        m_onlineUnsimulatedSeconds = 0.0;
        m_onlineStartAfterPreparation = false;
        m_onlineVirtualBatchPending = false;
        m_onlineLastCompletedFrames = m_onlineJob->diagnosticState().completedFrames;
        m_onlineFirstFrameMilliseconds = -1.0;
        m_onlineWaitingForFirstFrame = true;
        m_onlineThicknessFramePending = false;
        m_onlineRefreshStartedAt = std::chrono::steady_clock::now();
        m_onlineSceneFrameCount = 0;
        m_onlineViewportFrameCount = 0;
        m_onlineThicknessFrameCount = 0;
        m_onlineLastPresentedAt = {};
        m_onlineNextDisplayAt = {};
        m_onlineFrameIntervals.clear();
        m_onlineMaximumFrameIntervalMilliseconds = -1.0;
        m_onlineLastLongFrameLog = {};
        updateOnlineScreenRefreshRate();
        m_panel.setOnlineRefreshStatistics(0.0, 0.0, -1.0, true,
            0.0, -1.0, -1.0, m_onlineScreenRefreshRate);
        m_onlineLastRenderMilliseconds = 0.0;
        m_onlineSpraying = true;
        if(!virtualSource) {
            m_onlineCurrentTablePose = m_liveTablePose;
            m_onlineCurrentGunPose = m_liveGunPose;
        }
        if(virtualSource) {
            resumeOnlineVirtualClock();
        } else {
            m_onlinePoseWatchdog->start();
        }
        m_onlinePendingPoints.clear();
        m_onlineLastPoseTimeSeconds = virtualSource
            ? m_onlineVirtualTimeSeconds : now;
        spraytrajectory::SprayPathPoint point;
        point.time = virtualSource ? m_onlineVirtualTimeSeconds
            : now - m_onlineStartTimeSeconds;
        point.tcpPose = m_onlineCurrentTablePose.inverse()
            * m_onlineCurrentGunPose;
        point.sprayEnabled = true;
        point.processId = "online";
        m_onlinePendingPoints.push_back(std::move(point));
        if(virtualSource) {
            const double integrationStep = m_onlineIntegrationSampling.timeStepSeconds(
                m_onlinePendingPoints.back().tcpPose.translation(),
                m_onlineVirtualRotating ? m_onlineVirtualRpm : 0.0,
                m_onlineVirtualMovingGun ? m_onlineGunMotion.speedMetersPerSecond() : 0.0);
            m_onlineDiagnostics->event(QStringLiteral("INTEGRATION_SAMPLING"), 0.0,
                QStringLiteral("adaptive_motion=1; initial_max_step_ms=%1; rpm=%2; gun_speed_m_s=%3; random_axis=%4")
                    .arg(integrationStep * 1000.0, 0, 'g', 9)
                    .arg(m_onlineVirtualRotating ? m_onlineVirtualRpm : 0.0)
                    .arg(m_onlineVirtualMovingGun ? m_onlineGunMotion.speedMetersPerSecond() : 0.0)
                    .arg(m_onlineVirtualRandomAxis));
        }
        if(!m_onlineResult) {
            m_onlineDisplayedTablePose = m_onlineCurrentTablePose;
            m_onlineDisplayedGunPose = m_onlineCurrentGunPose;
            updateOnlinePoseDisplay();
        }
        m_status = QStringLiteral("Online accumulation active.");
        m_onlineStatus = m_status;
        m_panel.setOnlinePredictionState(true, true,
            m_status);
        if(m_onlineVirtualRandomAxis) {
            m_panel.setOnlineRandomRotationState(m_onlineDisplayedRotationAxis,
                m_onlineRandomRotation.seed());
        }
        refreshViewModel();
        m_onlineGuiStartupMilliseconds += elapsedMilliseconds(guiPreparationStartedAt);
        m_onlineDiagnostics->event(QStringLiteral("GUI_PREPARATION"), m_onlineGuiStartupMilliseconds);
    }

    void CoatingAnalysisModuleController::stopOnlineSpray()
    {
        if(m_onlineStartAfterPreparation) {
            m_onlineStartAfterPreparation = false;
            m_panel.setOnlinePredictionState(m_onlineActive, false,
                QStringLiteral("Online accumulation paused."));
            m_onlineDiagnostics->finish(QStringLiteral("start canceled during preparation"));
            m_onlineDiagnosticTimer->stop();
            resumeOnlineVirtualClock();
            return;
        }
        m_onlineStartAfterPreparation = false;
        finishOnlineSpray(!m_onlineVirtualSource);
    }

    QString CoatingAnalysisModuleController::onlineStoppedStatus() const
    {
        QString status = m_onlineFinishing
            ? QStringLiteral("Spraying stopped; finalizing submitted results.")
            : (m_onlineVirtualSource && (m_onlineVirtualMovingGun || m_onlineVirtualRotating)
                ? QStringLiteral("Virtual motion active; spraying is stopped.")
                : QStringLiteral("Online accumulation paused."));
        if(m_onlineVirtualSource) {
            status += QStringLiteral("\nSimulated time: %1 s\nUnsimulated wall time: %2 s")
                .arg(m_onlineDisplayedTimeSeconds, 0, 'f', 3)
                .arg(m_onlineUnsimulatedSeconds, 0, 'f', 3);
        }
        return status;
    }

    void CoatingAnalysisModuleController::finishOnlineSpray(
        bool includeStopSample)
    {
        if(!m_onlineSpraying) {
            return;
        }
        m_onlinePoseWatchdog->stop();
        m_onlineVirtualTimer->stop();
        if(m_onlineVirtualSource) {
            const double requestedTime = m_onlineVirtualRunBaseSeconds + std::chrono::duration<double>(
                std::chrono::steady_clock::now() - m_onlineVirtualRunStartedAt).count();
            // Stop at the submitted integration boundary. Never create more
            // physical samples merely to catch up with elapsed wall time.
            m_onlineVirtualStopping = true;
            m_onlineVirtualStopTime = m_onlineLastPoseTimeSeconds;
            m_onlineVirtualTimeSeconds = m_onlineVirtualStopTime;
            m_onlineUnsimulatedSeconds = std::max(0.0, requestedTime - m_onlineVirtualStopTime);
        }
        m_onlineDiagnostics->event(QStringLiteral("STOP_REQUESTED"), 0.0,
            QStringLiteral("submitted_time_s=%1; unsimulated_wall_time_s=%2")
                .arg(m_onlineVirtualSource ? m_onlineVirtualStopTime
                    : m_onlineLastPoseTimeSeconds - m_onlineStartTimeSeconds, 0, 'g', 12)
                .arg(m_onlineUnsimulatedSeconds, 0, 'g', 12));
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if(includeStopSample && !m_onlinePendingPoints.empty()
            && now - m_onlineLastPoseTimeSeconds <= 0.5) {
            spraytrajectory::SprayPathPoint point = m_onlinePendingPoints.back();
            point.time = now - m_onlineStartTimeSeconds;
            m_onlinePendingPoints.push_back(std::move(point));
        }
        flushOnlineTrajectory(true);
        m_onlinePendingPoints.clear();
        m_onlineSpraying = false;
        m_onlineFinishing = m_onlineLastSubmittedFrameId > m_onlineLastAcknowledgedFrameId;
        if(!m_onlineFinishing) m_onlineWaitingForFirstFrame = false;
        m_status = onlineStoppedStatus();
        m_onlineStatus = m_status;
        m_panel.setOnlinePredictionState(true, false,
            m_status, m_onlineFinishing);
        refreshViewModel();
        // A manual display cap must not defer the final stopped frame.
        applyPendingOnlineField();
        if(m_onlineLastSubmittedFrameId == 0
            || m_onlineLastAcknowledgedFrameId >= m_onlineLastSubmittedFrameId) {
            m_onlineDiagnostics->finish(QStringLiteral("spray stopped"));
            m_onlineDiagnosticTimer->stop();
            m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
        }
        if(!m_onlineFinishing) resumeOnlineVirtualClock();
    }

    void CoatingAnalysisModuleController::flushOnlineTrajectory(bool finalInterval)
    {
        const bool hasInterval = m_onlinePendingPoints.size() >= 2;
        if(!hasInterval && (!finalInterval || m_onlineLastSubmittedFrameId == 0)) {
            return;
        }
        spraytrajectory::SprayTrajectory trajectory;
        if(hasInterval) {
            spraytrajectory::SpraySegment segment;
            segment.processId = "online";
            segment.sprayEnabled = true;
            segment.points = m_onlinePendingPoints;
            trajectory.segments.push_back(std::move(segment));
        }
        OnlinePredictionFrame frame;
        frame.submittedAt = std::chrono::steady_clock::now();
        frame.inputIntervalMilliseconds = m_onlineLastInputSubmittedAt == std::chrono::steady_clock::time_point{}
            ? 0.0 : std::chrono::duration<double, std::milli>(frame.submittedAt - m_onlineLastInputSubmittedAt).count();
        m_onlineLastInputSubmittedAt = frame.submittedAt;
        frame.timeSeconds = m_onlineVirtualSource ? m_onlineLastPoseTimeSeconds
            : m_onlineLastPoseTimeSeconds - m_onlineStartTimeSeconds;
        if(hasInterval) {
            const auto& points = trajectory.segments.front().points;
            frame.timeSeconds = points.back().time;
            frame.integratedMilliseconds = (points.back().time - points.front().time) * 1000.0;
        }
        // An empty final trajectory refreshes full statistics and the display
        // at the existing time, without adding a synthetic deposition interval.
        frame.finalInterval = finalInterval;
        frame.tablePose = m_onlineCurrentTablePose;
        frame.gunPose = m_onlineCurrentGunPose;
        frame.rotationAxis = m_onlineVirtualAxis;
        frame.tool = m_onlineTool;
        m_onlineLastSubmittedFrameId = m_onlineJob->append(std::move(trajectory), std::move(frame));
        if(m_onlineVirtualSource) m_onlineVirtualBatchPending = m_onlineLastSubmittedFrameId != 0;
        if(hasInterval) {
            spraytrajectory::SprayPathPoint last = m_onlinePendingPoints.back();
            m_onlinePendingPoints.clear();
            m_onlinePendingPoints.push_back(std::move(last));
        }
    }

    void CoatingAnalysisModuleController::handleLiveRobotPose(
        const RobotQtViewerEvent& event)
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || event.liveSprayRobotId.isEmpty()
            || event.liveTurntableRobotId.isEmpty()) {
            return;
        }
        Eigen::Isometry3d gun = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d table = Eigen::Isometry3d::Identity();
        if(!services->robotEndEffectorTransform(event.liveSprayRobotId, gun)
            || !services->robotEndEffectorTransform(event.liveTurntableRobotId, table)) {
            return;
        }
        m_liveGunPose = gun;
        m_liveTablePose = table;
        m_liveGunRobotId = event.liveSprayRobotId;
        m_liveTableRobotId = event.liveTurntableRobotId;
        m_liveSampleTimeSeconds = event.liveSampleTimeSeconds;
        if(!m_onlineActive || m_onlineVirtualSource) {
            return;
        }
        if(event.liveSprayRobotId != m_onlineGunRobotId
            || event.liveTurntableRobotId != m_onlineTableRobotId) {
            resetOnlinePrediction();
            return;
        }
        if(!m_onlineSpraying) {
            m_onlineCurrentTablePose = table;
            m_onlineCurrentGunPose = gun;
            if(!m_onlineFinishing) {
                m_onlineDisplayedTablePose = table;
                m_onlineDisplayedGunPose = gun;
                updateOnlinePoseDisplay();
            }
            return;
        }
        if(event.liveSampleTimeSeconds <= m_onlineLastPoseTimeSeconds) {
            return;
        }
        if(event.liveSampleTimeSeconds - m_onlineLastPoseTimeSeconds > 0.5) {
            flushOnlineTrajectory();
            m_onlinePendingPoints.clear();
        }
        m_onlineCurrentTablePose = table;
        m_onlineCurrentGunPose = gun;
        spraytrajectory::SprayPathPoint point;
        point.time = event.liveSampleTimeSeconds - m_onlineStartTimeSeconds;
        point.tcpPose = table.inverse() * gun;
        point.sprayEnabled = true;
        point.processId = "online";
        m_onlinePendingPoints.push_back(std::move(point));
        m_onlineLastPoseTimeSeconds = event.liveSampleTimeSeconds;
        flushOnlineTrajectory();
    }

    void CoatingAnalysisModuleController::advanceVirtualOnlineSpray()
    {
        if(!m_onlineActive || !onlineModeActive() || !m_onlineVirtualSource
            || !m_onlineBackendReady || m_onlineFinishing || m_onlineVirtualBatchPending
            || (!m_onlineSpraying && !m_onlineVirtualMovingGun && !m_onlineVirtualRotating)) {
            return;
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - m_onlineVirtualRunStartedAt).count();
        const double targetTime = m_onlineVirtualRunBaseSeconds + elapsed;
        if(!m_onlineSpraying) {
            // Powder-off motion changes transforms only. Keep the resident GPU
            // thickness field, and never connect samples across a powder-off gap.
            updateOnlineVirtualPoses(targetTime);
            m_onlineDisplayedTablePose = m_onlineCurrentTablePose;
            m_onlineDisplayedGunPose = m_onlineCurrentGunPose;
            m_onlineDisplayedRotationAxis = m_onlineVirtualAxis;
            m_onlineDisplayedTimeSeconds = targetTime;
            updateOnlinePoseDisplay();
            m_onlineRefreshCadence.setFramesPerSecondLimit(m_panel.onlineFramesPerSecondLimit());
            m_onlineVirtualTimer->start(static_cast<int>(
                std::ceil(m_onlineRefreshCadence.displayIntervalMilliseconds())));
            return;
        }
        const auto intervalBudget = m_onlineJob->integrationIntervalBudget();
        std::size_t intervals = 0;
        while(targetTime - m_onlineLastPoseTimeSeconds > 1.0e-9) {
            if(intervals++ >= intervalBudget) break;
            const Eigen::Vector3d relativeGunPosition = m_onlineCurrentTablePose.inverse()
                * m_onlineCurrentGunPose.translation();
            const double integrationStep = m_onlineIntegrationSampling.timeStepSeconds(
                relativeGunPosition, m_onlineVirtualRotating ? m_onlineVirtualRpm : 0.0,
                m_onlineVirtualMovingGun ? m_onlineGunMotion.speedMetersPerSecond() : 0.0);
            const double nextTurnTime = m_onlineVirtualMovingGun
                ? m_onlineVirtualTimeSeconds + m_onlineGunMotion.nextTurnTimeSeconds(m_onlineMotionClock.gunSeconds())
                    - m_onlineMotionClock.gunSeconds()
                : std::numeric_limits<double>::infinity();
            const double intervalTarget = std::min(targetTime,
                nextTurnTime);
            const double time = onlineNextSampleTimeSeconds(
                m_onlineLastPoseTimeSeconds, intervalTarget, integrationStep);
            updateOnlineVirtualPoses(time);
            spraytrajectory::SprayPathPoint point;
            point.time = time;
            point.tcpPose = m_onlineCurrentTablePose.inverse()
                * m_onlineCurrentGunPose;
            point.sprayEnabled = true;
            point.processId = "online";
            m_onlinePendingPoints.push_back(std::move(point));
        }
        m_onlineVirtualTimeSeconds = m_onlineLastPoseTimeSeconds;
        // All physical substeps share one GPU submission/readback and display
        // update. GUI refresh cadence must not determine integration accuracy.
        flushOnlineTrajectory();
        if(!m_onlineVirtualBatchPending) {
            m_onlineVirtualTimer->start(0);
        }
    }

    void CoatingAnalysisModuleController::updateOnlinePoseDisplay()
    {
        if(!m_active || !onlineModeActive() || !m_onlineActive) {
            return;
        }
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }
        for(const OnlineObject& object : m_onlineObjects) {
            services->previewSceneObjectTransform(object.id,
                makeTransformDesc(m_onlineDisplayedTablePose
                    * m_onlineInitialTablePose.inverse()
                    * object.worldFromObject));
        }
        const Eigen::Vector3d position = m_onlineDisplayedGunPose.translation();
        const Eigen::Matrix3d& rotation = m_onlineDisplayedGunPose.linear();
        CoatingTrajectoryPreviewPoint point;
        point.positionX = position.x();
        point.positionY = position.y();
        point.positionZ = position.z();
        const Eigen::Vector3d sprayDirection = rotation * m_onlineDisplayedTool.sprayDirectionLocal;
        point.directionX = sprayDirection.x();
        point.directionY = sprayDirection.y();
        point.directionZ = sprayDirection.z();
        point.frameXAxisX = rotation(0, 0);
        point.frameXAxisY = rotation(1, 0);
        point.frameXAxisZ = rotation(2, 0);
        point.frameYAxisX = rotation(0, 1);
        point.frameYAxisY = rotation(1, 1);
        point.frameYAxisZ = rotation(2, 1);
        point.frameZAxisX = rotation(0, 2);
        point.frameZAxisY = rotation(1, 2);
        point.frameZAxisZ = rotation(2, 2);
        point.sprayEnabled = m_onlineSpraying;
        point.startsNewSegment = true;
        services->setCoatingTrajectoryPreview({ point }, true, false, true);
        updateOnlineInfluenceDisplay();
    }

    void CoatingAnalysisModuleController::updateOnlineInfluenceDisplay()
    {
        auto* services = m_context.viewportServices();
        if(!services) return;
        if(!m_active || !onlineModeActive() || !m_onlineActive || !m_onlineInfluencePreview
            || !m_panel.onlineInfluencePreviewEnabled()) {
            services->setSprayInfluencePreview({});
            return;
        }
        auto& preview = *m_onlineInfluencePreview;
        preview.visible = true;
        preview.thresholdRatio = static_cast<float>(m_panel.onlineInfluenceThresholdRatio());
        const auto surfaceFromWorld = m_onlineDisplayedTablePose.inverse();
        const Eigen::Isometry3d gunInSurface = surfaceFromWorld * m_onlineDisplayedGunPose;
        preview.worldToSurfaceMillimeters = surfaceFromWorld.matrix().cast<float>();
        preview.worldToSurfaceMillimeters.topRows<3>() *= 1000.0f;
        preview.gunPositionMillimeters = (gunInSurface.translation() * 1000.0).cast<float>();
        const Eigen::Vector3d direction = (gunInSurface.linear() * m_onlineDisplayedTool.sprayDirectionLocal).normalized();
        const Eigen::Vector3d preferredMajor = gunInSurface.linear() * m_onlineDisplayedTool.powderFeedDirectionLocal;
        const Eigen::Vector3d major = (preferredMajor - direction * preferredMajor.dot(direction)).normalized();
        preview.direction = direction.cast<float>();
        preview.majorAxis = major.cast<float>();
        preview.minorAxis = direction.cross(major).normalized().cast<float>();
        // Finite drawing extent only. The Gaussian itself has no hard cutoff.
        preview.beamLengthMillimeters = static_cast<float>(1000.0 * std::max(0.001,
            (gunInSurface.translation() - m_onlineVirtualCenter).norm() + m_onlineModelRadiusMeters));
        services->setSprayInfluencePreview(preview);
    }

    void CoatingAnalysisModuleController::updateOnlineScreenRefreshRate()
    {
        QWindow* window = m_panel.window()->windowHandle();
        QScreen* screen = window != nullptr ? window->screen() : QGuiApplication::primaryScreen();
        m_onlineScreenRefreshRate = screen != nullptr && screen->refreshRate() > 0.0
            ? screen->refreshRate() : 60.0;
    }

    void CoatingAnalysisModuleController::refreshOnlineReadouts(double timeSeconds)
    {
        CoatingAnalysisInfoView view;
        view.hasThickness = true;
        view.thicknessMetrics = m_onlineResult->metrics;
        view.predictionTiming = m_onlineResult->timing;
        view.predictionElapsedSeconds = timeSeconds;
        view.minimumDisplayThicknessMicrometers =
            m_onlineResult->metrics.minThickness * kMetersToMicrometers;
        view.maximumDisplayThicknessMicrometers =
            m_onlineResult->metrics.maxThickness * kMetersToMicrometers;
        view.uniformityStatistics = m_onlineUniformity;
        m_infoPanel.applyThicknessInfo(view);
        m_treePanel.applyThicknessInfo(true, view.thicknessMetrics);
    }

    void CoatingAnalysisModuleController::updateOnlineRefreshStatistics()
    {
        if(!m_active || !onlineModeActive()) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(
            now - m_onlineRefreshStartedAt).count();
        if(seconds <= 0.0) {
            return;
        }
        updateOnlineScreenRefreshRate();
        double p95 = -1.0;
        double p99 = -1.0, average = -1.0;
        if(!m_onlineFrameIntervals.empty()) {
            std::sort(m_onlineFrameIntervals.begin(), m_onlineFrameIntervals.end());
            p95 = m_onlineFrameIntervals[static_cast<std::size_t>(
                std::ceil(0.95 * m_onlineFrameIntervals.size())) - 1];
            p99 = m_onlineFrameIntervals[static_cast<std::size_t>(
                std::ceil(0.99 * m_onlineFrameIntervals.size())) - 1];
            double sum = 0.0;
            for(const double interval : m_onlineFrameIntervals) sum += interval;
            average = sum / m_onlineFrameIntervals.size();
        }
        const auto workerState = m_onlineJob->diagnosticState();
        const auto completed = workerState.completedFrames;
        const double latestInputTime = m_onlineVirtualSource
            ? (m_onlineVirtualStopping ? m_onlineVirtualStopTime
                : (m_onlineSpraying ? m_onlineVirtualRunBaseSeconds
                    + std::chrono::duration<double>(now - m_onlineVirtualRunStartedAt).count()
                    : m_onlineVirtualTimeSeconds))
            : m_onlineLastPoseTimeSeconds - m_onlineStartTimeSeconds;
        m_onlineComputeBacklogMilliseconds = m_onlineBackendReady && (m_onlineSpraying || m_onlineFinishing)
            ? std::max(0.0, latestInputTime - workerState.completedTimeSeconds) * 1000.0 : 0.0;
        const double computeFps = completed >= m_onlineLastCompletedFrames
            ? (completed - m_onlineLastCompletedFrames) / seconds : 0.0;
        m_onlineLastCompletedFrames = completed;
        // Qt window submissions are not measurements of physical monitor scanout.
        m_panel.setOnlineRefreshStatistics(m_onlineSceneFrameCount / seconds,
            m_onlineThicknessFrameCount / seconds, m_onlineFirstFrameMilliseconds,
            m_onlineWaitingForFirstFrame, m_onlineViewportFrameCount / seconds,
            p95, m_onlineMaximumFrameIntervalMilliseconds, m_onlineScreenRefreshRate,
            computeFps, average, p99, m_onlineComputeBacklogMilliseconds);
        if(!m_onlineDiagnostics->filePath().isEmpty()) {
            const auto state = m_onlineJob->diagnosticState();
            m_onlineDiagnostics->checkWaitingPhase(state.phase, state.milliseconds);
            m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
        }
        if(m_onlineActive && m_onlineVirtualRandomAxis) {
            m_panel.setOnlineRandomRotationState(m_onlineDisplayedRotationAxis,
                m_onlineRandomRotation.seed());
        }
        m_onlineRefreshStartedAt = now;
        m_onlineSceneFrameCount = 0;
        m_onlineViewportFrameCount = 0;
        m_onlineThicknessFrameCount = 0;
        m_onlineFrameIntervals.clear();
    }

    void CoatingAnalysisModuleController::handleOnlineField(
        const std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>& result,
        const ThicknessUniformityStatistics& uniformity,
        const OnlinePredictionFrame& frame)
    {
        if(!m_onlineActive) {
            m_onlineJob->acknowledgeFrame(frame.id);
            return;
        }
        m_onlinePendingFrameId = frame.id;
        const auto now = std::chrono::steady_clock::now();
        m_onlineRefreshCadence.setFramesPerSecondLimit(m_panel.onlineFramesPerSecondLimit());
        m_onlineFieldReceivedAt = now;
        m_onlineDeliveryWaitMilliseconds = elapsedMilliseconds(frame.computedAt);
        if(m_active && onlineModeActive() && m_onlineSpraying && now < m_onlineNextDisplayAt) {
            m_onlinePendingDisplay = std::make_unique<PendingOnlineDisplay>(
                PendingOnlineDisplay{ result, uniformity, frame });
            const int waitMilliseconds = static_cast<int>(std::ceil(
                std::chrono::duration<double, std::milli>(m_onlineNextDisplayAt - now).count()));
            // Keep the worker's presentation backpressure while waiting. No spray
            // intervals are dropped, and the cloud and poses stay in the same frame.
            QTimer::singleShot(waitMilliseconds, Qt::PreciseTimer, this,
                [this, frameId = frame.id]() {
                    if(m_onlineActive && m_onlinePendingDisplay
                        && m_onlinePendingDisplay->frame.id == frameId) {
                        applyPendingOnlineField();
                    }
                });
            return;
        }
        applyOnlineField(result, uniformity, frame);
    }

    void CoatingAnalysisModuleController::applyPendingOnlineField()
    {
        if(!m_onlinePendingDisplay) return;
        auto pending = std::move(m_onlinePendingDisplay);
        applyOnlineField(pending->result, pending->uniformity, pending->frame);
    }

    void CoatingAnalysisModuleController::applyOnlineField(
        const std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>& result,
        const ThicknessUniformityStatistics& uniformity,
        const OnlinePredictionFrame& frame)
    {
        const auto resultHandlingStartedAt = std::chrono::steady_clock::now();
        if(!m_onlineActive) {
            m_onlineJob->acknowledgeFrame(frame.id);
            return;
        }
        const bool firstResult = !m_onlineResult;
        m_onlineResult = result;
        m_onlineDisplayOverlays = frame.displayOverlays;
        m_onlineGpuDisplayFields = frame.gpuDisplayFields;
        m_onlineWorkerMappingMilliseconds = frame.mappingMilliseconds;
        m_onlinePendingFrameId = frame.id;
        m_onlineDisplayedTablePose = frame.tablePose;
        m_onlineDisplayedGunPose = frame.gunPose;
        m_onlineDisplayedRotationAxis = frame.rotationAxis;
        if(frame.tool) m_onlineDisplayedTool = *frame.tool;
        m_onlineDisplayedTimeSeconds = frame.timeSeconds;
        m_onlineDiagnosticFrameEligible = m_onlineDiagnostics->active()
            && frame.submittedAt >= m_onlineDiagnosticStartedAt;
        if(m_onlineDiagnosticFrameEligible) {
            m_onlineDiagnosticFrame = {};
            m_onlineDiagnosticFrame.id = frame.id;
            m_onlineDiagnosticFrame.vertices = result->size();
            m_onlineDiagnosticFrame.sprayPoints = frame.processedSprayPointCount;
            m_onlineDiagnosticFrame.integratedMilliseconds = frame.integratedMilliseconds;
            // Attribute integration lag at compute completion; presentation waits
            // already have their own CSV columns and must not masquerade as compute lag.
            m_onlineDiagnosticFrame.computeBacklogMilliseconds = m_onlineVirtualSource
                ? std::max(0.0, (std::min(m_onlineVirtualStopping ? m_onlineVirtualStopTime
                        : std::numeric_limits<double>::infinity(), m_onlineVirtualRunBaseSeconds
                            + std::chrono::duration<double>(frame.computedAt - m_onlineVirtualRunStartedAt).count())
                    - frame.timeSeconds) * 1000.0)
                : frame.computeBacklogMilliseconds;
            m_onlineDiagnosticFrame.physicalTimeSeconds = frame.timeSeconds;
            m_onlineDiagnosticSubmittedAt = frame.submittedAt;
            m_onlineDiagnosticRenderedAt = {};
            const auto& timing = result->timing;
            m_onlineDiagnosticFrame.gpuResidentDisplay = timing.gpuResidentDisplay;
            m_onlineDiagnosticFrame.thicknessReadbackBytes = timing.thicknessReadbackBytes;
            m_onlineDiagnosticFrame.statisticsReadbackBytes = timing.statisticsReadbackBytes;
            if(!timing.gpuResidentDisplay && m_onlineShowThickness && frame.displayOverlays) {
                for(const auto& overlay : *frame.displayOverlays) {
                    for(const auto& mesh : overlay.subMeshes) {
                        m_onlineDiagnosticFrame.scalarUploadBytes += mesh.values.size() * sizeof(float);
                    }
                }
            }
            using Stage = OnlineDiagnosticStage;
            m_onlineDiagnosticFrame.at(Stage::InputGap) = frame.inputIntervalMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::InputQueue) = frame.inputQueueMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::BackendCpu) = std::max(0.0,
                timing.backendTotalMilliseconds - timing.uploadMilliseconds
                - timing.dispatchMilliseconds - timing.readbackMilliseconds
                - timing.resultConversionMilliseconds - timing.gpuTimerReadMilliseconds);
            m_onlineDiagnosticFrame.at(Stage::BackendCpu) = std::max(0.0,
                m_onlineDiagnosticFrame.at(Stage::BackendCpu) - timing.gpuCompletionWaitMilliseconds);
            m_onlineDiagnosticFrame.at(Stage::GpuDisplayCopy) = timing.gpuDisplayCopyMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::GpuStatistics) = timing.gpuStatisticsMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::GpuCompletionWait) = timing.gpuCompletionWaitMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::Upload) = timing.uploadMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::Dispatch) = timing.dispatchMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::GpuCompute) = timing.pureGpuMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::Readback) = timing.readbackMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::ResultConversion) = timing.resultConversionMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::GpuTimerWait) = timing.gpuTimerReadMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::Statistics) = frame.statisticsMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::Mapping) = frame.mappingMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::DeliveryQueue) = frame.deliveryQueueMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::PreviousPresentation) = frame.previousPresentationMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::GuiPreparation) = m_onlineGuiStartupMilliseconds;
            m_onlineGuiStartupMilliseconds = 0.0;
            m_onlineDiagnosticFrame.at(Stage::ContextPreparation) = frame.contextPreparationMilliseconds;
            m_onlineDiagnosticFrame.at(Stage::BackendInitialization) = frame.backendInitializationMilliseconds;
        }
        if(frame.statisticsUpdated) m_onlineUniformity = uniformity;
        else if(!m_onlineSpraying) {
            m_onlineUniformity = calculateThicknessUniformityStatistics(*result,
                result->metrics.minThickness, result->metrics.maxThickness);
        }
        m_onlineStatus = QStringLiteral("Online thickness: %1 vertices, maximum %2 um.")
            .arg(static_cast<qulonglong>(result->size()))
            .arg(result->metrics.maxThickness * kMetersToMicrometers, 0, 'f', 3);
        if(!m_onlineSpraying) m_onlineStatus = onlineStoppedStatus();
        if(m_active && onlineModeActive()) {
            const auto displayStart = std::chrono::steady_clock::now();
            m_onlinePacingWaitMilliseconds = elapsedMilliseconds(m_onlineFieldReceivedAt);
            if(m_onlineDiagnosticFrameEligible) {
                m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::GuiResult) =
                    std::chrono::duration<double, std::milli>(displayStart - resultHandlingStartedAt).count();
                m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::Pacing) = std::max(0.0,
                    std::chrono::duration<double, std::milli>(resultHandlingStartedAt - m_onlineFieldReceivedAt).count());
            }
            m_onlineStatisticsMilliseconds = frame.statisticsMilliseconds;
            m_status = m_onlineStatus;
            if(!restoreOnlineDisplay()) {
                if(m_onlineDiagnosticFrameEligible) {
                    m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::GuiPose) = m_onlinePoseMilliseconds;
                    m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::GuiOverlay) = m_onlineOverlayMilliseconds;
                    m_onlineDiagnostics->event(QStringLiteral("DISPLAY_FAILED"),
                        elapsedMilliseconds(displayStart), m_onlineStatus);
                }
                handleOnlineFramePresented(frame.id, false);
                return;
            }
            const auto infoStart = std::chrono::steady_clock::now();
            if(firstResult || !m_onlineSpraying || frame.statisticsUpdated) {
                m_panel.setOnlinePredictionState(
                    m_onlineActive, m_onlineSpraying, m_status, m_onlineFinishing);
                if(m_onlineVirtualRandomAxis) {
                    m_panel.setOnlineRandomRotationState(frame.rotationAxis,
                        m_onlineRandomRotation.seed());
                }
                // Only structural transitions need the complete controls/visibility view.
                if(firstResult || !m_onlineSpraying) refreshViewModel();
                else refreshOnlineReadouts(frame.timeSeconds);
                publishStateChanged();
            }
            if(!firstResult && m_onlineSpraying) {
                emit thicknessLegendChanged(true,
                    result->metrics.minThickness * kMetersToMicrometers,
                    result->metrics.maxThickness * kMetersToMicrometers, false, false);
            }
            const auto now = std::chrono::steady_clock::now();
            if(now - m_onlineLastDisplayLog >= std::chrono::seconds(1)) {
                m_onlineLastDisplayLog = now;
                LOG_DEBUG("rs2026") << "Online thickness display: vertices="
                    << result->size() << ", applyAndInfoMs="
                    << std::chrono::duration<double, std::milli>(
                        now - displayStart).count()
                    << ", poseMs=" << m_onlinePoseMilliseconds
                    << ", mappingMs=" << m_onlineMappingMilliseconds
                    << ", workerMappingMs=" << m_onlineWorkerMappingMilliseconds
                    << ", overlayMs=" << m_onlineOverlayMilliseconds
                    << ", infoAndLegendMs=" << elapsedMilliseconds(infoStart)
                    << ", fullInfoRefresh=" << (firstResult || !m_onlineSpraying);
            }
            RobotQtViewerViewportServices* services = m_context.viewportServices();
            const double applyMilliseconds = elapsedMilliseconds(displayStart);
            if(services && m_onlineDiagnosticFrameEligible) {
                std::uint64_t measuredFrameId = 0;
                const double gpuDrawMs = services->latestCoatingGpuDrawMilliseconds(measuredFrameId);
                m_onlineDiagnostics->gpuDraw(measuredFrameId, gpuDrawMs);
            }
            m_onlineApplyMilliseconds = applyMilliseconds;
            if(m_onlineDiagnosticFrameEligible) {
                using Stage = OnlineDiagnosticStage;
                m_onlineDiagnosticFrame.expectedIntervalMilliseconds = m_onlineRefreshCadence.displayIntervalMilliseconds();
                m_onlineDiagnosticFrame.at(Stage::GuiPose) = m_onlinePoseMilliseconds;
                m_onlineDiagnosticFrame.at(Stage::GuiOverlay) = m_onlineOverlayMilliseconds;
                m_onlineDiagnosticFrame.at(Stage::GuiInfo) = elapsedMilliseconds(infoStart);
            }
            if(m_onlineSpraying) {
                m_onlineRefreshCadence.setFramesPerSecondLimit(m_panel.onlineFramesPerSecondLimit());
                // Start a fresh period after a late frame; never burst to catch up.
                m_onlineNextDisplayAt = displayStart + std::chrono::microseconds(
                    static_cast<long long>(m_onlineRefreshCadence.displayIntervalMilliseconds() * 1000.0));
            }
            m_onlinePresentationRequestedAt = std::chrono::steady_clock::now();
            if(services != nullptr
                && services->requestCoatingFramePresentation(frame.id)) {
                return;
            }
        }
        handleOnlineFramePresented(frame.id, false);
    }

    void CoatingAnalysisModuleController::resetOnlinePrediction()
    {
        if(auto* services = m_context.viewportServices()) services->setSprayInfluencePreview({});
        m_onlineInfluencePreview.reset();
        m_onlineVirtualRotating = false;
        m_onlineVirtualMovingGun = false;
        m_onlineMotionClock.reset();
        m_onlineMotionParametersPending = false;
        m_panel.setOnlineMotionState(false, false);
        m_onlineBackendReady = false;
        m_onlineStartAfterPreparation = false;
        m_onlineVirtualBatchPending = false;
        m_onlineVirtualStopping = false;
        m_onlineFinishing = false;
        m_onlineUnsimulatedSeconds = 0.0;
        m_onlineIntegrationSampling.setSurfaceDistanceQuery({});
        m_onlineDiagnostics->finish(QStringLiteral("online prediction reset"));
        m_onlineDiagnosticTimer->stop();
        if(!m_onlineDiagnostics->filePath().isEmpty()) {
            m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
        }
        if(!m_onlineActive && !m_onlineSpraying) {
            return;
        }
        m_onlinePoseWatchdog->stop();
        m_onlineVirtualTimer->stop();
        m_onlineJob->reset();
        m_onlinePendingFrameId = 0;
        m_onlineLastSubmittedFrameId = m_onlineLastAcknowledgedFrameId = 0;
        m_onlineDiagnosticFrameEligible = false;
        m_onlinePendingDisplay.reset();
        m_onlineNextDisplayAt = {};
        m_onlineLastPresentedAt = {};
        m_onlineFrameIntervals.clear();
        m_onlineMaximumFrameIntervalMilliseconds = -1.0;
        m_onlineDisplayedTimeSeconds = 0.0;
        if(onlineModeActive()) {
            RobotQtViewerViewportServices* services = m_context.viewportServices();
            if(services != nullptr) {
                services->setSurfaceScalarProbeEnabled(false, QString());
                for(const OnlineObject& object : m_onlineObjects) {
                    services->clearSurfaceScalarOverlay(object.id);
                    services->previewSceneObjectTransform(
                        object.id, makeTransformDesc(object.worldFromObject));
                }
                services->setCoatingTrajectoryPreview({}, false);
                if(m_active) {
                    services->setCoatingAnalysisView(true);
                }
            }
        }
        m_onlineActive = false;
        m_onlineSpraying = false;
        if(m_active && onlineModeActive()) {
            applyModelVisibilityOverrides();
        }
        m_onlineObjects.clear();
        m_onlinePendingPoints.clear();
        m_onlineResult.reset();
        m_onlineDisplayOverlays.reset();
        m_onlineGpuDisplayFields.reset();
        m_onlineUniformity = ThicknessUniformityStatistics();
        m_onlineFirstFrameMilliseconds = -1.0;
        m_onlineWaitingForFirstFrame = false;
        m_onlineThicknessFramePending = false;
        m_onlineSprayRequestedAt = {};
        m_onlineRefreshStartedAt = std::chrono::steady_clock::now();
        m_onlineSceneFrameCount = 0;
        m_onlineViewportFrameCount = 0;
        m_onlineThicknessFrameCount = 0;
        m_panel.setOnlineRefreshStatistics(0.0, 0.0, -1.0, false);
        m_onlinePickEnabled = false;
        m_onlineStatus = QStringLiteral("Ready to start online prediction.");
        m_panel.setOnlinePredictionState(false, false,
            m_onlineStatus);
        if(onlineModeActive()) {
            m_status = m_onlineStatus;
            refreshViewModel();
            publishStateChanged();
        }
        if(m_active && m_mode == CoatingAnalysisMode::Prediction
            && m_session.hasResult) {
            QString error;
            applyCurrentSurfaceOverlay(&error);
        }
        if(m_active && m_mode == CoatingAnalysisMode::Prediction) {
            updateTrajectoryPreviewVisibility();
        }
    }

    void CoatingAnalysisModuleController::predictThickness()
    {
        if(anyPredictionRunning() || onlineModeActive()) {
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
            task.options.base.trajectorySamplingMode =
                m_session.appliedTrajectorySamplingMode;
            task.options.base.timeStep =
                m_session.appliedTrajectoryTimeStepSeconds;
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

    bool CoatingAnalysisModuleController::onlineModeActive() const
    {
        return m_mode == CoatingAnalysisMode::Online;
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
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            if(m_session.hasResult) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
        }
        m_predictionSession = std::move(m_session);
        m_predictionModelVisibility = std::move(m_modelVisibility);
        m_predictionStatus = m_status;
        m_session = std::move(m_simulationSession);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_modelVisibility = std::move(m_simulationModelVisibility);
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_mode = CoatingAnalysisMode::Simulation;
        if(!m_simulationReady) {
            m_session.objectId = QString::fromLatin1(kSimulationPlateObjectId);
            m_session.showModel = true;
            m_session.showTrajectory = true;
            m_session.showSprayPoints = true;
            m_simulationStatus = QStringLiteral("Simulation mode active.");
            rebuildSimulation(false, true);
            return;
        }
        restoreSimulationPreview();
        applyOverlayAfterReload();
        m_status = m_simulationStatus;
        refreshViewModel();
        publishStateChanged();
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
        m_simulationSession = std::move(m_session);
        m_simulationModelVisibility = std::move(m_modelVisibility);
        m_session = std::move(m_predictionSession);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_modelVisibility = std::move(m_predictionModelVisibility);
        m_mode = CoatingAnalysisMode::Prediction;
        m_status = m_predictionStatus;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
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

    void CoatingAnalysisModuleController::restoreSimulationPreview()
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr || !m_simulation.displayModel) {
            return;
        }
        QString error;
        if(!services->setCoatingPredictionModel(
                m_session.objectId, *m_simulation.displayModel, &error)) {
            m_status = error;
            return;
        }
        QHash<QString, bool> visibility;
        for(const simulation_project::SceneObjectDesc& object :
            m_context.document().objects) {
            if(object.objectType == "workpiece") {
                visibility.insert(QString::fromStdString(object.id), false);
            }
        }
        services->setCoatingModelVisibilities(visibility);
        services->setCoatingModelVisible(m_session.objectId, m_session.showModel);
        submitTrajectoryPreview();
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
        m_treePanel.setWaypoints(&m_session.waypoints);
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
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            if(m_session.hasResult) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
        }
        m_predictionSession = std::move(m_session);
        m_predictionModelVisibility = std::move(m_modelVisibility);
        m_predictionStatus = m_status;
        m_session = std::move(m_reproductionSession);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_modelVisibility = std::move(m_reproductionModelVisibility);
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_mode = CoatingAnalysisMode::Reproduction;
        applyModelVisibilityOverrides();
        if(!m_reproductionSetupInitialized) {
            m_reproductionSetupInitialized = true;
            generateReproductionScene(true);
            generateReproductionTrajectory();
        } else {
            restoreReproductionPreviews(false);
        }
        if(m_session.hasResult) {
            applyOverlayAfterReload();
        }
        if(!m_reproductionSceneReady && !m_reproductionTrajectoryReady
            && m_reproductionStatus == QStringLiteral("No reproduction result yet.")) {
            updateReproductionPreparationStatus();
        }
        m_status = m_reproductionStatus;
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::exitReproduction()
    {
        if(anyPredictionRunning() || !reproductionActive()) {
            return;
        }
        m_reproductionFramePending = false;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            if(m_session.hasResult && !m_reproductionResultUsesGeneratedScene) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
        }
        clearReproductionGeneratedPreview();
        m_reproductionSession = std::move(m_session);
        m_reproductionModelVisibility = std::move(m_modelVisibility);
        m_session = std::move(m_predictionSession);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_modelVisibility = std::move(m_predictionModelVisibility);
        m_mode = CoatingAnalysisMode::Prediction;
        m_status = m_predictionStatus;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        applyModelVisibilityOverrides();
        submitTrajectoryPreview();
        applyOverlayAfterReload();
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::enterOnline()
    {
        if(anyPredictionRunning()) {
            return;
        }
        const QString selectedWorkpieceId = m_session.objectId;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            services->clearCoatingPredictionDebugState();
            if(m_session.hasResult) {
                services->setSurfaceScalarOverlayVisible(m_session.objectId, false);
            }
            if(m_session.predictionDisplayModel) {
                services->clearCoatingPredictionModel(m_session.objectId);
            }
            services->setCoatingAnalysisView(true);
        }
        m_predictionSession = std::move(m_session);
        m_predictionModelVisibility = std::move(m_modelVisibility);
        m_predictionStatus = m_status;
        m_session = std::move(m_onlineSession);
        m_modelVisibility = std::move(m_onlineModelVisibility);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_mode = CoatingAnalysisMode::Online;
        m_onlineRefreshStartedAt = std::chrono::steady_clock::now();
        m_onlineLastPresentedAt = {};
        m_onlineNextDisplayAt = {};
        m_onlineFrameIntervals.clear();
        m_onlineSceneFrameCount = 0;
        m_onlineViewportFrameCount = 0;
        m_onlineThicknessFrameCount = 0;
        m_onlineRefreshTimer->start();
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setCoatingFrameDrivenRefresh(true);
        }
        const auto* selected = findObject(m_context.document(), selectedWorkpieceId);
        const QString onlineObjectId = selected != nullptr
                && selected->objectType == "workpiece"
            ? selectedWorkpieceId : QString();
        if(m_session.objectId != onlineObjectId) {
            resetOnlinePrediction();
            m_onlineStatus = QStringLiteral("Ready to start online prediction.");
            m_session.objectId = onlineObjectId;
            m_session.modelName = selected != nullptr
                ? QString::fromStdString(selected->name) : QString();
            m_session.sourcePath = selected != nullptr
                ? QString::fromStdString(selected->sourcePath) : QString();
            m_session.modelInfo = CoatingAnalysisModelInfo();
            if(selected != nullptr) {
                updateSelectedModelInfo();
            }
        }
        m_status = onlineObjectId.isEmpty()
            ? QStringLiteral("Load the debug model in Thickness Prediction first.")
            : m_onlineStatus;
        if(selected && !onlineObjectId.isEmpty() && !m_onlineActive
            && m_panel.onlinePoseSource() == OnlinePoseSource::Virtual) {
            const auto path = simulation_project::AssetResolver::resolveProjectPath(
                makeResolveContext(m_context.projectSession()), selected->sourcePath);
            std::string error;
            const auto model = assetcore::AssetManager::instance().tryLoadModel(
                path.generic_u8string(), static_cast<float>(selected->visualScale), &error);
            if(model) m_onlineJob->prepareModel(model, makeTransform(selected->transform));
        }
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        applyModelVisibilityOverrides();
        updateTrajectoryPreviewVisibility();
        restoreOnlineDisplay();
        m_panel.setOnlinePredictionState(m_onlineActive, m_onlineSpraying, m_status, m_onlineFinishing);
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::exitOnline()
    {
        if(!onlineModeActive()) {
            return;
        }
        m_onlineVirtualMovingGun = false;
        m_onlineVirtualRotating = false;
        m_onlineVirtualTimer->stop();
        m_panel.setOnlineMotionState(false, false);
        stopOnlineSpray();
        // Preserve the last computed field even when leaving before its display deadline.
        applyPendingOnlineField();
        m_onlineDiagnostics->finish(QStringLiteral("left online page"));
        m_onlineDiagnosticTimer->stop();
        m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
        m_onlineRefreshTimer->stop();
        m_onlineThicknessFramePending = false;
        if(m_onlinePendingFrameId != 0) {
            handleOnlineFramePresented(m_onlinePendingFrameId, false);
        }
        m_onlineStatus = m_status;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSprayInfluencePreview({});
            services->setCoatingFrameDrivenRefresh(false);
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            for(const OnlineObject& object : m_onlineObjects) {
                services->setSurfaceScalarOverlayVisible(object.id, false);
                services->previewSceneObjectTransform(
                    object.id, makeTransformDesc(object.worldFromObject));
            }
            services->setCoatingAnalysisView(true);
        }
        m_onlineSession = std::move(m_session);
        m_onlineModelVisibility = std::move(m_modelVisibility);
        m_session = std::move(m_predictionSession);
        m_modelVisibility = std::move(m_predictionModelVisibility);
        m_treePanel.setWaypoints(&m_session.waypoints);
        m_mode = CoatingAnalysisMode::Prediction;
        m_status = m_predictionStatus;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        applyModelVisibilityOverrides();
        submitTrajectoryPreview();
        applyOverlayAfterReload();
        refreshViewModel();
        publishStateChanged();
    }

    bool CoatingAnalysisModuleController::restoreOnlineDisplay()
    {
        if(!m_active || !onlineModeActive() || !m_onlineActive) {
            return false;
        }
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return false;
        }
        const auto poseStart = std::chrono::steady_clock::now();
        updateOnlinePoseDisplay();
        m_onlinePoseMilliseconds = elapsedMilliseconds(poseStart);
        m_onlineMappingMilliseconds = 0.0;
        m_onlineOverlayMilliseconds = 0.0;
        if(m_onlineDisplayOverlays) {
            for(std::size_t index = 0; index < m_onlineDisplayOverlays->size(); ++index) {
                const auto& overlay = (*m_onlineDisplayOverlays)[index];
                const auto overlayStart = std::chrono::steady_clock::now();
                if(m_onlineShowThickness) {
                    QString error;
                    const auto* gpuField = m_onlineGpuDisplayFields
                        ? &m_onlineGpuDisplayFields->at(index) : nullptr;
                    if(!services->applySurfaceScalarOverlay(overlay, &error, false, gpuField)) {
                        m_status = QStringLiteral("Online display failed: ") + error;
                        m_onlineStatus = m_status;
                        return false;
                    }
                    m_onlineThicknessFramePending = true;
                }
                // Keep computing and caching while hidden, without uploading
                // an invisible cloud. Showing it again applies the latest field.
                services->setSurfaceScalarOverlayVisible(
                    QString::fromStdString(overlay.objectId), m_onlineShowThickness);
                m_onlineOverlayMilliseconds += elapsedMilliseconds(overlayStart);
            }
        }
        services->setSurfaceScalarProbeEnabled(
            m_onlinePickEnabled && m_onlineShowThickness,
            m_session.objectId);
        return true;
    }

    bool CoatingAnalysisModuleController::rebuildReproductionScene(
        QString* errorMessage)
    {
        m_reproductionPlateStack = PlateStackData();

        if(m_panel.reproductionSceneSource()
            == ReproductionSceneSource::GeneratedPlateStack) {
            if(!SimulationExperiment::buildPlateStack(
                    m_panel.reproductionPlateStackParameters(),
                    m_reproductionPlateStack,
                    errorMessage)) {
                return false;
            }
        } else if(m_session.objectId.isEmpty()
            || findObject(m_context.document(), m_session.objectId) == nullptr) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "Load an analysis model or select the generated plate stack.");
            }
            return false;
        }

        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    bool CoatingAnalysisModuleController::rebuildReproductionTrajectory(
        QString* errorMessage)
    {
        m_reproductionGeneratedTrajectory = spraytrajectory::SprayTrajectory();

        if(m_panel.reproductionTrajectorySource()
            != ReproductionTrajectorySource::ImportedTrajectory) {
            if(!SimulationExperiment::buildTrajectory(
                    m_panel.reproductionGeneratedTrajectoryParameters(),
                    m_reproductionGeneratedTrajectory,
                    errorMessage)) {
                return false;
            }
        } else if(m_session.trajectory.empty()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "Load a spray trajectory or select a generated trajectory.");
            }
            return false;
        }

        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    void CoatingAnalysisModuleController::clearReproductionGeneratedPreview()
    {
        const QString objectId = QString::fromLatin1(
            kReproductionPlateStackObjectId);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearSurfaceScalarOverlay(objectId);
            services->clearCoatingPredictionModel(objectId);
            services->setCoatingModelVisible(objectId, false);
        }
        m_reproductionGeneratedPreviewVisible = false;
    }

    QString CoatingAnalysisModuleController::activeCoatingObjectId() const
    {
        if(reproductionActive()
            && (m_reproductionGeneratedPreviewVisible
                || m_reproductionResultUsesGeneratedScene)) {
            return QString::fromLatin1(kReproductionPlateStackObjectId);
        }
        return m_session.objectId;
    }

    bool CoatingAnalysisModuleController::generatedReproductionSceneActive() const
    {
        return reproductionActive()
            && m_panel.reproductionSceneSource()
                == ReproductionSceneSource::GeneratedPlateStack;
    }

    bool CoatingAnalysisModuleController::generatedReproductionTrajectoryActive() const
    {
        return reproductionActive()
            && m_panel.reproductionTrajectorySource()
                != ReproductionTrajectorySource::ImportedTrajectory;
    }

    void CoatingAnalysisModuleController::submitReproductionTrajectoryPreview(
        const spraytrajectory::SprayTrajectory& trajectory)
    {
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            return;
        }
        std::vector<CoatingTrajectoryPreviewPoint> preview;
        preview.reserve(trajectory.flattenedPoints().size());
        for(const spraytrajectory::SpraySegment& segment : trajectory.segments) {
            bool first = true;
            for(const spraytrajectory::SprayPathPoint& point : segment.points) {
                CoatingTrajectoryPreviewPoint item;
                const Eigen::Vector3d position = point.tcpPose.translation();
                const Eigen::Matrix3d& rotation = point.tcpPose.linear();
                const Eigen::Vector3d direction = rotation * Eigen::Vector3d::UnitZ();
                item.positionX = position.x();
                item.positionY = position.y();
                item.positionZ = position.z();
                item.directionX = direction.x();
                item.directionY = direction.y();
                item.directionZ = direction.z();
                item.frameXAxisX = rotation(0, 0);
                item.frameXAxisY = rotation(1, 0);
                item.frameXAxisZ = rotation(2, 0);
                item.frameYAxisX = rotation(0, 1);
                item.frameYAxisY = rotation(1, 1);
                item.frameYAxisZ = rotation(2, 1);
                item.frameZAxisX = rotation(0, 2);
                item.frameZAxisY = rotation(1, 2);
                item.frameZAxisZ = rotation(2, 2);
                item.sprayEnabled = point.sprayEnabled && segment.sprayEnabled;
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

    void CoatingAnalysisModuleController::invalidateReproductionResult()
    {
        m_reproductionFramePending = false;
        if(!m_session.hasReproductionResult) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearSurfaceScalarOverlay(
                QString::fromLatin1(kReproductionPlateStackObjectId));
            if(!m_session.objectId.isEmpty()) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.clearResult();
        m_hasCurrentThickness = false;
        m_reproductionResultUsesGeneratedScene = false;
    }

    void CoatingAnalysisModuleController::updateReproductionPreparationStatus()
    {
        m_reproductionStatus = m_reproductionSceneDetails
            + QStringLiteral("\n") + m_reproductionTrajectoryDetails;
        m_status = m_reproductionStatus;
    }

    bool CoatingAnalysisModuleController::displayPreparedReproductionScene(
        bool focusView,
        QString* errorMessage)
    {
        if(!m_reproductionSceneReady) {
            return false;
        }
        const bool generatedScene = m_panel.reproductionSceneSource()
            == ReproductionSceneSource::GeneratedPlateStack;
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("The viewport is unavailable.");
            }
            return false;
        }

        if(generatedScene) {
            const QString objectId = QString::fromLatin1(
                kReproductionPlateStackObjectId);
            QString displayError;
            if(!m_reproductionPlateStack.displayModel
                || !services->setCoatingPredictionModel(
                    objectId,
                    *m_reproductionPlateStack.displayModel,
                    &displayError)) {
                if(errorMessage != nullptr) {
                    *errorMessage = displayError.isEmpty()
                    ? QStringLiteral("Failed to display the generated plate stack.")
                    : displayError;
                }
                return false;
            }
            QHash<QString, bool> visibility;
            for(const simulation_project::SceneObjectDesc& object :
                m_context.document().objects) {
                if(object.objectType == "workpiece") {
                    visibility.insert(QString::fromStdString(object.id), false);
                }
            }
            services->setCoatingModelVisibilities(visibility);
            services->setCoatingModelVisible(objectId, m_session.showModel);
            if(focusView) {
                services->focusCoatingObject(objectId, 0.3);
            }
            m_reproductionGeneratedPreviewVisible = true;
        } else {
            applyModelVisibilityOverrides();
            if(focusView && !m_session.objectId.isEmpty()) {
                services->focusCoatingObject(m_session.objectId, 0.3);
            }
        }

        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    void CoatingAnalysisModuleController::displayPreparedReproductionTrajectory()
    {
        if(!m_reproductionTrajectoryReady) {
            return;
        }
        if(m_panel.reproductionTrajectorySource()
            != ReproductionTrajectorySource::ImportedTrajectory) {
            submitReproductionTrajectoryPreview(m_reproductionGeneratedTrajectory);
        } else {
            submitTrajectoryPreview();
        }
    }

    void CoatingAnalysisModuleController::restoreReproductionPreviews(
        bool focusView)
    {
        if(!reproductionActive()) {
            return;
        }
        QString error;
        if(m_reproductionSceneReady
            && !displayPreparedReproductionScene(focusView, &error)) {
            m_reproductionSceneReady = false;
            m_reproductionSceneDetails = error;
            updateReproductionPreparationStatus();
        }
        displayPreparedReproductionTrajectory();
    }

    void CoatingAnalysisModuleController::generateReproductionScene(bool focusView)
    {
        if(!reproductionActive() || anyPredictionRunning()) {
            return;
        }
        invalidateReproductionResult();
        m_reproductionSceneReady = false;
        clearReproductionGeneratedPreview();

        QString error;
        if(!rebuildReproductionScene(&error)) {
            m_reproductionSceneDetails = error;
            updateReproductionPreparationStatus();
            refreshViewModel();
            publishStateChanged();
            return;
        }

        m_reproductionSceneReady = true;
        if(!displayPreparedReproductionScene(focusView, &error)) {
            m_reproductionSceneReady = false;
            m_reproductionSceneDetails = error;
            updateReproductionPreparationStatus();
            refreshViewModel();
            publishStateChanged();
            return;
        }

        if(m_panel.reproductionSceneSource()
            == ReproductionSceneSource::GeneratedPlateStack) {
            const PlateStackParameters& parameters =
                m_reproductionPlateStack.parameters;
            m_reproductionSceneDetails = QStringLiteral(
                "Scene ready: %1 plates, %2 mm spacing, "
                "%3 x %4 cells per plate, %5 vertices.")
                .arg(parameters.plateCount)
                .arg(parameters.plateSpacingMillimeters, 0, 'f', 2)
                .arg(static_cast<qulonglong>(m_reproductionPlateStack.rowCount))
                .arg(static_cast<qulonglong>(m_reproductionPlateStack.columnCount))
                .arg(static_cast<qulonglong>(
                    m_reproductionPlateStack.workpiece.samples.size()));
        } else {
            m_reproductionSceneDetails = QStringLiteral("Scene ready: %1.")
                .arg(m_session.modelName);
        }
        updateReproductionPreparationStatus();
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::generateReproductionTrajectory()
    {
        if(!reproductionActive() || anyPredictionRunning()) {
            return;
        }
        invalidateReproductionResult();
        m_reproductionTrajectoryReady = false;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setCoatingTrajectoryPreview({}, false);
        }

        QString error;
        if(!rebuildReproductionTrajectory(&error)) {
            m_reproductionTrajectoryDetails = error;
            updateReproductionPreparationStatus();
            refreshViewModel();
            publishStateChanged();
            return;
        }

        m_reproductionTrajectoryReady = true;
        displayPreparedReproductionTrajectory();
        const bool generatedTrajectory = m_panel.reproductionTrajectorySource()
            != ReproductionTrajectorySource::ImportedTrajectory;
        const spraytrajectory::SprayTrajectory& trajectory = generatedTrajectory
            ? m_reproductionGeneratedTrajectory : m_session.trajectory;
        m_reproductionTrajectoryDetails = generatedTrajectory
            ? QStringLiteral("Trajectory ready: %1 generated points.")
                  .arg(static_cast<qulonglong>(
                      trajectory.flattenedPoints().size()))
            : QStringLiteral("Trajectory ready: %1.")
                  .arg(m_session.trajectoryName);
        updateReproductionPreparationStatus();
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::startReproductionBenchmark()
    {
        if(!reproductionActive() || anyPredictionRunning()
            || m_reproductionBenchmarkActive) {
            return;
        }
        QSettings settings;
        const QString directory = settings.value(
            QString::fromLatin1(kReproductionExportDirectorySettingsKey)).toString();
        const QString fileName = QStringLiteral("reproduction-benchmark-%1.csv")
            .arg(QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd-hhmmss")));
        const QString path = QFileDialog::getSaveFileName(&m_panel,
            QStringLiteral("Save raw benchmark measurements"),
            directory.isEmpty() ? fileName : QDir(directory).filePath(fileName),
            QStringLiteral("CSV files (*.csv)"));
        if(path.isEmpty()) {
            return;
        }
        QFile file(path);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            m_status = file.errorString();
            emit statusMessageRequested(m_status, 5000);
            return;
        }
        QTextStream stream(&file);
        stream.setCodec("UTF-8");
        stream << "algorithm_id,run_type,repeat,status,scene_side_mm,plate_count,"
                  "plate_spacing_mm,cell_mm,spray_distance_mm,incidence_deg,"
                  "scan_speed_mm_s,point_interval_s,input_vertices,input_triangles,"
                  "trajectory_points,output_vertices,output_triangles,"
                  "preparation_ms,core_ms,conversion_ms,display_ms,"
                  "first_frame_wait_ms,total_ms,nonzero_vertices,error\n";
        file.close();
        settings.setValue(
            QString::fromLatin1(kReproductionExportDirectorySettingsKey),
            QFileInfo(path).absolutePath());
        m_reproductionBenchmarkPath = path;
        m_reproductionBenchmarkAlgorithmIndex = 0;
        m_reproductionBenchmarkAttempt = 0;
        m_reproductionBenchmarkActive = true;
        QTimer::singleShot(0, this,
            [this]() { advanceReproductionBenchmark(); });
    }

    void CoatingAnalysisModuleController::advanceReproductionBenchmark()
    {
        if(!m_reproductionBenchmarkActive) {
            return;
        }
        if(!reproductionActive()) {
            m_reproductionBenchmarkActive = false;
            return;
        }
        if(m_reproductionBenchmarkAlgorithmIndex
            >= static_cast<int>(kBenchmarkAlgorithms.size())) {
            m_reproductionBenchmarkActive = false;
            m_status = QStringLiteral("Benchmark completed: %1")
                .arg(m_reproductionBenchmarkPath);
            appendReproductionDiagnostic(QStringLiteral("Benchmark"), m_status);
            refreshViewModel();
            emit statusMessageRequested(m_status, 10000);
            return;
        }

        m_reproductionBenchmarkRunPending = true;
        m_reproductionTimingValid = false;
        m_reproductionInputVertexCount = 0;
        m_reproductionInputTriangleCount = 0;
        m_reproductionOutputVertexCount = 0;
        m_reproductionOutputTriangleCount = 0;
        const auto algorithm = kBenchmarkAlgorithms[
            static_cast<std::size_t>(m_reproductionBenchmarkAlgorithmIndex)];
        if(m_reproductionBenchmarkAttempt == 0
            && !m_panel.applyReproductionBenchmarkSetup(algorithm)) {
            recordReproductionBenchmarkRun(QStringLiteral("setup_failed"),
                QStringLiteral("Recommended setup or calibration is unavailable."));
            return;
        }
        if(!m_reproductionSceneReady || !m_reproductionTrajectoryReady) {
            recordReproductionBenchmarkRun(QStringLiteral("setup_failed"),
                m_reproductionSceneDetails + QStringLiteral("; ")
                    + m_reproductionTrajectoryDetails);
            return;
        }
        QTimer::singleShot(0, this, [this]() {
            if(!m_reproductionBenchmarkActive) {
                return;
            }
            runAlgorithmReproduction();
            if(!m_reproductionRunActive && !m_reproductionFramePending) {
                recordReproductionBenchmarkRun(
                    QStringLiteral("failed"), m_status);
            }
        });
    }

    void CoatingAnalysisModuleController::recordReproductionBenchmarkRun(
        const QString& status, const QString& error)
    {
        if(!m_reproductionBenchmarkRunPending) {
            return;
        }
        m_reproductionBenchmarkRunPending = false;
        const auto algorithm = kBenchmarkAlgorithms[
            static_cast<std::size_t>(m_reproductionBenchmarkAlgorithmIndex)];
        const PlateStackParameters scene = m_panel.reproductionPlateStackParameters();
        const SimulationExperimentParameters trajectory =
            m_panel.reproductionGeneratedTrajectoryParameters();
        const auto number = [](double value) {
            return QString::number(value, 'f', 6);
        };
        const std::size_t nonzero = m_reproductionTimingValid && m_session.hasResult
            ? activeThicknessCount(m_session.prediction.field) : 0;
        QFile file(m_reproductionBenchmarkPath);
        if(file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
            QTextStream stream(&file);
            stream.setCodec("UTF-8");
            stream << csvCell(QString::fromLatin1(
                spraythickness::reproductionAlgorithmId(algorithm))) << ','
                << (m_reproductionBenchmarkAttempt == 0 ? "warmup" : "measured") << ','
                << m_reproductionBenchmarkAttempt << ',' << csvCell(status) << ','
                << number(scene.plateSideMillimeters) << ',' << scene.plateCount << ','
                << number(scene.plateSpacingMillimeters) << ','
                << number(scene.cellSizeMillimeters) << ','
                << number(trajectory.sprayDistanceMillimeters) << ','
                << number(trajectory.incidenceAngleDegrees) << ','
                << number(trajectory.scanSpeedMillimetersPerSecond) << ','
                << number(trajectory.trajectoryPointIntervalSeconds) << ','
                << m_reproductionInputVertexCount << ','
                << m_reproductionInputTriangleCount << ','
                << m_reproductionGeneratedTrajectory.flattenedPoints().size() << ','
                << m_reproductionOutputVertexCount << ','
                << m_reproductionOutputTriangleCount << ',';
            if(m_reproductionTimingValid) {
                stream << number(m_reproductionPreparationMilliseconds) << ','
                    << number(m_reproductionCoreMilliseconds) << ','
                    << number(m_reproductionConversionMilliseconds) << ','
                    << number(m_reproductionDisplayMilliseconds) << ','
                    << number(m_reproductionPresentationMilliseconds) << ','
                    << number(m_reproductionTotalMilliseconds);
            } else {
                stream << ",,,,,";
            }
            stream << ',' << nonzero << ',' << csvCell(error) << '\n';
        } else {
            m_reproductionBenchmarkActive = false;
            m_status = QStringLiteral("Benchmark log write failed: %1")
                .arg(file.errorString());
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        if(status == QStringLiteral("setup_failed")) {
            m_reproductionBenchmarkAttempt = 6;
        } else {
            ++m_reproductionBenchmarkAttempt;
        }
        if(m_reproductionBenchmarkAttempt > 5) {
            m_reproductionBenchmarkAttempt = 0;
            ++m_reproductionBenchmarkAlgorithmIndex;
        }
        if(m_reproductionBenchmarkActive) {
            QTimer::singleShot(0, this,
                [this]() { advanceReproductionBenchmark(); });
        }
    }

    void CoatingAnalysisModuleController::runAlgorithmReproduction()
    {
        if(!reproductionActive() || anyPredictionRunning()) {
            return;
        }
        m_reproductionDiagnostics.clear();
        m_reproductionGpuProgressBucket = -1;
        appendReproductionDiagnostic(QStringLiteral("Task preparation"),
            QStringLiteral("Preparing the selected scene and trajectory."));
        if(!m_reproductionSceneReady || !m_reproductionTrajectoryReady) {
            m_status = QStringLiteral(
                "Generate the scene and trajectory before running an algorithm.");
            appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
            refreshViewModel();
            return;
        }

        const bool generatedScene = m_panel.reproductionSceneSource()
            == ReproductionSceneSource::GeneratedPlateStack;
        const bool generatedTrajectory = m_panel.reproductionTrajectorySource()
            != ReproductionTrajectorySource::ImportedTrajectory;
        RobotQtViewerViewportServices* services = m_context.viewportServices();
        if(services == nullptr) {
            m_status = QStringLiteral(
                "The analysis model, trajectory, or viewport is unavailable.");
            appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
            refreshViewModel();
            return;
        }

        m_reproductionStartedAt = std::chrono::steady_clock::now();
        m_reproductionRunActive = true;
        m_reproductionTimingValid = false;
        m_reproductionFramePending = false;
        m_reproductionPreparationMilliseconds = 0.0;
        m_reproductionCoreMilliseconds = 0.0;
        m_reproductionConversionMilliseconds = 0.0;
        m_reproductionDisplayMilliseconds = 0.0;
        m_reproductionPresentationMilliseconds = 0.0;
        m_reproductionTotalMilliseconds = 0.0;
        m_reproductionInputVertexCount = 0;
        m_reproductionInputTriangleCount = 0;
        m_reproductionOutputVertexCount = 0;
        m_reproductionOutputTriangleCount = 0;

        try {
            sprayworkpiece::WorkpieceModel workpiece;
            PaintingAnalysisMeshBinding binding;
            std::shared_ptr<assetcore::ModelDesc> displayModel;
            std::filesystem::path sourcePath;
            QString predictionObjectId;

            if(generatedScene) {
                workpiece = m_reproductionPlateStack.workpiece;
                displayModel = m_reproductionPlateStack.displayModel;
                sourcePath = std::filesystem::path(
                    "algorithm-reproduction-plate-stack.stl");
                predictionObjectId = QString::fromLatin1(
                    kReproductionPlateStackObjectId);
                binding.sampleIndicesBySubMesh = { {} };
                binding.sampleIndicesBySubMesh.front().resize(
                    workpiece.samples.size());
                std::iota(binding.sampleIndicesBySubMesh.front().begin(),
                    binding.sampleIndicesBySubMesh.front().end(), std::size_t{ 0 });
            } else {
                const simulation_project::SceneObjectDesc* object =
                    findObject(m_context.document(), m_session.objectId);
                if(object == nullptr) {
                    throw std::runtime_error("The analysis model is unavailable.");
                }
                sourcePath = simulation_project::AssetResolver::resolveProjectPath(
                    makeResolveContext(m_context.projectSession()), object->sourcePath);
                std::string loadError;
                const std::shared_ptr<assetcore::ModelDesc> model =
                    assetcore::AssetManager::instance().tryLoadModel(
                        sourcePath.generic_u8string(),
                        static_cast<float>(object->visualScale),
                        &loadError);
                if(!model) {
                    throw std::runtime_error(loadError.empty()
                        ? "Failed to load the analysis mesh."
                        : loadError);
                }
                PaintingAnalysisMeshData mesh = PaintingAnalysisMeshAdapter::build(
                    *model, object->name, sourcePath.generic_u8string(),
                    makeTransform(object->transform));
                workpiece = std::move(mesh.workpiece);
                binding = std::move(mesh.binding);
                displayModel = std::move(mesh.displayModel);
                predictionObjectId = QString::fromStdString(object->id);
            }
            if(workpiece.empty()) {
                throw std::runtime_error("The model has no mesh vertices.");
            }

            const spraytrajectory::SprayTrajectory& trajectory = generatedTrajectory
                ? m_reproductionGeneratedTrajectory : m_session.trajectory;
            if(trajectory.empty()) {
                throw std::runtime_error("The reproduction trajectory is empty.");
            }
            const spraythickness::TrajectorySamplingMode samplingMode =
                spraythickness::TrajectorySamplingMode::OriginalPoints;
            const double timeStepSeconds = generatedTrajectory
                ? m_panel.reproductionGeneratedTrajectoryParameters()
                    .trajectoryPointIntervalSeconds
                : m_session.appliedTrajectoryTimeStepSeconds;
            const auto trajectoryPoints = trajectory.flattenedPoints();
            const std::size_t sprayingPoints = static_cast<std::size_t>(
                std::count_if(trajectoryPoints.begin(), trajectoryPoints.end(),
                    [](const auto& point) { return point.sprayEnabled; }));
            const double worldSprayAxisZ = trajectoryPoints.empty() ? 0.0
                : (trajectoryPoints.front().tcpPose.linear()
                    * m_panel.reproductionSprayDirectionLocal()).z();
            appendReproductionDiagnostic(QStringLiteral("Input prepared"),
                QStringLiteral("vertices=%1, triangles=%2, trajectory points=%3, "
                               "spraying points=%4, world spray axis Z=%5")
                    .arg(static_cast<qulonglong>(workpiece.samples.size()))
                    .arg(static_cast<qulonglong>(workpiece.triangleIndices.size() / 3))
                    .arg(static_cast<qulonglong>(trajectoryPoints.size()))
                    .arg(static_cast<qulonglong>(sprayingPoints))
                    .arg(worldSprayAxisZ, 0, 'f', 3));
            m_reproductionInputVertexCount = workpiece.samples.size();
            m_reproductionInputTriangleCount =
                workpiece.triangleIndices.size() / 3;

            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(predictionObjectId);
                if(!m_session.objectId.isEmpty()) {
                    services->clearSurfaceScalarOverlay(m_session.objectId);
                }
            }
            m_session.clearResult();
            m_session.binding = std::move(binding);
            m_session.predictionDisplayModel = std::move(displayModel);
            m_reproductionWorkpiece = workpiece;
            m_predictionObjectId = predictionObjectId;
            m_reproductionResultUsesGeneratedScene = generatedScene;
            m_predictionProgress = 0.1;
            m_predictionStartedAt = std::chrono::steady_clock::now();
            m_predictionTimerActive = true;

            const spraythickness::ReproductionAlgorithmKind algorithm =
                m_panel.reproductionAlgorithm();
            m_status = QStringLiteral("Running %1...")
                .arg(QString::fromLatin1(
                    spraythickness::reproductionAlgorithmName(algorithm)));
            appendReproductionDiagnostic(QStringLiteral("Algorithm"), m_status);
            refreshViewModel();
            publishStateChanged();

            QString startError;
            if(algorithm
                == spraythickness::ReproductionAlgorithmKind::CurrentMethod) {
                spraythickness::ThicknessPredictionTask task;
                task.model = m_panel.thicknessModel();
                task.workpiece = workpiece;
                task.trajectory = trajectory;
                task.tool.name = "Algorithm reproduction spray gun";
                task.tool.sprayDirectionLocal =
                    m_panel.reproductionSprayDirectionLocal();
                task.tool.powderFeedDirectionLocal =
                    m_panel.reproductionPowderFeedDirectionLocal();
                task.process.id = generatedTrajectory
                    ? "simulation"
                    : spraythickness::thicknessModelId(task.model);
                task.process.name = task.process.id;
                task.options.base.trajectorySamplingMode = samplingMode;
                task.options.base.timeStep = timeStepSeconds;
                task.options.enableBvhOcclusion = true;
                task.options.enableHistoryCorrection =
                    m_panel.reproductionHistoryCorrectionEnabled();
                m_reproductionPreparationMilliseconds =
                    elapsedMilliseconds(m_reproductionStartedAt);
                m_reproductionGpuRun = true;
                appendReproductionDiagnostic(QStringLiteral("GPU prediction"),
                    QStringLiteral("Full scene BVH occlusion enabled."));
                if(!m_predictionJob->start(std::move(task), &startError)) {
                    m_reproductionGpuRun = false;
                    throw std::runtime_error(startError.toStdString());
                }
            } else {
                spraycore::SprayTool tool;
                tool.name = "Algorithm reproduction spray gun";
                tool.sprayDirectionLocal =
                    m_panel.reproductionSprayDirectionLocal();
                tool.powderFeedDirectionLocal =
                    m_panel.reproductionPowderFeedDirectionLocal();
                PublishedReproductionRuntimeInputs runtimeInputs;
                runtimeInputs.modelSourcePath = sourcePath;
                runtimeInputs.generatedPlateStack = generatedScene;
                appendReproductionDiagnostic(QStringLiteral("Scene geometry"),
                    QStringLiteral("shared input vertices=%1, triangles=%2")
                        .arg(static_cast<qulonglong>(m_reproductionInputVertexCount))
                        .arg(static_cast<qulonglong>(m_reproductionInputTriangleCount)));
                appendReproductionDiagnostic(QStringLiteral("Calibration file"),
                    m_panel.reproductionConfigurationPath());
                spraythickness::AlgorithmReproductionTask task =
                    PublishedReproductionAdapter::buildTask(
                        algorithm,
                        workpiece,
                        trajectory,
                        tool,
                        samplingMode,
                        timeStepSeconds,
                        std::filesystem::path(
                            m_panel.reproductionConfigurationPath().toStdWString()),
                        runtimeInputs);
                m_reproductionPreparationMilliseconds =
                    elapsedMilliseconds(m_reproductionStartedAt);
                if(!m_reproductionJob->start(std::move(task), &startError)) {
                    throw std::runtime_error(startError.toStdString());
                }
            }
        } catch(const std::exception& exception) {
            m_reproductionRunActive = false;
            m_predictionObjectId.clear();
            m_predictionProgress = 0.0;
            m_predictionTimerActive = false;
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
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
        if(m_reproductionBenchmarkActive) {
            m_reproductionBenchmarkActive = false;
            recordReproductionBenchmarkRun(QStringLiteral("canceled"),
                QStringLiteral("Canceled by user."));
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
        m_predictionProgress = 0.1 + 0.8 * std::clamp(progress, 0.0, 1.0);
        m_status = message;
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::handleReproductionDiagnostic(
        const QString& stage, const QString& details)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        appendReproductionDiagnostic(stage, details);
        refreshViewModel();
    }

    void CoatingAnalysisModuleController::appendReproductionDiagnostic(
        const QString& stage, const QString& details)
    {
        m_reproductionDiagnostics.append(QStringLiteral("[%1] %2")
            .arg(stage, details));
        constexpr int maximumEntries = 40;
        while(m_reproductionDiagnostics.size() > maximumEntries) {
            m_reproductionDiagnostics.removeFirst();
        }
        m_reproductionStatus = m_reproductionDiagnostics.join(
            QStringLiteral("\n"));
    }

    void CoatingAnalysisModuleController::handleReproductionFinished(
        const spraythickness::AlgorithmReproductionResult& result)
    {
        if(spraythickness::reproductionCanceled(result)) {
            m_reproductionRunActive = false;
            m_predictionObjectId.clear();
            m_predictionProgress = 0.0;
            m_predictionTimerActive = false;
            m_session.clearResult();
            m_status = QStringLiteral("Algorithm reproduction canceled.");
            appendReproductionDiagnostic(QStringLiteral("Canceled"), m_status);
            recordReproductionBenchmarkRun(QStringLiteral("canceled"), m_status);
            refreshViewModel();
            publishStateChanged();
            return;
        }
        PublishedReproductionDisplayData display;
        const auto conversionStartedAt = std::chrono::steady_clock::now();
        appendReproductionDiagnostic(QStringLiteral("Display conversion"),
            QStringLiteral("Converting the algorithm result to a scalar field."));
        try {
            display = PublishedReproductionDisplayAdapter::build(
                result, m_reproductionWorkpiece, m_session.binding,
                m_reproductionResultUsesGeneratedScene
                    ? m_panel.wuNormalDisplayScale() : 1.0);
        } catch(const std::exception& exception) {
            handleReproductionFailed(QStringLiteral(
                "Reproduction display conversion failed: %1")
                    .arg(QString::fromLocal8Bit(exception.what())));
            return;
        }
        m_reproductionConversionMilliseconds =
            elapsedMilliseconds(conversionStartedAt);
        m_reproductionOutputVertexCount =
            display.scalarField.field.results.size();
        m_reproductionOutputTriangleCount = 0;
        if(display.displayModel) {
            for(const auto& subMesh : display.displayModel->subMeshes()) {
                m_reproductionOutputTriangleCount +=
                    subMesh.geometry.indices.size() / 3;
            }
        }
        appendReproductionDiagnostic(QStringLiteral("Output geometry"),
            QStringLiteral("display vertices=%1, triangles=%2")
                .arg(static_cast<qulonglong>(m_reproductionOutputVertexCount))
                .arg(static_cast<qulonglong>(m_reproductionOutputTriangleCount)));
        appendReproductionDiagnostic(QStringLiteral("Converted field"),
            thicknessFieldDiagnostic(display.scalarField.field));
        m_predictionProgress = 0.95;
        m_reproductionCoreMilliseconds =
            spraythickness::reproductionStatistics(result).elapsedMilliseconds;
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
        const QString completion = QStringLiteral(
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
        appendReproductionDiagnostic(QStringLiteral("Completed"), completion);
        for(const std::string& note :
            spraythickness::reproductionImplementationNotes(result)) {
            appendReproductionDiagnostic(QStringLiteral("Implementation note"),
                QString::fromStdString(note));
        }
        m_status = m_reproductionStatus;
        m_predictionProgress = 0.95;
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
        m_reproductionRunActive = false;
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        m_session.clearResult();
        m_status = message.isEmpty()
            ? QStringLiteral("Algorithm reproduction failed.")
            : message;
        appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
        recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 5000);
    }

    void CoatingAnalysisModuleController::applyWuDisplayScale()
    {
        if(!reproductionActive() || anyPredictionRunning()
            || !m_reproductionResultUsesGeneratedScene
            || !m_session.hasResult || !m_session.hasReproductionResult
            || m_session.reproduction.algorithm
                != spraythickness::ReproductionAlgorithmKind::Wu2020) {
            return;
        }
        try {
            const auto display = PublishedReproductionDisplayAdapter::build(
                m_session.reproduction, m_reproductionWorkpiece,
                m_session.binding, m_panel.wuNormalDisplayScale());
            const auto previousModel = m_session.predictionDisplayModel;
            m_session.predictionDisplayModel = display.displayModel;
            QString error;
            if(!applyCurrentSurfaceOverlay(&error)) {
                m_session.predictionDisplayModel = previousModel;
                applyCurrentSurfaceOverlay(nullptr);
                throw std::runtime_error(error.toStdString());
            }
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setSurfaceScalarOverlayVisible(
                    activeCoatingObjectId(), m_session.showThickness);
            }
            appendReproductionDiagnostic(QStringLiteral("Display height"),
                QStringLiteral("Normal height scale=%1x; physical thickness unchanged.")
                    .arg(m_panel.wuNormalDisplayScale(), 0, 'f', 1));
            refreshViewModel();
            publishStateChanged();
        } catch(const std::exception& exception) {
            emit statusMessageRequested(
                QString::fromLocal8Bit(exception.what()), 5000);
        }
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
        task.options.base.trajectorySamplingMode =
            m_session.appliedTrajectorySamplingMode;
        task.options.base.timeStep =
            m_session.appliedTrajectoryTimeStepSeconds;
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
        m_predictionProgress = m_reproductionGpuRun
            ? 0.1 + 0.8 * std::clamp(progress, 0.0, 1.0)
            : std::clamp(progress, 0.0, 1.0);
        if(m_reproductionGpuRun) {
            const int bucket = std::clamp(static_cast<int>(progress * 10.0), 0, 10);
            if(bucket != m_reproductionGpuProgressBucket) {
                m_reproductionGpuProgressBucket = bucket;
                appendReproductionDiagnostic(QStringLiteral("GPU progress"),
                    QStringLiteral("%1%: %2").arg(bucket * 10).arg(message));
            }
        }
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
        m_predictionProgress = m_reproductionRunActive ? 0.95 : 0.0;

        if(prediction.field.empty()) {
            m_reproductionRunActive = false;
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = prediction.warnings.empty()
                ? QStringLiteral("GPU thickness prediction produced no result.")
                : QString::fromStdString(prediction.warnings.front());
            if(reproductionGpu) {
                appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
                recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
            }
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
            return;
        }

        RobotQtViewerViewportServices* services = m_context.viewportServices();
        const bool generatedObject =
            objectId == QString::fromLatin1(kSimulationPlateObjectId)
            || objectId == QString::fromLatin1(kReproductionPlateStackObjectId);
        if(services == nullptr || (!generatedObject
                && findObject(m_context.document(), objectId) == nullptr)) {
            m_reproductionRunActive = false;
            m_reproductionGpuRun = false;
            m_session.clearResult();
            m_status = QStringLiteral("The analysis model or viewport is no longer available.");
            if(reproductionGpu) {
                appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
                recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
            }
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
            return;
        }

        const auto displayStartedAt = std::chrono::steady_clock::now();
        if(m_reproductionRunActive) {
            appendReproductionDiagnostic(QStringLiteral("Computed field"),
                thicknessFieldDiagnostic(prediction.field));
        }
        if(reproductionGpu) {
            appendReproductionDiagnostic(QStringLiteral("Display upload"),
                QStringLiteral("Uploading the computed thickness field."));
        }
        try {
            smrobot::visualization::SurfaceScalarOverlay overlay =
                PaintingAnalysisMeshAdapter::makeOverlay(
                    objectId.toStdString(),
                    m_session.binding,
                    prediction);
            if(m_reproductionRunActive) {
                std::size_t overlayVertices = 0;
                std::size_t overlayNonzero = 0;
                for(const auto& subMesh : overlay.subMeshes) {
                    overlayVertices += subMesh.values.size();
                    overlayNonzero += static_cast<std::size_t>(std::count_if(
                        subMesh.values.begin(), subMesh.values.end(),
                        [](double value) { return value != 0.0; }));
                }
                appendReproductionDiagnostic(QStringLiteral("Display field"),
                    QStringLiteral("vertices=%1, nonzero=%2, range=[%3, %4] um")
                        .arg(static_cast<qulonglong>(overlayVertices))
                        .arg(static_cast<qulonglong>(overlayNonzero))
                        .arg(overlay.range.minimum * kMetersToMicrometers, 0, 'g', 9)
                        .arg(overlay.range.maximum * kMetersToMicrometers, 0, 'g', 9));
            }
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
                m_reproductionRunActive = false;
                m_reproductionGpuRun = false;
                m_session.clearResult();
                m_status = applyError.isEmpty()
                    ? QStringLiteral("Failed to display the thickness result.")
                    : applyError;
                if(reproductionGpu) {
                    appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
                }
                recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
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
                current.bvhOcclusion = true;
                current.historyCorrection =
                    m_panel.reproductionHistoryCorrectionEnabled();
                current.statistics.elapsedMilliseconds = prediction.timing.valid
                    ? prediction.timing.backendTotalMilliseconds
                    : elapsedSeconds * 1000.0;
                current.statistics.evaluatedElementCount =
                    prediction.field.results.size();
                current.statistics.trajectorySampleCount =
                    prediction.timing.sprayPointCount;
                m_session.reproduction.nativeResult = std::move(current);
                m_session.hasReproductionResult = true;
                m_reproductionOutputVertexCount =
                    prediction.field.results.size();
                m_reproductionOutputTriangleCount =
                    m_reproductionWorkpiece.triangleIndices.size() / 3;
            }
            // Ensure the prediction workpiece node is visible so the scalar
            // overlay model actually renders.
            services->setCoatingModelVisible(objectId, true);
            services->setSurfaceScalarOverlayVisible(objectId, true);
            services->setSurfaceScalarProbeEnabled(false, QString());
            if(m_reproductionRunActive) {
                if(reproductionGpu) {
                    m_reproductionCoreMilliseconds =
                        spraythickness::reproductionStatistics(
                            m_session.reproduction).elapsedMilliseconds;
                }
                m_reproductionDisplayMilliseconds =
                    elapsedMilliseconds(displayStartedAt);
                m_reproductionOverlayReadyAt = std::chrono::steady_clock::now();
                m_reproductionFramePending = true;
                m_reproductionRunActive = false;
                if(m_reproductionBenchmarkRunPending) {
                    const auto overlayReadyAt = m_reproductionOverlayReadyAt;
                    QTimer::singleShot(15000, this, [this, overlayReadyAt]() {
                        if(m_reproductionBenchmarkRunPending
                            && m_reproductionFramePending
                            && m_reproductionOverlayReadyAt == overlayReadyAt) {
                            m_reproductionFramePending = false;
                            recordReproductionBenchmarkRun(
                                QStringLiteral("first_frame_timeout"),
                                QStringLiteral("No viewport frame after overlay upload."));
                        }
                    });
                }
            }
            m_status = QStringLiteral("GPU thickness prediction completed in %1 s.")
                .arg(elapsedSeconds, 0, 'f', 3);
            if(reproductionGpu) {
                const QString completion = QStringLiteral(
                    "Current method (GPU) completed in %1 s.\n"
                    "Output: vertices; elements: %2; spray samples: %3.")
                    .arg(elapsedSeconds, 0, 'f', 3)
                    .arg(static_cast<qulonglong>(prediction.field.results.size()))
                    .arg(static_cast<qulonglong>(prediction.timing.sprayPointCount));
                appendReproductionDiagnostic(QStringLiteral("Completed"), completion);
                m_status = completion;
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
            m_reproductionRunActive = false;
            m_reproductionGpuRun = false;
            services->clearSurfaceScalarOverlay(objectId);
            if(objectId != QString::fromLatin1(
                    kReproductionPlateStackObjectId)) {
                services->clearCoatingPredictionModel(objectId);
            }
            m_session.clearResult();
            m_status = QString::fromLocal8Bit(exception.what());
            if(reproductionGpu) {
                appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
            }
            recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
            if(objectId == QString::fromLatin1(kSimulationPlateObjectId)) {
                m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
            }
            refreshViewModel();
            publishStateChanged();
        }
    }

    void CoatingAnalysisModuleController::handleOnlineFramePresented(
        quint64 frameId, bool displayed)
    {
        const auto presentedAt = std::chrono::steady_clock::now();
        if(frameId == 0 || frameId != m_onlinePendingFrameId) return;
        m_onlinePendingFrameId = 0;
        m_onlineLastAcknowledgedFrameId = frameId;
        if(displayed && m_active && onlineModeActive()) {
            const auto now = presentedAt;
            if(m_onlineLastPresentedAt != std::chrono::steady_clock::time_point{}) {
                const double intervalMilliseconds = std::chrono::duration<double, std::milli>(
                    now - m_onlineLastPresentedAt).count();
                m_onlineFrameIntervals.push_back(intervalMilliseconds);
                // Lifetime maximum for this spray run; periodic P95 samples are separate.
                const bool newMaximum = intervalMilliseconds > m_onlineMaximumFrameIntervalMilliseconds;
                m_onlineMaximumFrameIntervalMilliseconds = std::max(
                    m_onlineMaximumFrameIntervalMilliseconds, intervalMilliseconds);
                // Capture the actual slow frame, not just the ordinary frame
                // that happens to fall on the once-per-second sampling clock.
                if(intervalMilliseconds >= 50.0
                    && (newMaximum || now - m_onlineLastLongFrameLog >= std::chrono::seconds(1))) {
                    m_onlineLastLongFrameLog = now;
                    LOG_DEBUG("rs2026") << "Online long frame: frameId=" << frameId
                        << ", intervalMs=" << intervalMilliseconds
                        << ", deliveryWaitMs=" << m_onlineDeliveryWaitMilliseconds
                        << ", pacingWaitMs=" << m_onlinePacingWaitMilliseconds
                        << ", applyMs=" << m_onlineApplyMilliseconds
                        << ", presentWaitMs=" << std::chrono::duration<double, std::milli>(
                            presentedAt - m_onlinePresentationRequestedAt).count()
                        << ", backendMs=" << (m_onlineResult
                            ? m_onlineResult->timing.backendTotalMilliseconds : 0.0)
                        << ", statisticsMs=" << m_onlineStatisticsMilliseconds
                        << ", poseMs=" << m_onlinePoseMilliseconds
                        << ", mappingMs=" << m_onlineMappingMilliseconds
                        << ", workerMappingMs=" << m_onlineWorkerMappingMilliseconds
                        << ", overlayMs=" << m_onlineOverlayMilliseconds
                        << ", renderMs=" << m_onlineLastRenderMilliseconds;
                }
            }
            m_onlineLastPresentedAt = now;
            if(now - m_onlineLastPresentationLog >= std::chrono::seconds(1)) {
                m_onlineLastPresentationLog = now;
                LOG_DEBUG("rs2026") << "Online presentation: waitMs="
                    << std::chrono::duration<double, std::milli>(
                        presentedAt - m_onlinePresentationRequestedAt).count()
                    << ", renderMs=" << m_onlineLastRenderMilliseconds
                    << ", virtualIntervalMs=" << m_onlineVirtualTimer->interval()
                    << ", displayIntervalMs=" << m_onlineRefreshCadence.displayIntervalMilliseconds()
                    << ", screenRefreshHz=" << m_onlineScreenRefreshRate;
            }
            ++m_onlineSceneFrameCount;
            if(m_onlineThicknessFramePending && m_onlineShowThickness) {
                ++m_onlineThicknessFrameCount;
                if(m_onlineWaitingForFirstFrame) {
                    m_onlineWaitingForFirstFrame = false;
                    m_onlineFirstFrameMilliseconds = std::chrono::duration<double, std::milli>(
                        presentedAt - m_onlineSprayRequestedAt).count();
                    LOG_DEBUG("rs2026") << "Online first cloud frame: totalMs="
                        << m_onlineFirstFrameMilliseconds;
                    updateOnlineRefreshStatistics();
                }
            }
        }
        if(m_onlineDiagnosticFrameEligible && m_onlineDiagnosticFrame.id == frameId) {
            m_onlineDiagnosticFrame.displayed = displayed && m_active && onlineModeActive();
            m_onlineDiagnosticFrame.inputToPresentMilliseconds = std::chrono::duration<double, std::milli>(
                presentedAt - m_onlineDiagnosticSubmittedAt).count();
            if(m_onlineDiagnosticRenderedAt != std::chrono::steady_clock::time_point{}) {
                m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::SwapWait) = std::chrono::duration<double, std::milli>(
                    presentedAt - m_onlineDiagnosticRenderedAt).count();
            }
            m_onlineDiagnostics->record(m_onlineDiagnosticFrame, presentedAt);
            m_onlineDiagnosticFrameEligible = false;
        }
        if(!m_onlineSpraying && frameId == m_onlineLastSubmittedFrameId) {
            m_onlineFinishing = false;
            if(!m_onlineDisplayedTool.sprayDirectionLocal.isApprox(m_onlineTool.sprayDirectionLocal)
                || !m_onlineDisplayedTool.powderFeedDirectionLocal.isApprox(m_onlineTool.powderFeedDirectionLocal)) {
                m_onlineDisplayedTool = m_onlineTool;
                updateOnlinePoseDisplay();
            }
            if(m_onlineMotionParametersPending) applyOnlineMotionParameters();
            m_onlineWaitingForFirstFrame = false;
            m_onlineStatus = onlineStoppedStatus();
            m_panel.setOnlinePredictionState(m_onlineActive, false, m_onlineStatus);
            if(m_active && onlineModeActive()) {
                m_status = m_onlineStatus;
                refreshViewModel();
            }
            m_onlineDiagnostics->finish(QStringLiteral("final frame processed"));
            m_onlineDiagnosticTimer->stop();
            m_panel.setOnlineDiagnostics(m_onlineDiagnostics->summary(), m_onlineDiagnostics->filePath());
            resumeOnlineVirtualClock();
        }
        m_onlineThicknessFramePending = false;
        m_onlineJob->acknowledgeFrame(frameId);
    }

    void CoatingAnalysisModuleController::handleOnlineFrameRenderProfile(
        quint64 frameId, double sceneUpdateMilliseconds, double drawMilliseconds)
    {
        if(m_onlineDiagnosticFrameEligible && frameId == m_onlineDiagnosticFrame.id
            && m_onlineDiagnosticRenderedAt == std::chrono::steady_clock::time_point{}) {
            const auto renderedAt = std::chrono::steady_clock::now();
            // Profile delivery is direct on the GUI thread. Subtract measured
            // paint work to isolate the request-to-paint scheduling delay.
            m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::PaintQueue) = std::max(0.0,
                std::chrono::duration<double, std::milli>(renderedAt - m_onlinePresentationRequestedAt).count()
                    - sceneUpdateMilliseconds - drawMilliseconds);
            m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::SceneUpdate) = sceneUpdateMilliseconds;
            m_onlineDiagnosticFrame.at(OnlineDiagnosticStage::DrawSubmission) = drawMilliseconds;
            m_onlineDiagnosticRenderedAt = renderedAt;
        }
    }

    void CoatingAnalysisModuleController::handleOnlineViewportFrameRendered(double milliseconds)
    {
        if(m_active && onlineModeActive()) {
            ++m_onlineViewportFrameCount;
            m_onlineLastRenderMilliseconds = milliseconds;
        }
    }

    void CoatingAnalysisModuleController::handleViewportFrameSwapped()
    {
        if(!m_reproductionFramePending || !reproductionActive()
            || !m_session.hasReproductionResult) {
            return;
        }
        m_reproductionFramePending = false;
        m_reproductionPresentationMilliseconds =
            elapsedMilliseconds(m_reproductionOverlayReadyAt);
        m_reproductionTotalMilliseconds =
            elapsedMilliseconds(m_reproductionStartedAt);
        m_reproductionTimingValid = true;
        m_predictionProgress = 1.0;
        appendReproductionDiagnostic(QStringLiteral("First frame"),
            QStringLiteral("Result presented after %1 ms total.")
                .arg(m_reproductionTotalMilliseconds, 0, 'f', 3));
        refreshViewModel();
        const bool hasThickness = m_session.hasResult
            && activeThicknessCount(m_session.prediction.field) != 0;
        recordReproductionBenchmarkRun(
            hasThickness ? QStringLiteral("success")
                         : QStringLiteral("empty_field"),
            hasThickness ? QString()
                         : QStringLiteral("Displayed thickness field is zero."));
    }

    void CoatingAnalysisModuleController::handlePredictionFailed(const QString& message)
    {
        if(m_predictionObjectId.isEmpty()) {
            return;
        }
        const QString objectId = m_predictionObjectId;
        const bool simulation =
            objectId == QString::fromLatin1(kSimulationPlateObjectId);
        const bool reproductionGpu = m_reproductionGpuRun;
        const bool generatedReproduction = reproductionGpu
            && objectId == QString::fromLatin1(kReproductionPlateStackObjectId);
        m_reproductionGpuRun = false;
        m_reproductionRunActive = false;
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->clearSurfaceScalarOverlay(objectId);
            if(!generatedReproduction) {
                services->clearCoatingPredictionModel(objectId);
            }
        }
        m_session.clearResult();
        m_status = message.isEmpty()
            ? QStringLiteral("GPU thickness prediction failed.")
            : message;
        if(simulation) {
            m_simulationStatus = m_status + QStringLiteral("\n") + m_simulationReadyStatus;
        } else if(reproductionGpu) {
            appendReproductionDiagnostic(QStringLiteral("Failed"), m_status);
        }
        if(reproductionGpu) {
            recordReproductionBenchmarkRun(QStringLiteral("failed"), m_status);
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
        if(onlineModeActive()) {
            m_onlineShowThickness = enabled;
            if(!enabled) {
                m_onlinePickEnabled = false;
            }
            if(enabled && m_onlineResult) restoreOnlineDisplay();
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                for(const OnlineObject& object : m_onlineObjects) {
                    services->setSurfaceScalarOverlayVisible(object.id, enabled);
                }
                services->setSurfaceScalarProbeEnabled(
                    m_onlinePickEnabled, m_session.objectId);
            }
            refreshViewModel();
            return;
        }
        m_session.showThickness = enabled && m_session.hasResult;
        if(!m_session.showThickness) {
            m_session.thicknessPickEnabled = false;
        }
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            const QString objectId = activeCoatingObjectId();
            services->setSurfaceScalarProbeEnabled(
                m_active && m_session.showThickness && m_session.thicknessPickEnabled,
                objectId);
            if(m_session.hasResult) {
                if(!services->setSurfaceScalarOverlayVisible(
                        objectId,
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
        if(onlineModeActive()) {
            m_onlinePickEnabled = enabled && m_onlineShowThickness
                && (m_onlineResult && !m_onlineResult->empty());
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setSurfaceScalarProbeEnabled(
                    m_onlinePickEnabled, m_session.objectId);
            }
            refreshViewModel();
            return;
        }
        m_session.thicknessPickEnabled = enabled && m_active &&
            m_session.hasResult && m_session.showThickness;
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(
                m_session.thicknessPickEnabled,
                m_session.thicknessPickEnabled
                    ? activeCoatingObjectId() : QString());
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::selectWorkpiece(const QString& objectId)
    {
        if(onlineModeActive() && objectId != m_predictionSession.objectId) {
            return;
        }
        if(onlineModeActive() && m_onlineActive
            && objectId != m_session.objectId) {
            resetOnlinePrediction();
        }
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
        const bool replacingSavedTrajectory = usesSavedTrajectoryPlan() ||
            usesOptimizationBaselineTrajectory() || usesOptimizedTrajectory();
        m_session.clearResult();
        if(replacingSavedTrajectory) {
            m_session.trajectory = spraytrajectory::SprayTrajectory();
            m_session.waypoints.clear();
            m_session.trajectoryName.clear();
            m_session.trajectoryPath.clear();
            m_session.trajectoryInfo = CoatingAnalysisTrajectoryInfo();
            m_treePanel.setWaypoints(nullptr);
            if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
                services->setCoatingTrajectoryPreview({}, false);
            }
        }
        m_session.manualThicknessRange = false;
        m_session.minimumDisplayThicknessMeters = 0.0;
        m_session.maximumDisplayThicknessMeters = 0.0;
        if(m_mode == CoatingAnalysisMode::Prediction) {
            resetReferenceResult();
        }
        m_session.objectId = objectId;
        m_session.modelName = QString::fromStdString(object->name);
        m_session.sourcePath = QString::fromStdString(object->sourcePath);
        m_session.modelInfo = CoatingAnalysisModelInfo();
        if(m_mode == CoatingAnalysisMode::Prediction) {
            m_hasRotationAxis = false;
            m_rotationPreviewWorkpiece = sprayworkpiece::WorkpieceModel();
            m_rotationSurfaceTriangleIndices.clear();
            m_rotationSeedTriangleIndex = 0;
            m_rotationFitElapsedMilliseconds = 0.0;
            m_hasLocalPreview = false;
            m_localPreviewDetails.clear();
            clearAxisymmetricProfileSelection();
            m_panel.setPeriodicLocalPredictionEnabled(false);
        }
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
        if(reproductionActive()) {
            m_reproductionSceneReady = false;
            m_reproductionSceneDetails = QStringLiteral(
                "Scene model changed. Generate the scene again.");
            updateReproductionPreparationStatus();
        }
        m_status = QStringLiteral("Workpiece selected: %1")
            .arg(m_session.modelName);
        if(!onlineModeActive()) {
            loadSavedTrajectoryPlanIfAvailable();
        }
        refreshViewModel();
        publishStateChanged();
    }

    void CoatingAnalysisModuleController::selectWorkpieceFromTree(const QString& objectId)
    {
        if(simulationActive()) {
            return;
        }
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
        const bool generatedSceneSelected = generatedReproductionSceneActive();
        const bool generatedScene = generatedSceneSelected
            && m_reproductionGeneratedPreviewVisible;
        if(!m_active || (m_mode == CoatingAnalysisMode::Prediction
                && m_session.objectId.isEmpty())) {
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
            if(object.objectType != "workpiece" && !onlineModeActive()) {
                continue;
            }
            const QString objectId = QString::fromStdString(object.id);
            const bool requestedVisibility = onlineModeActive()
                ? (objectId == m_session.objectId
                    || (m_onlineActive && !m_onlineVirtualSource
                        && object.objectType == "fixture"))
                :
                (generatedSceneSelected || (reproductionActive()
                    && m_session.objectId.isEmpty()))
                ? false
                : (m_modelVisibility.contains(objectId)
                    ? m_modelVisibility.value(objectId)
                    : (objectId == m_session.objectId && m_session.showModel));
            // Keep the original STL visible during local preview. The local
            // prediction sector is rendered separately as a green overlay;
            // keeping the source object visible preserves exact picking and
            // rotation-center behavior without changing prediction inputs.
            visibility[objectId] = requestedVisibility;
        }
        services->setCoatingModelVisibilities(visibility);
        if(generatedScene) {
            services->setCoatingModelVisible(
                QString::fromLatin1(kReproductionPlateStackObjectId),
                m_session.showModel);
        }
        for(const QString& objectId : visibility.keys()) {
            m_modelVisibilityOverrideIds.push_back(objectId);
        }
    }

    void CoatingAnalysisModuleController::clearSession()
    {
        resetOnlinePrediction();
        m_predictionJob->cancel();
        m_reproductionJob->cancel();
        clearReproductionGeneratedPreview();
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
        if(reproductionActive()) {
            m_mode = CoatingAnalysisMode::Prediction;
        }
        if(onlineModeActive()) {
            m_mode = CoatingAnalysisMode::Prediction;
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
        m_predictionSession.clear();
        m_simulationSession.clear();
        m_reproductionSession.clear();
        m_onlineSession.clear();
        m_predictionModelVisibility.clear();
        m_simulationModelVisibility.clear();
        m_reproductionModelVisibility.clear();
        m_onlineModelVisibility.clear();
        m_predictionStatus.clear();
        m_simulationReady = false;
        m_simulation = SimulationExperimentData();
        m_simulationReadyStatus.clear();
        m_simulationStatus.clear();
        m_reproductionWorkpiece = sprayworkpiece::WorkpieceModel();
        m_reproductionPlateStack = PlateStackData();
        m_reproductionGeneratedTrajectory = spraytrajectory::SprayTrajectory();
        m_reproductionSceneReady = false;
        m_reproductionTrajectoryReady = false;
        m_reproductionSetupInitialized = false;
        m_reproductionResultUsesGeneratedScene = false;
        m_reproductionSceneDetails = QStringLiteral("Scene: not generated.");
        m_reproductionTrajectoryDetails =
            QStringLiteral("Trajectory: not generated.");
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
        if(onlineModeActive()) {
            services->setCoatingTrajectoryPreviewVisible(false);
            return;
        }
        const bool generatedTrajectory = generatedReproductionTrajectoryActive()
            && m_reproductionTrajectoryReady
            && !m_reproductionGeneratedTrajectory.empty();
        const bool hasTrajectory = generatedTrajectory
            || !m_session.waypoints.empty();
        const bool visible = m_active && hasTrajectory
            && (m_session.showTrajectory || m_session.showSprayPoints);
        if(visible && !services->setCoatingTrajectoryPreviewVisible(
                true,
                m_session.showTrajectory,
                m_session.showSprayPoints)) {
            if(generatedTrajectory) {
                submitReproductionTrajectoryPreview(
                    m_reproductionGeneratedTrajectory);
            } else {
                submitTrajectoryPreview();
            }
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

        std::vector<CoatingTrajectoryPreviewPoint> previewPoints;
        const auto appendPreviewPoint = [this, &previewPoints](
            const auto& point,
            bool sprayEnabled,
            bool startsNewSegment) {
            const Eigen::Vector3d position = point.tcpPose.translation();
            const Eigen::Matrix3d& rotation = point.tcpPose.linear();
            const Eigen::Vector3d direction =
                rotation * m_panel.sprayDirectionLocal();
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
            previewPoint.sprayEnabled = sprayEnabled;
            previewPoint.startsNewSegment = startsNewSegment;
            previewPoints.push_back(previewPoint);
        };

        if(m_session.trajectorySamplingApplied
            && !m_session.trajectoryPreviewSamples.empty()) {
            previewPoints.reserve(m_session.trajectoryPreviewSamples.size());
            std::vector<double> segmentStartTimes;
            segmentStartTimes.reserve(m_session.trajectory.segments.size());
            for(const spraytrajectory::SpraySegment& segment :
                m_session.trajectory.segments) {
                if(!segment.points.empty()) {
                    segmentStartTimes.push_back(segment.points.front().time);
                }
            }
            std::sort(segmentStartTimes.begin(), segmentStartTimes.end());

            const auto containsTime = [](const std::vector<double>& values,
                                         double time) {
                const auto value = std::lower_bound(values.begin(), values.end(), time);
                const double tolerance = std::max(1.0, std::abs(time)) * 1.0e-12;
                return (value != values.end() && std::abs(*value - time) <= tolerance)
                    || (value != values.begin()
                        && std::abs(*(value - 1) - time) <= tolerance);
            };
            for(const spraytrajectory::SprayTrajectorySample& sample :
                m_session.trajectoryPreviewSamples) {
                appendPreviewPoint(
                    sample,
                    sample.sprayEnabled,
                    containsTime(segmentStartTimes, sample.time));
            }
        } else {
            previewPoints.reserve(m_session.waypoints.size());
            for(const spraytrajectory::SpraySegment& segment :
                m_session.trajectory.segments) {
                bool startsNewSegment = true;
                for(const spraytrajectory::SprayPathPoint& point : segment.points) {
                    appendPreviewPoint(
                        point,
                        point.sprayEnabled && segment.sprayEnabled,
                        startsNewSegment);
                    startsNewSegment = false;
                }
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
        const QString objectId = activeCoatingObjectId();
        const bool generatedObject = objectId
            == QString::fromLatin1(kSimulationPlateObjectId)
            || objectId == QString::fromLatin1(kReproductionPlateStackObjectId);
        if(!m_session.hasResult || !m_active || !m_session.showThickness ||
            (!generatedObject
                && findObject(m_context.document(), objectId) == nullptr)) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            QString error;
            const bool applied = applyCurrentSurfaceOverlay(&error);
            if(applied) {
                services->setSurfaceScalarProbeEnabled(
                    m_session.showThickness && m_session.thicknessPickEnabled,
                    objectId);
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
        const bool onlineHasField = onlineModeActive()
            && (m_onlineResult && !m_onlineResult->empty());
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
        viewModel.trajectorySamplingApplyRequired =
            m_session.trajectorySamplingDirty;
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
            && !viewModel.trajectorySamplingApplyRequired
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
            && !viewModel.trajectorySamplingApplyRequired
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
        viewModel.progress = reproductionActive()
                && m_session.hasReproductionResult && m_reproductionTimingValid
            ? 1.0 : m_predictionProgress;
        viewModel.showModel = m_session.showModel;
        viewModel.showTrajectory = m_session.showTrajectory;
        viewModel.showSprayPoints = m_session.showSprayPoints;
        viewModel.showThickness = onlineModeActive()
            ? m_onlineShowThickness : m_session.showThickness;
        viewModel.thicknessPickEnabled = onlineModeActive()
            ? m_onlinePickEnabled : m_session.thicknessPickEnabled;
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
            && m_reproductionRunActive;
        const bool reproductionHasModel = m_panel.reproductionSceneSource()
                == ReproductionSceneSource::GeneratedPlateStack
            || viewModel.hasModel;
        const bool reproductionHasTrajectory =
            m_panel.reproductionTrajectorySource()
                != ReproductionTrajectorySource::ImportedTrajectory
            || viewModel.hasTrajectory;
        viewModel.canRunReproduction = reproductionActive()
            && reproductionHasModel && reproductionHasTrajectory
            && m_reproductionSceneReady && m_reproductionTrajectoryReady;
        viewModel.canExportReproduction = reproductionActive()
            && m_session.hasReproductionResult;
        viewModel.reproductionDetails = m_reproductionStatus;
        const QString reproductionLog = m_reproductionDiagnostics.join(
            QStringLiteral("\n"));
        if(!reproductionLog.isEmpty()
            && m_reproductionStatus != reproductionLog) {
            viewModel.reproductionDetails += QStringLiteral("\n\n")
                + reproductionLog;
        }
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
        if(onlineHasField) {
            viewModel.minimumMicrometers =
                m_onlineResult->metrics.minThickness * kMetersToMicrometers;
            viewModel.maximumMicrometers =
                m_onlineResult->metrics.maxThickness * kMetersToMicrometers;
            viewModel.midpointMicrometers =
                (viewModel.minimumMicrometers + viewModel.maximumMicrometers) * 0.5;
            viewModel.averageMicrometers =
                m_onlineResult->metrics.averageThickness * kMetersToMicrometers;
        }
        m_panel.applyViewModel(viewModel);

        CoatingAnalysisTreeView treeView;
        treeView.trajectoryName = m_session.trajectoryName;
        treeView.trajectoryInfo = m_session.trajectoryInfo;
        treeView.workpieces = viewModel.workpieces;
        treeView.selectedWorkpieceId = m_session.objectId;
        treeView.showModel = m_session.showModel;
        treeView.hasThickness = onlineHasField || m_session.hasResult;
        if(treeView.hasThickness) {
            treeView.thicknessMetrics = onlineHasField
                ? m_onlineResult->metrics : m_session.prediction.metrics;
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
        infoView.trajectorySamplingApplied =
            m_session.trajectorySamplingApplied;
        infoView.trajectorySamplingTimeStepSeconds =
            m_session.appliedTrajectoryTimeStepSeconds;
        infoView.trajectoryControlPointCount =
            m_session.trajectoryControlPointCount;
        infoView.trajectoryInterpolatedPointCount =
            m_session.trajectoryInterpolatedPointCount;
        infoView.trajectorySamplePointCount =
            m_session.trajectorySamplingApplied
            ? m_session.trajectoryPreviewSamples.size()
            : m_session.waypoints.size();
        infoView.trajectoryEffectiveSprayDurationSeconds =
            m_session.trajectoryEffectiveSprayDurationSeconds;
        infoView.hasThickness = onlineHasField || m_session.hasResult;
        infoView.predictionElapsedSeconds = m_session.predictionElapsedSeconds;
        infoView.reproductionTimingValid = reproductionActive()
            && m_session.hasReproductionResult && m_reproductionTimingValid;
        infoView.reproductionFramePending = reproductionActive()
            && m_session.hasReproductionResult && m_reproductionFramePending;
        if(infoView.reproductionTimingValid) {
            infoView.reproductionConversionApplicable =
                m_session.reproduction.algorithm
                    != spraythickness::ReproductionAlgorithmKind::CurrentMethod;
            infoView.reproductionVisibilityCountAvailable =
                infoView.reproductionConversionApplicable;
            infoView.reproductionHiddenCountAvailable =
                infoView.reproductionConversionApplicable
                && m_session.reproduction.algorithm
                    != spraythickness::ReproductionAlgorithmKind::Wu2020
                && m_session.reproduction.algorithm
                    != spraythickness::ReproductionAlgorithmKind::DynamicSurface2026;
            infoView.reproductionPreparationMilliseconds =
                m_reproductionPreparationMilliseconds;
            infoView.reproductionCoreMilliseconds =
                m_reproductionCoreMilliseconds;
            infoView.reproductionConversionMilliseconds =
                m_reproductionConversionMilliseconds;
            infoView.reproductionDisplayMilliseconds =
                m_reproductionDisplayMilliseconds;
            infoView.reproductionPresentationMilliseconds =
                m_reproductionPresentationMilliseconds;
            infoView.reproductionTotalMilliseconds =
                m_reproductionTotalMilliseconds;
            infoView.reproductionInputVertexCount =
                m_reproductionInputVertexCount;
            infoView.reproductionInputTriangleCount =
                m_reproductionInputTriangleCount;
            infoView.reproductionOutputVertexCount =
                m_reproductionOutputVertexCount;
            infoView.reproductionOutputTriangleCount =
                m_reproductionOutputTriangleCount;
            infoView.reproductionStatistics =
                spraythickness::reproductionStatistics(m_session.reproduction);
        }
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
        if(onlineHasField) {
            infoView.thicknessMetrics = m_onlineResult->metrics;
            infoView.predictionTiming = m_onlineResult->timing;
            infoView.predictionElapsedSeconds = m_onlineDisplayedTimeSeconds;
            infoView.manualThicknessRange = false;
            infoView.minimumDisplayThicknessMicrometers =
                viewModel.minimumMicrometers;
            infoView.maximumDisplayThicknessMicrometers =
                viewModel.maximumMicrometers;
            infoView.uniformityStatistics = m_onlineUniformity;
        }
        m_infoPanel.applyInfo(infoView);

        CoatingAnalysisVisibilityView visibilityView;
        visibilityView.hasModel = viewModel.hasModel
            || (generatedReproductionSceneActive()
                && m_reproductionGeneratedPreviewVisible);
        visibilityView.hasTrajectory = viewModel.hasTrajectory
            || (generatedReproductionTrajectoryActive()
                && m_reproductionTrajectoryReady
                && !m_reproductionGeneratedTrajectory.empty());
        visibilityView.hasThickness = onlineHasField || m_session.hasResult;
        visibilityView.showModel = m_session.showModel;
        visibilityView.showTrajectory = m_session.showTrajectory;
        visibilityView.showSprayPoints = m_session.showSprayPoints;
        visibilityView.showThickness = onlineModeActive()
            ? m_onlineShowThickness : m_session.showThickness;
        visibilityView.thicknessPickEnabled = onlineModeActive()
            ? m_onlinePickEnabled : m_session.thicknessPickEnabled;
        m_visibilityBar.applyVisibility(visibilityView);

        emit thicknessLegendChanged(
            m_active && (onlineHasField || viewModel.hasResult),
            viewModel.minimumMicrometers,
            viewModel.maximumMicrometers,
            onlineHasField ? false : m_session.showRelativeError,
            !onlineModeActive() && !anyPredictionRunning()
                && !m_session.showRelativeError);
    }

    void CoatingAnalysisModuleController::publishStateChanged()
    {
        RobotQtViewerCoatingAnalysisPayload payload;
        payload.objectId = activeCoatingObjectId();
        const bool onlineResult = onlineModeActive()
            && (m_onlineResult && !m_onlineResult->empty());
        payload.hasResult = onlineResult || m_session.hasResult;
        payload.showThickness = onlineModeActive()
            ? m_onlineShowThickness : m_session.showThickness;
        if(onlineResult) {
            payload.minimumThicknessMeters = m_onlineResult->metrics.minThickness;
            payload.maximumThicknessMeters = m_onlineResult->metrics.maxThickness;
        } else if(m_session.hasResult) {
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

    void CoatingAnalysisModuleController::handleModelDeleteRequested(
        const QString& objectId)
    {
        if(anyPredictionRunning()) {
            m_status = QStringLiteral("Cannot delete a model while prediction is running.");
            refreshViewModel();
            emit statusMessageRequested(m_status, 3000);
            return;
        }
        const simulation_project::SceneObjectDesc* object =
            findObject(m_context.document(), objectId);
        if(object == nullptr || object->objectType != "workpiece") {
            m_status = QStringLiteral("The selected model is no longer available.");
            refreshViewModel();
            return;
        }

        const QString title = coatingAnalysisTranslate(
            m_languageCode, QStringLiteral("Delete Model"));
        const QString question = coatingAnalysisTranslate(
            m_languageCode,
            QStringLiteral("Delete this model from the current project and 3D scene?"));
        if(QMessageBox::question(
               &m_treePanel,
               title,
               question,
               QMessageBox::Yes | QMessageBox::No,
               QMessageBox::No) != QMessageBox::Yes) {
            return;
        }

        const QString modelName = object->name.empty()
            ? objectId
            : QString::fromStdString(object->name);
        SceneEntityWorkflowController workflow(m_context);
        const SceneEntityDeleteResult result = workflow.deleteEntity(
            SceneEntityKind::Object, objectId);
        if(!result.success) {
            m_status = result.message;
            refreshViewModel();
            emit statusMessageRequested(m_status, 5000);
            return;
        }

        m_modelVisibility.remove(objectId);
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->removeSceneObject(objectId);
            services->rebuildCollisionDetectorsFromDocument(m_context.document());
        }
        if(m_context.selectionModel().state().objectId == objectId) {
            m_context.selectionModel().clear(QStringLiteral("coatingAnalysisDeleteModel"));
        }
        ensureWorkpieceSelection();
        m_status = QStringLiteral("Model deleted from project: %1").arg(modelName);
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 4000);
    }

    void CoatingAnalysisModuleController::handleTrajectoryDeleteRequested()
    {
        if(anyPredictionRunning()) {
            m_status = QStringLiteral("Cannot delete the trajectory while prediction is running.");
            refreshViewModel();
            emit statusMessageRequested(m_status, 3000);
            return;
        }
        if(m_session.trajectory.empty()) {
            return;
        }

        const QString title = coatingAnalysisTranslate(
            m_languageCode, QStringLiteral("Delete Trajectory"));
        const QString question = coatingAnalysisTranslate(
            m_languageCode,
            QStringLiteral("Delete the loaded trajectory and its prediction result?"));
        if(QMessageBox::question(
               &m_treePanel,
               title,
               question,
               QMessageBox::Yes | QMessageBox::No,
               QMessageBox::No) != QMessageBox::Yes) {
            return;
        }

        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->setCoatingTrajectoryPreview({}, false);
            services->clearCoatingPredictionDebugState();
            if(m_session.hasResult) {
                services->clearSurfaceScalarOverlay(m_session.objectId);
            }
        }
        m_session.trajectoryName.clear();
        m_session.trajectoryPath.clear();
        m_session.trajectoryInfo = CoatingAnalysisTrajectoryInfo();
        m_session.trajectory = spraytrajectory::SprayTrajectory();
        m_session.waypoints.clear();
        m_session.clearTrajectorySampling();
        m_session.clearResult();
        resetReferenceResult();
        m_predictionObjectId.clear();
        m_predictionProgress = 0.0;
        m_predictionTimerActive = false;
        m_hasLocalPreview = false;
        m_localPreviewDetails.clear();
        m_treePanel.setWaypoints(nullptr);
        m_hasCurrentThickness = false;
        emit thicknessToolTipRequested(QString(), QPoint(), false);
        m_status = QStringLiteral("Trajectory and dependent prediction result deleted.");
        refreshViewModel();
        publishStateChanged();
        emit statusMessageRequested(m_status, 4000);
    }

    void CoatingAnalysisModuleController::handleThicknessClearRequested()
    {
        if(!m_session.hasResult) {
            return;
        }
        if(RobotQtViewerViewportServices* services = m_context.viewportServices()) {
            services->setSurfaceScalarProbeEnabled(false, QString());
            services->clearSurfaceScalarOverlay(activeCoatingObjectId());
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
