#pragma once

#include "CoatingAnalysisSession.h"
#include "PaintingAnalysisMeshAdapter.h"
#include "SimulationExperiment.h"
#include "OnlineVirtualMotion.h"
#include "OnlinePredictionDiagnostics.h"

#include <QHash>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QStringList>

#include <Eigen/Core>

#include <cstdint>
#include <chrono>
#include <memory>
#include <vector>

class QMenu;
class QTimer;

namespace spraytrajectory
{
    struct SprayPathPoint;
    class SprayTrajectory;
}

namespace robot_qt_viewer
{
    class CoatingAnalysisPanel;
    class CoatingAnalysisTreePanel;
    class CoatingAnalysisInfoPanel;
    class CoatingAnalysisVisibilityBar;
    class RobotQtViewerDocumentContext;
    class ThicknessPredictionJobController;
    class OnlineThicknessPredictionJobController;
    struct OnlinePredictionFrame;
    class AlgorithmReproductionJobController;
    struct RobotQtViewerEvent;

    class CoatingAnalysisModuleController : public QObject
    {
        Q_OBJECT

    public:
        CoatingAnalysisModuleController(
            CoatingAnalysisPanel& panel,
            CoatingAnalysisTreePanel& treePanel,
            CoatingAnalysisInfoPanel& infoPanel,
            CoatingAnalysisVisibilityBar& visibilityBar,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);
        ~CoatingAnalysisModuleController() override;

        void activate();
        void deactivate();
        void setLanguageCode(const QString& languageCode);
        void populateViewportContextMenu(QMenu* menu);
        void handleEvent(const RobotQtViewerEvent& event);
        void handleSurfaceScalarHover(
            const QString& objectId,
            double valueMeters,
            double worldX,
            double worldY,
            double worldZ,
            const QPoint& viewportPosition,
            bool hit);
        void handleRotationSurfacePicked(
            const QString& objectId,
            std::uint32_t triangleIndex,
            double hitX,
            double hitY,
            double hitZ,
            double normalX,
            double normalY,
            double normalZ);

        // Waypoint inspection source for the virtualized tree and details dialog.
        const std::vector<spraytrajectory::SprayPathPoint>& waypoints() const;
        const spraytrajectory::SprayPathPoint& waypointAt(std::size_t index) const;
        double waypointDurationAt(std::size_t index) const;

    public slots:
        void handleViewportFrameSwapped();
        void handleOnlineFramePresented(quint64 frameId, bool displayed);
        void handleOnlineViewportFrameRendered(double milliseconds);
        void handleOnlineFrameRenderProfile(quint64 frameId,
            double sceneUpdateMilliseconds, double drawMilliseconds);
        void setThicknessDisplayRange(
            double minimumMicrometers,
            double maximumMicrometers);
        void resetThicknessDisplayRange();

    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
        void thicknessToolTipRequested(
            const QString& text,
            const QPoint& viewportPosition,
            bool visible);
        void thicknessLegendChanged(
            bool visible,
            double minimumMicrometers,
            double maximumMicrometers,
            bool relativeError,
            bool editingEnabled);

    private:
        struct AxisymmetricProfileState;
        struct PendingOnlineDisplay;
        bool loadModel(const QString& path, double scaleToMeters);
        void openModelFromDialog();
        bool loadTrajectory(const QString& path);
        void openTrajectoryFromDialog();
        void savedTrajectorySourceChanged();
        bool loadSelectedSavedTrajectory(bool reportFailure);
        bool loadSavedTrajectoryPlan(bool reportFailure);
        bool loadOptimizationBaselineTrajectory(
            std::size_t trajectoryCount,
            bool reportFailure);
        bool loadOptimizedTrajectory(
            std::size_t trajectoryCount,
            bool reportFailure);
        void loadSavedTrajectoryPlanIfAvailable(bool refreshSavedTrajectory = false);
        bool usesSavedTrajectoryPlan() const;
        bool usesOptimizationBaselineTrajectory() const;
        bool usesOptimizedTrajectory() const;
        void clearLoadedTrajectory();
        void applyLoadedTrajectory(
            spraytrajectory::SprayTrajectory trajectory,
            const QString& sourcePath,
            std::size_t warningCount = 0);
        void selectModelFileFromDialog();
        void selectTrajectoryFileFromDialog();
        void handleTrajectorySamplingParametersChanged();
        void applyTrajectorySampling();
        void predictThickness();
        void startOnlineSpray();
        void stopOnlineSpray();
        void finishOnlineSpray(bool includeStopSample);
        void resetOnlinePrediction();
        void handleLiveRobotPose(const RobotQtViewerEvent& event);
        void advanceVirtualOnlineSpray();
        void updateOnlinePoseDisplay();
        void flushOnlineTrajectory();
        void updateOnlineRefreshStatistics();
        void updateOnlineScreenRefreshRate();
        void refreshOnlineReadouts(double timeSeconds);
        void handleOnlineField(
            const std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>& result,
            const ThicknessUniformityStatistics& uniformity,
            const OnlinePredictionFrame& frame);
        void applyOnlineField(
            const std::shared_ptr<const spraythickness::OnlineThicknessSnapshot>& result,
            const ThicknessUniformityStatistics& uniformity,
            const OnlinePredictionFrame& frame);
        void applyPendingOnlineField();
        void setCurrentResultAsReference();
        void clearReferenceResult();
        void checkCurrentResultAgainstReference();
        void previewLocalInputs();
        void selectAxisymmetricProfileRegion();
        void cancelPrediction();
        void handlePredictionProgress(double progress, const QString& message);
        void handlePredictionFinished(const spraythickness::ThicknessPredictionResult& prediction);
        void handlePredictionFailed(const QString& message);
        void setShowModel(bool enabled);
        void setShowTrajectory(bool enabled);
        void setShowSprayPoints(bool enabled);
        void setShowThickness(bool enabled);
        void setThicknessPickEnabled(bool enabled);
        void setLocalDebugVisibility(
            bool cylindricalSurface,
            bool rotationAxis,
            bool localSector,
            bool sprayPoints);
        void selectWorkpiece(const QString& objectId);
        void selectWorkpieceFromTree(const QString& objectId);
        void ensureWorkpieceSelection();
        void updateSelectedModelInfo();
        void clearModelVisibilityOverrides();
        void applyModelVisibilityOverrides();
        void clearSession();
        void applyVisualizationState();
        void submitTrajectoryPreview();
        void updateTrajectoryPreviewVisibility();
        void applyOverlayAfterReload();
        bool applyCurrentSurfaceOverlay(QString* errorMessage);
        void updateThicknessRangeAndStatistics();
        void refreshViewModel();
        void publishStateChanged();
        bool ensurePreviewWorkpieceLoaded();
        bool hasEffectiveRotationAxis() const;
        Eigen::Vector3d effectiveRotationAxisOrigin() const;
        Eigen::Vector3d effectiveRotationAxisDirection() const;
        QString rotationAxisSource() const;
        void applyLocalDebugVisibility();
        bool ensureAxisymmetricProfileSlice();
        void clearAxisymmetricProfileSelection();
        bool rebuildAxisymmetricProfileReduction();
        void updateAxisymmetricProfileDebugState();
        void resetReferenceResult();
        void enterSimulation();
        void exitSimulation();
        void rebuildSimulation(bool runAfterBuild = false, bool focusView = false);
        void restoreSimulationPreview();
        void runSimulationPrediction();
        void exportSimulationResult();
        void enterReproduction();
        void exitReproduction();
        void enterOnline();
        void exitOnline();
        bool restoreOnlineDisplay();
        void generateReproductionScene(bool focusView = false);
        void generateReproductionTrajectory();
        bool rebuildReproductionScene(QString* errorMessage = nullptr);
        bool rebuildReproductionTrajectory(QString* errorMessage = nullptr);
        bool displayPreparedReproductionScene(
            bool focusView, QString* errorMessage = nullptr);
        void displayPreparedReproductionTrajectory();
        void restoreReproductionPreviews(bool focusView = false);
        void updateReproductionPreparationStatus();
        void invalidateReproductionResult();
        void clearReproductionGeneratedPreview();
        void submitReproductionTrajectoryPreview(
            const spraytrajectory::SprayTrajectory& trajectory);
        QString activeCoatingObjectId() const;
        bool generatedReproductionSceneActive() const;
        bool generatedReproductionTrajectoryActive() const;
        void runAlgorithmReproduction();
        void startReproductionBenchmark();
        void advanceReproductionBenchmark();
        void recordReproductionBenchmarkRun(
            const QString& status, const QString& error = {});
        void createReproductionTemplate();
        void cancelAlgorithmReproduction();
        void exportAlgorithmReproduction();
        void handleReproductionProgress(double progress, const QString& message);
        void handleReproductionDiagnostic(
            const QString& stage, const QString& details);
        void appendReproductionDiagnostic(
            const QString& stage, const QString& details);
        void handleReproductionFinished(
            const spraythickness::AlgorithmReproductionResult& result);
        void handleReproductionFailed(const QString& message);
        void applyWuDisplayScale();
        bool simulationActive() const;
        bool reproductionActive() const;
        bool onlineModeActive() const;
        bool anyPredictionRunning() const;

        void handleWaypointInfoRequested(int index);
        void handleModelVisibilityToggleRequested(const QString& objectId);
        void handleModelSetAsWorkpiece(const QString& objectId);
        void handleModelDeleteRequested(const QString& objectId);
        void handleTrajectoryDeleteRequested();
        void handleThicknessClearRequested();

        CoatingAnalysisPanel& m_panel;
        CoatingAnalysisTreePanel& m_treePanel;
        CoatingAnalysisInfoPanel& m_infoPanel;
        CoatingAnalysisVisibilityBar& m_visibilityBar;
        RobotQtViewerDocumentContext& m_context;
        CoatingAnalysisSession m_session;
        std::unique_ptr<ThicknessPredictionJobController> m_predictionJob;
        std::unique_ptr<OnlineThicknessPredictionJobController> m_onlineJob;
        std::unique_ptr<OnlinePredictionDiagnostics> m_onlineDiagnostics;
        std::unique_ptr<AlgorithmReproductionJobController> m_reproductionJob;
        QString m_status = QStringLiteral("Load a model and trajectory to begin.");
        QString m_predictionObjectId;
        double m_predictionProgress = 0.0;
        bool m_active = false;
        bool m_hasCurrentThickness = false;
        double m_currentThicknessMeters = 0.0;
        bool m_hasRotationAxis = false;
        Eigen::Vector3d m_rotationAxisOrigin = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_rotationAxisDirection = Eigen::Vector3d::UnitZ();
        sprayworkpiece::WorkpieceModel m_rotationPreviewWorkpiece;
        std::vector<std::size_t> m_rotationSurfaceTriangleIndices;
        std::size_t m_rotationSeedTriangleIndex = 0;
        double m_rotationFitElapsedMilliseconds = 0.0;
        bool m_hasLocalPreview = false;
        QString m_localPreviewDetails;
        std::unique_ptr<AxisymmetricProfileState> m_axisymmetricProfile;
        bool m_showCylindricalSurface = true;
        bool m_showRotationAxis = true;
        bool m_showLocalSector = true;
        bool m_showLocalSprayPoints = true;
        std::chrono::steady_clock::time_point m_predictionStartedAt{};
        bool m_predictionTimerActive = false;
        spraythickness::ThicknessField m_referenceThickness;
        std::size_t m_referenceActiveVertexCount = 0;
        QString m_validationDetails = QStringLiteral("No reference result.");
        std::vector<QString> m_modelVisibilityOverrideIds;
        QHash<QString, bool> m_modelVisibility;
        CoatingAnalysisMode m_mode{ CoatingAnalysisMode::Prediction };
        bool m_reproductionGpuRun = false;
        bool m_reproductionRunActive = false;
        bool m_reproductionTimingValid = false;
        double m_reproductionPreparationMilliseconds = 0.0;
        double m_reproductionCoreMilliseconds = 0.0;
        double m_reproductionConversionMilliseconds = 0.0;
        double m_reproductionDisplayMilliseconds = 0.0;
        double m_reproductionPresentationMilliseconds = 0.0;
        double m_reproductionTotalMilliseconds = 0.0;
        std::chrono::steady_clock::time_point m_reproductionStartedAt{};
        std::chrono::steady_clock::time_point m_reproductionOverlayReadyAt{};
        bool m_reproductionFramePending = false;
        std::size_t m_reproductionInputVertexCount = 0;
        std::size_t m_reproductionInputTriangleCount = 0;
        std::size_t m_reproductionOutputVertexCount = 0;
        std::size_t m_reproductionOutputTriangleCount = 0;
        QString m_reproductionStatus = QStringLiteral("No reproduction result yet.");
        QStringList m_reproductionDiagnostics;
        QString m_reproductionBenchmarkPath;
        int m_reproductionBenchmarkAlgorithmIndex = 0;
        int m_reproductionBenchmarkAttempt = 0;
        bool m_reproductionBenchmarkActive = false;
        bool m_reproductionBenchmarkRunPending = false;
        int m_reproductionGpuProgressBucket = -1;
        sprayworkpiece::WorkpieceModel m_reproductionWorkpiece;
        PlateStackData m_reproductionPlateStack;
        spraytrajectory::SprayTrajectory m_reproductionGeneratedTrajectory;
        bool m_reproductionSceneReady = false;
        bool m_reproductionTrajectoryReady = false;
        bool m_reproductionSetupInitialized = false;
        bool m_reproductionGeneratedPreviewVisible = false;
        bool m_reproductionResultUsesGeneratedScene = false;
        QString m_reproductionSceneDetails = QStringLiteral("Scene: not generated.");
        QString m_reproductionTrajectoryDetails =
            QStringLiteral("Trajectory: not generated.");
        SimulationExperimentData m_simulation;
        bool m_simulationReady = false;
        QString m_simulationReadyStatus;
        QString m_simulationStatus;
        QString m_languageCode{ QStringLiteral("en") };
        CoatingAnalysisSession m_predictionSession;
        CoatingAnalysisSession m_simulationSession;
        CoatingAnalysisSession m_reproductionSession;
        CoatingAnalysisSession m_onlineSession;
        QHash<QString, bool> m_predictionModelVisibility;
        QHash<QString, bool> m_simulationModelVisibility;
        QHash<QString, bool> m_reproductionModelVisibility;
        QHash<QString, bool> m_onlineModelVisibility;
        QString m_predictionStatus;
        QString m_onlineStatus{ QStringLiteral("Ready to start online prediction.") };
        struct OnlineObject
        {
            QString id;
            Eigen::Isometry3d worldFromObject = Eigen::Isometry3d::Identity();
            PaintingAnalysisMeshBinding binding;
        };
        std::vector<OnlineObject> m_onlineObjects;
        std::vector<spraytrajectory::SprayPathPoint> m_onlinePendingPoints;
        std::shared_ptr<const spraythickness::OnlineThicknessSnapshot> m_onlineResult;
        std::shared_ptr<const std::vector<smrobot::visualization::SurfaceScalarOverlay>> m_onlineDisplayOverlays;
        std::unique_ptr<PendingOnlineDisplay> m_onlinePendingDisplay;
        ThicknessUniformityStatistics m_onlineUniformity;
        std::chrono::steady_clock::time_point m_onlineLastDisplayLog{};
        std::chrono::steady_clock::time_point m_onlineRefreshStartedAt{};
        std::chrono::steady_clock::time_point m_onlineSprayRequestedAt{};
        std::chrono::steady_clock::time_point m_onlinePresentationRequestedAt{};
        std::chrono::steady_clock::time_point m_onlineLastPresentationLog{};
        std::chrono::steady_clock::time_point m_onlineLastLongFrameLog{};
        std::chrono::steady_clock::time_point m_onlineFieldReceivedAt{};
        std::chrono::steady_clock::time_point m_onlineNextDisplayAt{};
        std::chrono::steady_clock::time_point m_onlineLastPresentedAt{};
        std::vector<double> m_onlineFrameIntervals;
        double m_onlineMaximumFrameIntervalMilliseconds = -1.0;
        double m_onlineScreenRefreshRate = 60.0;
        double m_onlineDisplayedTimeSeconds = 0.0;
        OnlineRefreshCadence m_onlineRefreshCadence;
        OnlineSprayIntegrationSampling m_onlineIntegrationSampling;
        double m_onlineLastRenderMilliseconds = 0.0;
        double m_onlineDeliveryWaitMilliseconds = 0.0;
        double m_onlinePacingWaitMilliseconds = 0.0;
        double m_onlineApplyMilliseconds = 0.0;
        double m_onlineStatisticsMilliseconds = 0.0;
        double m_onlinePoseMilliseconds = 0.0;
        double m_onlineMappingMilliseconds = 0.0;
        double m_onlineWorkerMappingMilliseconds = 0.0;
        double m_onlineOverlayMilliseconds = 0.0;
        std::size_t m_onlineSceneFrameCount = 0;
        std::size_t m_onlineViewportFrameCount = 0;
        std::size_t m_onlineThicknessFrameCount = 0;
        double m_onlineFirstFrameMilliseconds = -1.0;
        bool m_onlineThicknessFramePending = false;
        bool m_onlineWaitingForFirstFrame = false;
        std::uint64_t m_onlinePendingFrameId = 0;
        std::uint64_t m_onlineLastSubmittedFrameId = 0;
        std::uint64_t m_onlineLastAcknowledgedFrameId = 0;
        OnlineDiagnosticFrame m_onlineDiagnosticFrame;
        std::chrono::steady_clock::time_point m_onlineDiagnosticStartedAt{};
        std::chrono::steady_clock::time_point m_onlineLastInputSubmittedAt{};
        std::chrono::steady_clock::time_point m_onlineDiagnosticSubmittedAt{};
        std::chrono::steady_clock::time_point m_onlineDiagnosticRenderedAt{};
        double m_onlineGuiStartupMilliseconds = 0.0;
        bool m_onlineDiagnosticFrameEligible = false;
        Eigen::Isometry3d m_liveGunPose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_liveTablePose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_onlineInitialTablePose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_onlineCurrentTablePose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_onlineCurrentGunPose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_onlineDisplayedTablePose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d m_onlineDisplayedGunPose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d m_onlineVirtualCenter = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_onlineVirtualAxis = Eigen::Vector3d::UnitZ();
        OnlineRandomWorkpieceRotation m_onlineRandomRotation;
        Eigen::Vector3d m_onlineDisplayedRotationAxis = Eigen::Vector3d::UnitZ();
        bool m_onlineVirtualRandomAxis = false;
        Eigen::Vector3d m_onlineVirtualGunStart = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_onlineVirtualGunEnd = Eigen::Vector3d::Zero();
        double m_onlineVirtualRpm = 0.0;
        double m_onlineVirtualGunSpeed = 0.0;
        double m_onlineVirtualTimeSeconds = 0.0;
        double m_onlineVirtualRunBaseSeconds = 0.0;
        std::chrono::steady_clock::time_point m_onlineVirtualRunStartedAt{};
        bool m_onlineVirtualSource = false;
        bool m_onlineVirtualRotating = true;
        QString m_liveGunRobotId;
        QString m_liveTableRobotId;
        QString m_onlineGunRobotId;
        QString m_onlineTableRobotId;
        double m_liveSampleTimeSeconds = 0.0;
        double m_onlineStartTimeSeconds = 0.0;
        double m_onlineLastPoseTimeSeconds = 0.0;
        bool m_onlineActive = false;
        bool m_onlineSpraying = false;
        bool m_onlineShowThickness = true;
        bool m_onlinePickEnabled = false;
        QTimer* m_onlinePoseWatchdog = nullptr;
        QTimer* m_onlineVirtualTimer = nullptr;
        QTimer* m_onlineRefreshTimer = nullptr;
        QTimer* m_onlineDiagnosticTimer = nullptr;
    };
}
