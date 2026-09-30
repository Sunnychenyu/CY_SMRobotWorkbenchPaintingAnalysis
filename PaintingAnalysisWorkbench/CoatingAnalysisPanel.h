#pragma once

#include "CoatingAnalysisViewModel.h"

#include <SprayThicknessPrediction/AlgorithmReproduction.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QHash>
#include <QWidget>
#include <QVector>

#include <cstddef>

#include "SimulationExperiment.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QTextBrowser;
class QSpinBox;
class QTabBar;
class QTabWidget;

namespace robot_qt_viewer
{
    class DepositionCurveWidget;

    enum class PredictionInputMode
    {
        CompleteAllSprayPoints = 0,
        LocalAllSprayPoints = 1,
        CompleteSpatialFilteredSprayPoints = 2,
        LocalSpatialFilteredSprayPoints = 3,
        AxisymmetricProfileSpatialFilteredSprayPoints = 4,
        CompleteSpatialFilteredCandidateVertices = 5,
        AdaptiveMeshSpatialFilteredCandidateVertices = 6,
        LocalSpatialFilteredCandidateVerticesFullBvh = 7
    };

    enum class ReproductionSceneSource
    {
        ImportedModel = 0,
        GeneratedPlateStack = 1
    };

    enum class ReproductionTrajectorySource
    {
        ImportedTrajectory = 0,
        GeneratedPointSpray = 1,
        GeneratedLineScan = 2
    };

    enum class SavedTrajectorySource
    {
        Planning = 0,
        DualOptimizationBaseline = 1,
        DualOptimization = 2,
        ThreeOptimizationBaseline = 3,
        ThreeOptimization = 4
    };

    // Right-side computation controls of the coating analysis workbench. Pure
    // input surface: selection and toggles forward to the module controller.
    // Read-out information lives in CoatingAnalysisInfoPanel, element visibility
    // lives in CoatingAnalysisVisibilityBar.
    class CoatingAnalysisPanel : public QWidget
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisPanel(QWidget* parent = nullptr);
        void applyViewModel(const CoatingAnalysisViewModel& viewModel);
        void setLanguageCode(const QString& languageCode);
        void setOnlinePredictionState(bool active, bool spraying, const QString& status);

        spraythickness::ThicknessModelKind thicknessModel() const;
        Eigen::Vector3d sprayDirectionLocal() const;
        Eigen::Vector3d powderFeedDirectionLocal() const;
        Eigen::Vector3d onlineSprayDirectionLocal() const;
        Eigen::Vector3d onlinePowderFeedDirectionLocal() const;
        bool onlineBvhOcclusionEnabled() const;
        bool onlineHistoryCorrectionEnabled() const;
        spraythickness::TrajectorySamplingMode trajectorySamplingMode() const;
        double timeStepSeconds() const;
        bool bvhOcclusionEnabled() const;
        bool historyCorrectionEnabled() const;
        PredictionInputMode predictionInputMode() const;
        bool periodicLocalPredictionEnabled() const;
        bool axisymmetricProfilePredictionEnabled() const;
        bool rotationBasedPredictionEnabled() const;
        bool spatialInfluenceFilteringEnabled() const;
        bool spatialCandidateVertexFilteringEnabled() const;
        bool adaptiveMeshPredictionEnabled() const;
        bool localCandidateVertexPredictionEnabled() const;
        double adaptiveMeshSimplificationPercent() const;
        bool overrideSpatialGridCellSize() const;
        double spatialGridCellSizeMillimeters() const;
        void setPeriodicLocalPredictionEnabled(bool enabled);
        Eigen::Vector3d periodicAxisDirection() const;
        std::size_t periodicSectorCount() const;
        std::size_t axisymmetricProfileSampleCount() const;
        QString selectedWorkpieceId() const;
        SavedTrajectorySource savedTrajectorySource() const;
        SimulationExperimentParameters simulationParameters() const;
        bool simulationActive() const;
        CoatingAnalysisMode mode() const;
        spraythickness::ReproductionAlgorithmKind reproductionAlgorithm() const;
        ReproductionSceneSource reproductionSceneSource() const;
        ReproductionTrajectorySource reproductionTrajectorySource() const;
        PlateStackParameters reproductionPlateStackParameters() const;
        SimulationExperimentParameters reproductionGeneratedTrajectoryParameters() const;
        Eigen::Vector3d reproductionSprayDirectionLocal() const;
        Eigen::Vector3d reproductionPowderFeedDirectionLocal() const;
        bool reproductionHistoryCorrectionEnabled() const;
        double wuNormalDisplayScale() const;
        QString reproductionConfigurationPath() const;
        void setReproductionConfigurationPath(const QString& path);
        bool applyReproductionBenchmarkSetup(
            spraythickness::ReproductionAlgorithmKind algorithm);

    signals:
        void openModelRequested();
        void openTrajectoryRequested();
        void selectModelFileRequested();
        void selectTrajectoryFileRequested();
        void savedTrajectorySourceChanged();
        void predictionRequested();
        void onlineSprayStartRequested();
        void onlineSprayStopRequested();
        void cancelPredictionRequested();
        void setReferenceRequested();
        void clearReferenceRequested();
        void checkReferenceRequested();
        void localInputPreviewRequested();
        void localPreviewParametersChanged();
        void rotationAxisChanged();
        void axisymmetricProfileSampleCountChanged();
        void spatialGridParametersChanged();
        void depositionDirectionsChanged();
        void trajectorySamplingParametersChanged();
        void trajectorySamplingApplyRequested();
        void profileRegionSelectionRequested();
        void adaptiveRegionSelectionRequested();
        void localDebugVisibilityChanged(
            bool cylindricalSurface,
            bool rotationAxis,
            bool localSector,
            bool sprayPoints);
        void workpieceChanged(const QString& objectId);
        void rotationSurfacePickRequested();
        void enterSimulationRequested();
        void exitSimulationRequested();
        void simulationParametersChanged();
        void simulationPredictionRequested();
        void simulationExportRequested();
        void enterReproductionRequested();
        void exitReproductionRequested();
        void enterOnlineRequested();
        void exitOnlineRequested();
        void reproductionRequested();
        void reproductionBenchmarkRequested();
        void reproductionInputsChanged();
        void reproductionSceneParametersChanged();
        void reproductionTrajectoryParametersChanged();
        void reproductionSceneApplyRequested();
        void reproductionTrajectoryApplyRequested();
        void reproductionRecommendedSetupRequested();
        void reproductionTemplateRequested();
        void cancelReproductionRequested();
        void reproductionExportRequested();
        void wuDisplayScaleApplyRequested();

    private:
        void refreshDepositionCurve();
        void ensurePowderFeedDirectionValid();
        void updateTrajectorySamplingUi();
        void emitLocalDebugVisibilityChanged();
        void setModeTab(int index);
        void updateReproductionInputUi();
        void updateReproductionSourceUi();
        void updateReproductionDescription();
        void updateReproductionRecommendation();
        void applyReproductionRecommendedSetup();
        bool reproductionForcesOriginalTrajectoryPoints() const;

        QComboBox* m_workpieceCombo = nullptr;
        QComboBox* m_algorithmCombo = nullptr;
        QComboBox* m_onlineAlgorithmCombo = nullptr;
        QComboBox* m_onlineSprayDirectionCombo = nullptr;
        QComboBox* m_onlinePowderFeedDirectionCombo = nullptr;
        QCheckBox* m_onlineBvhCheckBox = nullptr;
        QCheckBox* m_onlineHistoryCheckBox = nullptr;
        QPushButton* m_onlineStartButton = nullptr;
        QPushButton* m_onlineStopButton = nullptr;
        QLabel* m_onlineStatusLabel = nullptr;
        bool m_onlineActive = false;
        bool m_onlineSpraying = false;
        QComboBox* m_sprayDirectionCombo = nullptr;
        QComboBox* m_powderFeedDirectionCombo = nullptr;
        DepositionCurveWidget* m_curveWidget = nullptr;
        QPushButton* m_openModelButton = nullptr;
        QPushButton* m_openTrajectoryButton = nullptr;
        QPushButton* m_selectModelButton = nullptr;
        QPushButton* m_selectTrajectoryButton = nullptr;
        QComboBox* m_savedTrajectorySourceCombo = nullptr;
        QComboBox* m_trajectorySamplingCombo = nullptr;
        QWidget* m_timeStepLabel = nullptr;
        QDoubleSpinBox* m_timeStepSpinBox = nullptr;
        QPushButton* m_applyTrajectorySamplingButton = nullptr;
        QCheckBox* m_bvhCheckBox = nullptr;
        QCheckBox* m_historyCheckBox = nullptr;
        QComboBox* m_predictionModeCombo = nullptr;
        QWidget* m_spatialGridOptionsWidget = nullptr;
        QCheckBox* m_overrideSpatialGridCellSizeCheckBox = nullptr;
        QDoubleSpinBox* m_spatialGridCellSizeSpinBox = nullptr;
        QWidget* m_adaptiveMeshOptionsWidget = nullptr;
        QPushButton* m_selectAdaptiveRegionButton = nullptr;
        QDoubleSpinBox* m_adaptiveMeshSimplificationSpinBox = nullptr;
        QGroupBox* m_localConfigGroup = nullptr;
        QLabel* m_rotationAxisStatusLabel = nullptr;
        QLabel* m_periodicSectorLabel = nullptr;
        QComboBox* m_periodicAxisCombo = nullptr;
        QSpinBox* m_periodicSectorCountSpinBox = nullptr;
        QLabel* m_axisymmetricProfileSampleCountLabel = nullptr;
        QSpinBox* m_axisymmetricProfileSampleCountSpinBox = nullptr;
        QPushButton* m_pickRotationSurfaceButton = nullptr;
        QPushButton* m_selectProfileRegionButton = nullptr;
        QPushButton* m_previewLocalInputsButton = nullptr;
        QCheckBox* m_showCylindricalSurfaceCheckBox = nullptr;
        QCheckBox* m_showRotationAxisCheckBox = nullptr;
        QCheckBox* m_showLocalSectorCheckBox = nullptr;
        QCheckBox* m_showLocalSprayPointsCheckBox = nullptr;
        QPushButton* m_predictionButton = nullptr;
        QPushButton* m_cancelButton = nullptr;
        QProgressBar* m_progressBar = nullptr;
        QLabel* m_statusLabel = nullptr;
        QGroupBox* m_validationGroup = nullptr;
        QPushButton* m_setReferenceButton = nullptr;
        QPushButton* m_clearReferenceButton = nullptr;
        QPushButton* m_checkReferenceButton = nullptr;
        QLabel* m_referenceStatusLabel = nullptr;
        QGroupBox* m_simulationGroup = nullptr;
        QComboBox* m_simulationKindCombo = nullptr;
        QDoubleSpinBox* m_plateSideSpinBox = nullptr;
        QDoubleSpinBox* m_plateCellSpinBox = nullptr;
        QDoubleSpinBox* m_simulationDistanceSpinBox = nullptr;
        QDoubleSpinBox* m_trajectoryOverrunSpinBox = nullptr;
        QDoubleSpinBox* m_simulationIncidenceSpinBox = nullptr;
        QDoubleSpinBox* m_simulationAzimuthSpinBox = nullptr;
        QDoubleSpinBox* m_simulationToolRollSpinBox = nullptr;
        QDoubleSpinBox* m_pointDurationSpinBox = nullptr;
        QDoubleSpinBox* m_scanSpeedSpinBox = nullptr;
        QSpinBox* m_scanPassCountSpinBox = nullptr;
        QDoubleSpinBox* m_entrySpeedSpinBox = nullptr;
        QDoubleSpinBox* m_exitSpeedSpinBox = nullptr;
        QDoubleSpinBox* m_trajectoryPointIntervalSpinBox = nullptr;
        QDoubleSpinBox* m_scanStartXSpinBox = nullptr;
        QDoubleSpinBox* m_scanStartYSpinBox = nullptr;
        QDoubleSpinBox* m_scanEndXSpinBox = nullptr;
        QDoubleSpinBox* m_scanEndYSpinBox = nullptr;
        QPushButton* m_runSimulationButton = nullptr;
        QPushButton* m_exportSimulationButton = nullptr;
        QGroupBox* m_reproductionGroup = nullptr;
        QTabWidget* m_reproductionSetupTabs = nullptr;
        QComboBox* m_reproductionSceneSourceCombo = nullptr;
        QPushButton* m_reproductionSelectModelButton = nullptr;
        QWidget* m_reproductionPlateStackWidget = nullptr;
        QDoubleSpinBox* m_reproductionPlateSideSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionPlateCellSpinBox = nullptr;
        QSpinBox* m_reproductionPlateCountSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionPlateSpacingSpinBox = nullptr;
        QPushButton* m_applyReproductionSceneButton = nullptr;
        QComboBox* m_reproductionTrajectorySourceCombo = nullptr;
        QPushButton* m_reproductionSelectTrajectoryButton = nullptr;
        QWidget* m_reproductionGeneratedTrajectoryWidget = nullptr;
        QWidget* m_reproductionPointTrajectoryWidget = nullptr;
        QWidget* m_reproductionLineTrajectoryWidget = nullptr;
        QWidget* m_reproductionTargetSpanLabel = nullptr;
        QDoubleSpinBox* m_reproductionTargetSpanSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionDistanceSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionTrajectoryOverrunSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionIncidenceSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionAzimuthSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionToolRollSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionPointDurationSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionScanSpeedSpinBox = nullptr;
        QSpinBox* m_reproductionScanPassCountSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionEntrySpeedSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionExitSpeedSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionPointIntervalSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionScanStartXSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionScanStartYSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionScanEndXSpinBox = nullptr;
        QDoubleSpinBox* m_reproductionScanEndYSpinBox = nullptr;
        QPushButton* m_applyReproductionTrajectoryButton = nullptr;
        QComboBox* m_reproductionAlgorithmCombo = nullptr;
        QComboBox* m_reproductionSprayDirectionCombo = nullptr;
        QComboBox* m_reproductionPowderFeedDirectionCombo = nullptr;
        QCheckBox* m_reproductionBvhCheckBox = nullptr;
        QCheckBox* m_reproductionHistoryCheckBox = nullptr;
        QWidget* m_wuDisplayScaleWidget = nullptr;
        QWidget* m_wuDisplayScaleLabel = nullptr;
        QDoubleSpinBox* m_wuDisplayScaleSpinBox = nullptr;
        QPushButton* m_applyWuDisplayScaleButton = nullptr;
        QLineEdit* m_reproductionConfigurationEdit = nullptr;
        QPushButton* m_selectReproductionConfigurationButton = nullptr;
        QPushButton* m_createReproductionTemplateButton = nullptr;
        QLabel* m_reproductionModelInputLabel = nullptr;
        QLabel* m_reproductionTrajectoryInputLabel = nullptr;
        QPushButton* m_runReproductionButton = nullptr;
        QPushButton* m_runReproductionBenchmarkButton = nullptr;
        QPushButton* m_applyReproductionRecommendedSetupButton = nullptr;
        QPushButton* m_cancelReproductionButton = nullptr;
        QPushButton* m_exportReproductionButton = nullptr;
        QProgressBar* m_reproductionProgressBar = nullptr;
        QTextBrowser* m_reproductionStatusBrowser = nullptr;
        QString m_reproductionRecommendationText;
        bool m_reproductionPresetAvailable = false;
        QGroupBox* m_reproductionDescriptionGroup = nullptr;
        QTextBrowser* m_reproductionDescriptionBrowser = nullptr;
        QHash<int, QString> m_reproductionConfigurationPaths;
        int m_reproductionConfigurationAlgorithm = -1;
        bool m_reproductionControlsLocked = false;
        CoatingAnalysisMode m_mode{ CoatingAnalysisMode::Prediction };
        QTabBar* m_modeTabBar = nullptr;
        QGroupBox* m_workpieceGroup = nullptr;
        QVector<QWidget*> m_sharedSections;
        QVector<QWidget*> m_predictionSections;
        QVector<QWidget*> m_onlineSections;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
