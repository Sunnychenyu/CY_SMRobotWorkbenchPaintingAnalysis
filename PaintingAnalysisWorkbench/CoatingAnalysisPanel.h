#pragma once

#include "CoatingAnalysisViewModel.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QWidget>
#include <QVector>

#include <cstddef>

#include "SimulationExperiment.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTabBar;

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

        spraythickness::ThicknessModelKind thicknessModel() const;
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

    signals:
        void openModelRequested();
        void openTrajectoryRequested();
        void selectModelFileRequested();
        void selectTrajectoryFileRequested();
        void savedTrajectorySourceChanged();
        void predictionRequested();
        void cancelPredictionRequested();
        void setReferenceRequested();
        void clearReferenceRequested();
        void checkReferenceRequested();
        void localInputPreviewRequested();
        void localPreviewParametersChanged();
        void rotationAxisChanged();
        void axisymmetricProfileSampleCountChanged();
        void spatialGridParametersChanged();
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

    private:
        void refreshDepositionCurve();
        void emitLocalDebugVisibilityChanged();
        void setModeTab(int index);

        QComboBox* m_workpieceCombo = nullptr;
        QComboBox* m_algorithmCombo = nullptr;
        DepositionCurveWidget* m_curveWidget = nullptr;
        QPushButton* m_openModelButton = nullptr;
        QPushButton* m_openTrajectoryButton = nullptr;
        QPushButton* m_selectModelButton = nullptr;
        QPushButton* m_selectTrajectoryButton = nullptr;
        QComboBox* m_savedTrajectorySourceCombo = nullptr;
        QComboBox* m_trajectorySamplingCombo = nullptr;
        QDoubleSpinBox* m_timeStepSpinBox = nullptr;
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
        bool m_simulationActive = false;
        QTabBar* m_modeTabBar = nullptr;
        QVector<QWidget*> m_predictionSections;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
