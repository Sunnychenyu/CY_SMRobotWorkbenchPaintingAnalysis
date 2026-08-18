#pragma once

#include "CoatingAnalysisViewModel.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QWidget>

#include <cstddef>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QSpinBox;

namespace robot_qt_viewer
{
    class DepositionCurveWidget;

    enum class PredictionInputMode
    {
        CompleteAllSprayPoints = 0,
        LocalAllSprayPoints = 1,
        CompleteSpatialFilteredSprayPoints = 2,
        LocalSpatialFilteredSprayPoints = 3,
        AxisymmetricProfileSpatialFilteredSprayPoints = 4
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
        bool overrideSpatialGridCellSize() const;
        double spatialGridCellSizeMillimeters() const;
        void setPeriodicLocalPredictionEnabled(bool enabled);
        Eigen::Vector3d periodicAxisDirection() const;
        std::size_t periodicSectorCount() const;
        std::size_t axisymmetricProfileSampleCount() const;
        QString selectedWorkpieceId() const;

    signals:
        void openModelRequested();
        void openTrajectoryRequested();
        void predictionRequested();
        void cancelPredictionRequested();
        void localInputPreviewRequested();
        void localPreviewParametersChanged();
        void axisymmetricProfileSampleCountChanged();
        void spatialGridParametersChanged();
        void profileRegionSelectionRequested();
        void localDebugVisibilityChanged(
            bool cylindricalSurface,
            bool rotationAxis,
            bool localSector,
            bool sprayPoints);
        void workpieceChanged(const QString& objectId);
        void rotationSurfacePickRequested();

    private:
        void refreshDepositionCurve();
        void emitLocalDebugVisibilityChanged();

        QComboBox* m_workpieceCombo = nullptr;
        QComboBox* m_algorithmCombo = nullptr;
        DepositionCurveWidget* m_curveWidget = nullptr;
        QPushButton* m_openModelButton = nullptr;
        QPushButton* m_openTrajectoryButton = nullptr;
        QComboBox* m_trajectorySamplingCombo = nullptr;
        QDoubleSpinBox* m_timeStepSpinBox = nullptr;
        QCheckBox* m_bvhCheckBox = nullptr;
        QCheckBox* m_historyCheckBox = nullptr;
        QComboBox* m_predictionModeCombo = nullptr;
        QWidget* m_spatialGridOptionsWidget = nullptr;
        QCheckBox* m_overrideSpatialGridCellSizeCheckBox = nullptr;
        QDoubleSpinBox* m_spatialGridCellSizeSpinBox = nullptr;
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
    };
}
