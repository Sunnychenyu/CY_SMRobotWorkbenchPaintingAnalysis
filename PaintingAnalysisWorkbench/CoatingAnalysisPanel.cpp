#include "CoatingAnalysisPanel.h"

#include "DepositionCurveWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    CoatingAnalysisPanel::CoatingAnalysisPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* rootLayout = new QVBoxLayout(this);
        rootLayout->setContentsMargins(6, 4, 6, 6);
        rootLayout->setSpacing(6);

        // Workpiece selection.
        auto* workpieceGroup = new QGroupBox(QStringLiteral("Workpiece"), this);
        auto* workpieceForm = new QFormLayout(workpieceGroup);
        m_workpieceCombo = new QComboBox(workpieceGroup);
        workpieceForm->setContentsMargins(8, 6, 8, 6);
        workpieceForm->setHorizontalSpacing(6);
        workpieceForm->setVerticalSpacing(4);
        workpieceForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        m_workpieceCombo->setSizeAdjustPolicy(
            QComboBox::AdjustToMinimumContentsLengthWithIcon);
        m_workpieceCombo->setMinimumContentsLength(14);
        m_workpieceCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        workpieceForm->addRow(QStringLiteral("Target"), m_workpieceCombo);
        rootLayout->addWidget(workpieceGroup);
        connect(m_workpieceCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                if(!m_workpieceCombo->currentData().isValid()) {
                    return;
                }
                emit workpieceChanged(m_workpieceCombo->currentData().toString());
            });

        // Model and trajectory loading.
        auto* dataGroup = new QGroupBox(QStringLiteral("Data"), this);
        auto* dataLayout = new QGridLayout(dataGroup);
        dataLayout->setContentsMargins(8, 6, 8, 6);
        dataLayout->setSpacing(6);
        m_openModelButton = new QPushButton(QStringLiteral("Debug Model"), dataGroup);
        m_openTrajectoryButton = new QPushButton(QStringLiteral("Debug Trajectory"), dataGroup);
        m_selectModelButton = new QPushButton(QStringLiteral("Select Model File"), dataGroup);
        m_selectTrajectoryButton = new QPushButton(QStringLiteral("Select Trajectory File"), dataGroup);
        m_openModelButton->setToolTip(
            QStringLiteral("Load the configured debug workpiece model."));
        m_openTrajectoryButton->setToolTip(
            QStringLiteral("Load the configured debug spray trajectory."));
        m_selectModelButton->setToolTip(QStringLiteral("Choose a workpiece model file."));
        m_selectTrajectoryButton->setToolTip(QStringLiteral("Choose a spray trajectory file."));
        m_openModelButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_openTrajectoryButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_selectModelButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_selectTrajectoryButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        dataLayout->addWidget(m_openModelButton, 0, 0);
        dataLayout->addWidget(m_openTrajectoryButton, 0, 1);
        dataLayout->addWidget(m_selectModelButton, 1, 0);
        dataLayout->addWidget(m_selectTrajectoryButton, 1, 1);
        rootLayout->addWidget(dataGroup);
        connect(m_openModelButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::openModelRequested);
        connect(m_openTrajectoryButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::openTrajectoryRequested);
        connect(m_selectModelButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::selectModelFileRequested);
        connect(m_selectTrajectoryButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::selectTrajectoryFileRequested);

        // Deposition algorithm + curve preview.
        auto* algorithmGroup = new QGroupBox(QStringLiteral("Deposition Model"), this);
        auto* algorithmLayout = new QVBoxLayout(algorithmGroup);
        algorithmLayout->setContentsMargins(8, 6, 8, 6);
        algorithmLayout->setSpacing(4);
        m_algorithmCombo = new QComboBox(algorithmGroup);
        m_algorithmCombo->addItem(
            QStringLiteral("Paper Gaussian (GPU)"),
            static_cast<int>(spraythickness::ThicknessModelKind::PaperGaussian));
        m_curveWidget = new DepositionCurveWidget(algorithmGroup);
        m_curveWidget->setFixedHeight(72);
        algorithmLayout->addWidget(m_algorithmCombo);
        algorithmLayout->addWidget(m_curveWidget);
        rootLayout->addWidget(algorithmGroup);
        connect(m_algorithmCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &CoatingAnalysisPanel::refreshDepositionCurve);

        // Trajectory sampling.
        auto* samplingGroup = new QGroupBox(QStringLiteral("Trajectory Sampling"), this);
        auto* samplingForm = new QFormLayout(samplingGroup);
        samplingForm->setContentsMargins(8, 6, 8, 6);
        samplingForm->setHorizontalSpacing(6);
        samplingForm->setVerticalSpacing(4);
        samplingForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        m_trajectorySamplingCombo = new QComboBox(samplingGroup);
        m_trajectorySamplingCombo->addItem(
            QStringLiteral("Original Points"),
            static_cast<int>(spraythickness::TrajectorySamplingMode::OriginalPoints));
        m_trajectorySamplingCombo->addItem(
            QStringLiteral("Resample by Time Step"),
            static_cast<int>(spraythickness::TrajectorySamplingMode::ResampleByTimeStep));
        m_timeStepSpinBox = new QDoubleSpinBox(samplingGroup);
        m_timeStepSpinBox->setRange(0.001, 1.0);
        m_timeStepSpinBox->setDecimals(3);
        m_timeStepSpinBox->setSingleStep(0.01);
        m_timeStepSpinBox->setValue(0.02);
        m_timeStepSpinBox->setSuffix(QStringLiteral(" s"));
        samplingForm->addRow(QStringLiteral("Sampling"), m_trajectorySamplingCombo);
        samplingForm->addRow(QStringLiteral("Time step"), m_timeStepSpinBox);
        rootLayout->addWidget(samplingGroup);
        connect(m_trajectorySamplingCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                m_timeStepSpinBox->setEnabled(
                    trajectorySamplingMode()
                    == spraythickness::TrajectorySamplingMode::ResampleByTimeStep);
            });

        // Computation options.
        auto* optionsGroup = new QGroupBox(QStringLiteral("Options"), this);
        auto* optionsLayout = new QVBoxLayout(optionsGroup);
        optionsLayout->setContentsMargins(8, 6, 8, 6);
        optionsLayout->setSpacing(3);
        m_bvhCheckBox = new QCheckBox(QStringLiteral("BVH shadow occlusion"), optionsGroup);
        m_historyCheckBox = new QCheckBox(QStringLiteral("Thermal exposure history"), optionsGroup);
        m_bvhCheckBox->setChecked(true);
        m_historyCheckBox->setChecked(true);
        optionsLayout->addWidget(m_bvhCheckBox);
        optionsLayout->addWidget(m_historyCheckBox);
        rootLayout->addWidget(optionsGroup);

        auto* modeGroup = new QGroupBox(QStringLiteral("Prediction Input"), this);
        auto* modeLayout = new QVBoxLayout(modeGroup);
        modeLayout->setContentsMargins(8, 6, 8, 6);
        modeLayout->setSpacing(4);
        m_predictionModeCombo = new QComboBox(modeGroup);
        m_predictionModeCombo->setSizeAdjustPolicy(
            QComboBox::AdjustToMinimumContentsLengthWithIcon);
        m_predictionModeCombo->setMinimumContentsLength(20);
        m_predictionModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        const auto addPredictionMode = [this](
                                           const QString& label,
                                           const QString& tooltip,
                                           PredictionInputMode mode) {
            m_predictionModeCombo->addItem(label, static_cast<int>(mode));
            m_predictionModeCombo->setItemData(
                m_predictionModeCombo->count() - 1,
                tooltip,
                Qt::ToolTipRole);
        };
        addPredictionMode(
            QStringLiteral("Complete - all spray points"),
            QStringLiteral("Complete model + all spray points"),
            PredictionInputMode::CompleteAllSprayPoints);
        addPredictionMode(
            QStringLiteral("Local - all spray points"),
            QStringLiteral("Local model + all spray points"),
            PredictionInputMode::LocalAllSprayPoints);
        addPredictionMode(
            QStringLiteral("Complete - spatial filtering"),
            QStringLiteral("Complete model + spatial-filtered spray points"),
            PredictionInputMode::CompleteSpatialFilteredSprayPoints);
        addPredictionMode(
            QStringLiteral("Complete - candidate vertices"),
            QStringLiteral("Complete model + spatial-filtered spray points + candidate vertices"),
            PredictionInputMode::CompleteSpatialFilteredCandidateVertices);
        addPredictionMode(
            QStringLiteral("Local - spatial filtering"),
            QStringLiteral("Local model + spatial-filtered spray points"),
            PredictionInputMode::LocalSpatialFilteredSprayPoints);
        addPredictionMode(
            QStringLiteral("Axisymmetric profile - spatial filtering"),
            QStringLiteral("Axisymmetric profile samples + spatial-filtered spray points"),
            PredictionInputMode::AxisymmetricProfileSpatialFilteredSprayPoints);
        addPredictionMode(
            QStringLiteral("Adaptive mesh - candidate filtering"),
            QStringLiteral("Dense selected region + sparse outside mesh + candidate spray points/vertices"),
            PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices);
        addPredictionMode(
            QStringLiteral("Local candidates - full BVH"),
            QStringLiteral("Selected local vertices + candidate spray points/vertices + complete-model occlusion BVH"),
            PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh);
        m_predictionModeCombo->setToolTip(
            m_predictionModeCombo->itemData(0, Qt::ToolTipRole).toString());
        modeLayout->addWidget(m_predictionModeCombo);
        m_spatialGridOptionsWidget = new QWidget(modeGroup);
        auto* spatialGridForm = new QFormLayout(m_spatialGridOptionsWidget);
        spatialGridForm->setContentsMargins(0, 0, 0, 0);
        m_overrideSpatialGridCellSizeCheckBox = new QCheckBox(
            QStringLiteral("Override grid cell size"),
            m_spatialGridOptionsWidget);
        m_spatialGridCellSizeSpinBox = new QDoubleSpinBox(
            m_spatialGridOptionsWidget);
        m_spatialGridCellSizeSpinBox->setRange(0.01, 1000.0);
        m_spatialGridCellSizeSpinBox->setDecimals(3);
        m_spatialGridCellSizeSpinBox->setSingleStep(1.0);
        m_spatialGridCellSizeSpinBox->setValue(10.0);
        spatialGridForm->addRow(m_overrideSpatialGridCellSizeCheckBox);
        spatialGridForm->addRow(
            QStringLiteral("Grid cell (mm)"),
            m_spatialGridCellSizeSpinBox);
        modeLayout->addWidget(m_spatialGridOptionsWidget);
        m_adaptiveMeshOptionsWidget = new QWidget(modeGroup);
        auto* adaptiveForm = new QFormLayout(m_adaptiveMeshOptionsWidget);
        adaptiveForm->setContentsMargins(0, 0, 0, 0);
        m_selectAdaptiveRegionButton = new QPushButton(
            QStringLiteral("Select prediction region"), m_adaptiveMeshOptionsWidget);
        m_adaptiveMeshSimplificationSpinBox = new QDoubleSpinBox(m_adaptiveMeshOptionsWidget);
        m_adaptiveMeshSimplificationSpinBox->setRange(1.0, 100.0);
        m_adaptiveMeshSimplificationSpinBox->setDecimals(1);
        m_adaptiveMeshSimplificationSpinBox->setSingleStep(1.0);
        m_adaptiveMeshSimplificationSpinBox->setValue(30.0);
        m_adaptiveMeshSimplificationSpinBox->setSuffix(QStringLiteral(" %"));
        adaptiveForm->addRow(m_selectAdaptiveRegionButton);
        adaptiveForm->addRow(QStringLiteral("Outside target vertex ratio"), m_adaptiveMeshSimplificationSpinBox);
        modeLayout->addWidget(m_adaptiveMeshOptionsWidget);
        rootLayout->addWidget(modeGroup);

        m_localConfigGroup = new QGroupBox(QStringLiteral("Local Setup"), this);
        auto* periodicLayout = new QVBoxLayout(m_localConfigGroup);
        periodicLayout->setContentsMargins(8, 6, 8, 6);
        periodicLayout->setSpacing(4);
        m_rotationAxisStatusLabel = new QLabel(
            QStringLiteral("Rotation axis: Not configured"), m_localConfigGroup);
        m_rotationAxisStatusLabel->setWordWrap(true);
        periodicLayout->addWidget(m_rotationAxisStatusLabel);
        auto* periodicForm = new QFormLayout();
        periodicForm->setHorizontalSpacing(6);
        periodicForm->setVerticalSpacing(4);
        periodicForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        m_periodicAxisCombo = new QComboBox(m_localConfigGroup);
        m_periodicAxisCombo->addItem(QStringLiteral("X axis"), 0);
        m_periodicAxisCombo->addItem(QStringLiteral("Y axis"), 1);
        m_periodicAxisCombo->addItem(QStringLiteral("Z axis"), 2);
        m_periodicAxisCombo->setCurrentIndex(2);
        m_periodicSectorCountSpinBox = new QSpinBox(m_localConfigGroup);
        m_periodicSectorCountSpinBox->setRange(2, 360);
        m_periodicSectorCountSpinBox->setValue(4);
        periodicForm->addRow(QStringLiteral("Rotation axis"), m_periodicAxisCombo);
        m_periodicSectorLabel = new QLabel(QStringLiteral("Sectors"), m_localConfigGroup);
        periodicForm->addRow(m_periodicSectorLabel, m_periodicSectorCountSpinBox);
        m_axisymmetricProfileSampleCountLabel = new QLabel(
            QStringLiteral("Profile samples"), m_localConfigGroup);
        m_axisymmetricProfileSampleCountSpinBox = new QSpinBox(m_localConfigGroup);
        m_axisymmetricProfileSampleCountSpinBox->setRange(16, 1000000);
        m_axisymmetricProfileSampleCountSpinBox->setSingleStep(128);
        m_axisymmetricProfileSampleCountSpinBox->setValue(1024);
        m_axisymmetricProfileSampleCountSpinBox->setToolTip(
            QStringLiteral("Total uniform samples across the selected profile region."));
        periodicForm->addRow(
            m_axisymmetricProfileSampleCountLabel,
            m_axisymmetricProfileSampleCountSpinBox);
        periodicLayout->addLayout(periodicForm);
        m_pickRotationSurfaceButton = new QPushButton(
            QStringLiteral("Pick surface and fit axis"), m_localConfigGroup);
        m_pickRotationSurfaceButton->setToolTip(
            QStringLiteral("Pick a cylindrical surface and fit its rotation axis."));
        periodicLayout->addWidget(m_pickRotationSurfaceButton);
        m_selectProfileRegionButton = new QPushButton(
            QStringLiteral("Select profile prediction region"), m_localConfigGroup);
        m_selectProfileRegionButton->setToolTip(
            QStringLiteral("Select the profile region that participates in thickness prediction."));
        periodicLayout->addWidget(m_selectProfileRegionButton);
        m_previewLocalInputsButton = new QPushButton(
            QStringLiteral("Preview local prediction inputs"), m_localConfigGroup);
        periodicLayout->addWidget(m_previewLocalInputsButton);

        auto* debugGroup = new QGroupBox(QStringLiteral("Local Debug Display"), m_localConfigGroup);
        auto* debugLayout = new QVBoxLayout(debugGroup);
        debugLayout->setContentsMargins(8, 6, 8, 6);
        debugLayout->setSpacing(3);
        m_showCylindricalSurfaceCheckBox = new QCheckBox(
            QStringLiteral("Cylinder surface"), debugGroup);
        m_showRotationAxisCheckBox = new QCheckBox(
            QStringLiteral("Rotation axis"), debugGroup);
        m_showLocalSectorCheckBox = new QCheckBox(
            QStringLiteral("Local sector"), debugGroup);
        m_showLocalSprayPointsCheckBox = new QCheckBox(
            QStringLiteral("Calculation spray points"), debugGroup);
        m_showCylindricalSurfaceCheckBox->setChecked(true);
        m_showRotationAxisCheckBox->setChecked(true);
        m_showLocalSectorCheckBox->setChecked(true);
        m_showLocalSprayPointsCheckBox->setChecked(true);
        debugLayout->addWidget(m_showCylindricalSurfaceCheckBox);
        debugLayout->addWidget(m_showRotationAxisCheckBox);
        debugLayout->addWidget(m_showLocalSectorCheckBox);
        debugLayout->addWidget(m_showLocalSprayPointsCheckBox);
        periodicLayout->addWidget(debugGroup);
        rootLayout->addWidget(m_localConfigGroup);

        const auto updateModeControls = [this]() {
            const bool localMode = periodicLocalPredictionEnabled();
            const bool profileMode = axisymmetricProfilePredictionEnabled();
            const bool rotationBasedMode = rotationBasedPredictionEnabled();
            const bool spatialMode = spatialInfluenceFilteringEnabled();
            const bool adaptiveMode = adaptiveMeshPredictionEnabled();
            const bool localCandidateMode = localCandidateVertexPredictionEnabled();
            m_localConfigGroup->setVisible(rotationBasedMode);
            m_periodicAxisCombo->setEnabled(rotationBasedMode);
            m_periodicSectorCountSpinBox->setEnabled(localMode);
            m_periodicSectorLabel->setVisible(localMode);
            m_periodicSectorCountSpinBox->setVisible(localMode);
            m_axisymmetricProfileSampleCountLabel->setVisible(profileMode);
            m_axisymmetricProfileSampleCountSpinBox->setVisible(profileMode);
            m_axisymmetricProfileSampleCountSpinBox->setEnabled(profileMode);
            m_pickRotationSurfaceButton->setEnabled(rotationBasedMode);
            m_selectProfileRegionButton->setVisible(profileMode);
            m_selectProfileRegionButton->setEnabled(profileMode);
            m_previewLocalInputsButton->setVisible(localMode);
            m_previewLocalInputsButton->setEnabled(localMode);
            m_showCylindricalSurfaceCheckBox->setEnabled(rotationBasedMode);
            m_showRotationAxisCheckBox->setEnabled(rotationBasedMode);
            m_showLocalSectorCheckBox->setEnabled(rotationBasedMode);
            m_showLocalSprayPointsCheckBox->setEnabled(rotationBasedMode);
            m_showLocalSectorCheckBox->setText(profileMode
                ? QStringLiteral("Profile line") : QStringLiteral("Local sector"));
            m_spatialGridOptionsWidget->setVisible(spatialMode);
            m_adaptiveMeshOptionsWidget->setVisible(adaptiveMode || localCandidateMode);
            m_selectAdaptiveRegionButton->setEnabled(adaptiveMode || localCandidateMode);
            m_adaptiveMeshSimplificationSpinBox->setVisible(adaptiveMode);
            m_adaptiveMeshSimplificationSpinBox->setEnabled(adaptiveMode);
            m_overrideSpatialGridCellSizeCheckBox->setEnabled(spatialMode);
            m_spatialGridCellSizeSpinBox->setEnabled(
                spatialMode && m_overrideSpatialGridCellSizeCheckBox->isChecked());
        };
        connect(m_predictionModeCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this, updateModeControls](int index) {
                m_predictionModeCombo->setToolTip(
                    m_predictionModeCombo->itemData(index, Qt::ToolTipRole).toString());
                updateModeControls();
                emit localPreviewParametersChanged();
            });
        connect(m_overrideSpatialGridCellSizeCheckBox, &QCheckBox::toggled,
            this, [this](bool checked) {
                m_spatialGridCellSizeSpinBox->setEnabled(
                    spatialInfluenceFilteringEnabled() && checked);
                emit spatialGridParametersChanged();
            });
        connect(m_spatialGridCellSizeSpinBox,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double) { emit spatialGridParametersChanged(); });
        connect(m_periodicSectorCountSpinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            [this](int) { emit localPreviewParametersChanged(); });
        connect(m_axisymmetricProfileSampleCountSpinBox,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            [this](int) { emit axisymmetricProfileSampleCountChanged(); });
        connect(m_periodicAxisCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                emit rotationAxisChanged();
                emit localPreviewParametersChanged();
            });
        connect(m_pickRotationSurfaceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::rotationSurfacePickRequested);
        connect(m_selectProfileRegionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::profileRegionSelectionRequested);
        connect(m_selectAdaptiveRegionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::adaptiveRegionSelectionRequested);
        connect(m_adaptiveMeshSimplificationSpinBox,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double) { emit localPreviewParametersChanged(); });
        connect(m_previewLocalInputsButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::localInputPreviewRequested);
        connect(m_showCylindricalSurfaceCheckBox, &QCheckBox::toggled,
            this, [this](bool) { emitLocalDebugVisibilityChanged(); });
        connect(m_showRotationAxisCheckBox, &QCheckBox::toggled,
            this, [this](bool) { emitLocalDebugVisibilityChanged(); });
        connect(m_showLocalSectorCheckBox, &QCheckBox::toggled,
            this, [this](bool) { emitLocalDebugVisibilityChanged(); });
        connect(m_showLocalSprayPointsCheckBox, &QCheckBox::toggled,
            this, [this](bool) { emitLocalDebugVisibilityChanged(); });
        updateModeControls();

        // Prediction run.
        auto* runGroup = new QGroupBox(QStringLiteral("Prediction"), this);
        auto* runLayout = new QVBoxLayout(runGroup);
        runLayout->setContentsMargins(8, 6, 8, 6);
        runLayout->setSpacing(4);
        m_predictionButton = new QPushButton(QStringLiteral("Start Prediction"), runGroup);
        m_cancelButton = new QPushButton(QStringLiteral("Cancel"), runGroup);
        m_cancelButton->setVisible(false);
        m_progressBar = new QProgressBar(runGroup);
        m_progressBar->setRange(0, 1000);
        m_progressBar->setTextVisible(false);
        m_statusLabel = new QLabel(QStringLiteral("Load a model and trajectory to begin."), runGroup);
        m_statusLabel->setWordWrap(true);
        runLayout->addWidget(m_predictionButton);
        runLayout->addWidget(m_cancelButton);
        runLayout->addWidget(m_progressBar);
        runLayout->addWidget(m_statusLabel);
        rootLayout->addWidget(runGroup);
        rootLayout->addStretch(1);

        connect(m_predictionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::predictionRequested);
        connect(m_cancelButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::cancelPredictionRequested);

        m_validationGroup = new QGroupBox(QStringLiteral("Result Validation"), this);
        auto* validationLayout = new QVBoxLayout(m_validationGroup);
        validationLayout->setContentsMargins(8, 6, 8, 6);
        validationLayout->setSpacing(4);
        auto* referenceButtonsLayout = new QHBoxLayout();
        referenceButtonsLayout->setContentsMargins(0, 0, 0, 0);
        referenceButtonsLayout->setSpacing(6);
        m_setReferenceButton = new QPushButton(
            QStringLiteral("Set as Reference"), m_validationGroup);
        m_clearReferenceButton = new QPushButton(
            QStringLiteral("Clear Reference"), m_validationGroup);
        m_checkReferenceButton = new QPushButton(
            QStringLiteral("Check Against Reference"), m_validationGroup);
        m_referenceStatusLabel = new QLabel(
            QStringLiteral("Reference: not set"), m_validationGroup);
        m_referenceStatusLabel->setWordWrap(true);
        referenceButtonsLayout->addWidget(m_setReferenceButton);
        referenceButtonsLayout->addWidget(m_clearReferenceButton);
        validationLayout->addLayout(referenceButtonsLayout);
        validationLayout->addWidget(m_checkReferenceButton);
        validationLayout->addWidget(m_referenceStatusLabel);
        rootLayout->insertWidget(rootLayout->count() - 1, m_validationGroup);

        connect(m_setReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::setReferenceRequested);
        connect(m_clearReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::clearReferenceRequested);
        connect(m_checkReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::checkReferenceRequested);

        refreshDepositionCurve();
    }

    void CoatingAnalysisPanel::applyViewModel(const CoatingAnalysisViewModel& viewModel)
    {
        {
            const QSignalBlocker blocker(m_workpieceCombo);
            bool itemsChanged = m_workpieceCombo->count() != viewModel.workpieces.size();
            if(!itemsChanged) {
                for(int i = 0; i < m_workpieceCombo->count(); ++i) {
                    if(m_workpieceCombo->itemData(i).toString() != viewModel.workpieces[i].id ||
                        m_workpieceCombo->itemText(i) != viewModel.workpieces[i].name) {
                        itemsChanged = true;
                        break;
                    }
                }
            }
            if(itemsChanged) {
                m_workpieceCombo->clear();
                for(const CoatingAnalysisWorkpieceItem& item : viewModel.workpieces) {
                    m_workpieceCombo->addItem(item.name, item.id);
                }
            }
            m_workpieceCombo->setCurrentIndex(
                m_workpieceCombo->findData(viewModel.selectedWorkpieceId));
        }
        m_workpieceCombo->setEnabled(
            !viewModel.workpieces.isEmpty() && !viewModel.predictionRunning);
        m_openModelButton->setEnabled(!viewModel.predictionRunning);
        m_openTrajectoryButton->setEnabled(!viewModel.predictionRunning);
        m_selectModelButton->setEnabled(!viewModel.predictionRunning);
        m_selectTrajectoryButton->setEnabled(!viewModel.predictionRunning);
        m_trajectorySamplingCombo->setEnabled(!viewModel.predictionRunning);
        m_timeStepSpinBox->setEnabled(
            !viewModel.predictionRunning
            && trajectorySamplingMode()
                == spraythickness::TrajectorySamplingMode::ResampleByTimeStep);
        m_algorithmCombo->setEnabled(!viewModel.predictionRunning);
        m_bvhCheckBox->setEnabled(!viewModel.predictionRunning);
        m_historyCheckBox->setEnabled(!viewModel.predictionRunning);
        m_predictionModeCombo->setEnabled(
            viewModel.hasModel && viewModel.hasTrajectory
            && !viewModel.predictionRunning);
        const bool spatialMode = spatialInfluenceFilteringEnabled();
        const bool adaptiveMode = adaptiveMeshPredictionEnabled();
        const bool localCandidateMode = localCandidateVertexPredictionEnabled();
        m_spatialGridOptionsWidget->setVisible(spatialMode);
        m_adaptiveMeshOptionsWidget->setVisible(adaptiveMode || localCandidateMode);
        m_selectAdaptiveRegionButton->setEnabled(
            (adaptiveMode || localCandidateMode) && !viewModel.predictionRunning);
        m_adaptiveMeshSimplificationSpinBox->setVisible(adaptiveMode);
        m_adaptiveMeshSimplificationSpinBox->setEnabled(
            adaptiveMode && !viewModel.predictionRunning);
        m_overrideSpatialGridCellSizeCheckBox->setEnabled(
            spatialMode && !viewModel.predictionRunning);
        m_spatialGridCellSizeSpinBox->setEnabled(
            spatialMode && !viewModel.predictionRunning
            && m_overrideSpatialGridCellSizeCheckBox->isChecked());
        m_localConfigGroup->setVisible(
            viewModel.localMode || viewModel.axisymmetricProfileMode
            || viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode);
        m_rotationAxisStatusLabel->setText(
            QStringLiteral("Rotation axis: %1").arg(viewModel.rotationAxisSource));
        m_pickRotationSurfaceButton->setEnabled(
            viewModel.hasModel && (viewModel.localMode
                || viewModel.axisymmetricProfileMode
                || viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode)
            && !viewModel.predictionRunning);
        m_selectProfileRegionButton->setVisible(viewModel.axisymmetricProfileMode);
        m_selectProfileRegionButton->setEnabled(
            viewModel.canSelectProfileRegion && !viewModel.predictionRunning);
        m_previewLocalInputsButton->setEnabled(
            viewModel.canPreviewLocalInputs && viewModel.localMode
            && !viewModel.predictionRunning);
        m_selectAdaptiveRegionButton->setVisible(
            viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode);
        m_selectAdaptiveRegionButton->setEnabled(
            (viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode)
            && viewModel.hasEffectiveRotationAxis
            && !viewModel.predictionRunning);
        m_previewLocalInputsButton->setText(
            viewModel.hasLocalPreview
                ? QStringLiteral("Refresh local input preview")
                : QStringLiteral("Preview local prediction inputs"));
        const bool periodicControlsEnabled = !viewModel.predictionRunning
            && (viewModel.localMode || viewModel.axisymmetricProfileMode
                || viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode);
        m_periodicAxisCombo->setEnabled(periodicControlsEnabled);
        m_periodicSectorCountSpinBox->setVisible(viewModel.localMode);
        m_periodicSectorLabel->setVisible(viewModel.localMode);
        m_periodicSectorCountSpinBox->setEnabled(
            periodicControlsEnabled && viewModel.localMode);
        m_axisymmetricProfileSampleCountLabel->setVisible(viewModel.axisymmetricProfileMode);
        m_axisymmetricProfileSampleCountSpinBox->setVisible(
            viewModel.axisymmetricProfileMode);
        m_axisymmetricProfileSampleCountSpinBox->setEnabled(
            !viewModel.predictionRunning && viewModel.axisymmetricProfileMode);
        m_previewLocalInputsButton->setVisible(viewModel.localMode);
        const bool debugControlsEnabled = periodicControlsEnabled
            && (viewModel.hasLocalPreview || viewModel.hasAxisymmetricProfileSelection
                || ((viewModel.adaptiveMeshMode || viewModel.localCandidateVertexMode)
                    && viewModel.hasEffectiveRotationAxis));
        m_showCylindricalSurfaceCheckBox->setEnabled(debugControlsEnabled);
        m_showRotationAxisCheckBox->setEnabled(debugControlsEnabled);
        m_showLocalSectorCheckBox->setEnabled(debugControlsEnabled);
        m_showLocalSprayPointsCheckBox->setEnabled(debugControlsEnabled);
        {
            const QSignalBlocker cylindricalBlocker(m_showCylindricalSurfaceCheckBox);
            const QSignalBlocker axisBlocker(m_showRotationAxisCheckBox);
            const QSignalBlocker sectorBlocker(m_showLocalSectorCheckBox);
            const QSignalBlocker sprayBlocker(m_showLocalSprayPointsCheckBox);
            m_showCylindricalSurfaceCheckBox->setChecked(viewModel.showCylindricalSurface);
            m_showRotationAxisCheckBox->setChecked(viewModel.showRotationAxis);
            m_showLocalSectorCheckBox->setChecked(viewModel.showLocalSector);
            m_showLocalSprayPointsCheckBox->setChecked(viewModel.showLocalSprayPoints);
        }
        m_predictionButton->setEnabled(
            viewModel.canStartPrediction && !viewModel.predictionRunning);
        m_cancelButton->setVisible(viewModel.predictionRunning);
        m_progressBar->setVisible(viewModel.predictionRunning);
        m_progressBar->setValue(static_cast<int>(viewModel.progress * 1000.0));
        m_statusLabel->setText(viewModel.status);
        m_validationGroup->setVisible(viewModel.hasModel && viewModel.hasTrajectory);
        m_setReferenceButton->setVisible(true);
        m_clearReferenceButton->setVisible(true);
        m_checkReferenceButton->setVisible(true);
        m_setReferenceButton->setEnabled(
            viewModel.canSetReference && !viewModel.predictionRunning);
        m_clearReferenceButton->setEnabled(
            viewModel.canClearReference && !viewModel.predictionRunning);
        m_checkReferenceButton->setEnabled(
            viewModel.canCheckReference && !viewModel.predictionRunning);
        m_referenceStatusLabel->setText(viewModel.referenceStatus);
    }

    spraythickness::ThicknessModelKind CoatingAnalysisPanel::thicknessModel() const
    {
        return static_cast<spraythickness::ThicknessModelKind>(
            m_algorithmCombo->currentData().toInt());
    }

    spraythickness::TrajectorySamplingMode CoatingAnalysisPanel::trajectorySamplingMode() const
    {
        return static_cast<spraythickness::TrajectorySamplingMode>(
            m_trajectorySamplingCombo->currentData().toInt());
    }

    double CoatingAnalysisPanel::timeStepSeconds() const
    {
        return m_timeStepSpinBox->value();
    }

    bool CoatingAnalysisPanel::bvhOcclusionEnabled() const
    {
        return m_bvhCheckBox->isChecked();
    }

    bool CoatingAnalysisPanel::historyCorrectionEnabled() const
    {
        return m_historyCheckBox->isChecked();
    }

    bool CoatingAnalysisPanel::periodicLocalPredictionEnabled() const
    {
        const PredictionInputMode mode = predictionInputMode();
        return mode == PredictionInputMode::LocalAllSprayPoints
            || mode == PredictionInputMode::LocalSpatialFilteredSprayPoints;
    }

    bool CoatingAnalysisPanel::axisymmetricProfilePredictionEnabled() const
    {
        return predictionInputMode()
            == PredictionInputMode::AxisymmetricProfileSpatialFilteredSprayPoints;
    }

    bool CoatingAnalysisPanel::rotationBasedPredictionEnabled() const
    {
        return periodicLocalPredictionEnabled() || axisymmetricProfilePredictionEnabled()
            || adaptiveMeshPredictionEnabled() || localCandidateVertexPredictionEnabled();
    }

    PredictionInputMode CoatingAnalysisPanel::predictionInputMode() const
    {
        return static_cast<PredictionInputMode>(
            m_predictionModeCombo->currentData().toInt());
    }

    bool CoatingAnalysisPanel::spatialInfluenceFilteringEnabled() const
    {
        const PredictionInputMode mode = predictionInputMode();
        return mode == PredictionInputMode::CompleteSpatialFilteredSprayPoints
            || mode == PredictionInputMode::CompleteSpatialFilteredCandidateVertices
            || mode == PredictionInputMode::LocalSpatialFilteredSprayPoints
            || mode == PredictionInputMode::AxisymmetricProfileSpatialFilteredSprayPoints
            || mode == PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices
            || mode == PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh;
    }

    bool CoatingAnalysisPanel::spatialCandidateVertexFilteringEnabled() const
    {
        return predictionInputMode()
            == PredictionInputMode::CompleteSpatialFilteredCandidateVertices
            || predictionInputMode()
                == PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices
            || predictionInputMode()
                == PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh;
    }

    bool CoatingAnalysisPanel::adaptiveMeshPredictionEnabled() const
    {
        return predictionInputMode()
            == PredictionInputMode::AdaptiveMeshSpatialFilteredCandidateVertices;
    }

    bool CoatingAnalysisPanel::localCandidateVertexPredictionEnabled() const
    {
        return predictionInputMode()
            == PredictionInputMode::LocalSpatialFilteredCandidateVerticesFullBvh;
    }

    double CoatingAnalysisPanel::adaptiveMeshSimplificationPercent() const
    {
        return m_adaptiveMeshSimplificationSpinBox->value();
    }

    bool CoatingAnalysisPanel::overrideSpatialGridCellSize() const
    {
        return spatialInfluenceFilteringEnabled()
            && m_overrideSpatialGridCellSizeCheckBox->isChecked();
    }

    double CoatingAnalysisPanel::spatialGridCellSizeMillimeters() const
    {
        return m_spatialGridCellSizeSpinBox->value();
    }

    void CoatingAnalysisPanel::setPeriodicLocalPredictionEnabled(bool enabled)
    {
        if(m_predictionModeCombo != nullptr) {
            m_predictionModeCombo->setCurrentIndex(enabled ? 1 : 0);
        }
    }

    Eigen::Vector3d CoatingAnalysisPanel::periodicAxisDirection() const
    {
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        axis[m_periodicAxisCombo->currentData().toInt()] = 1.0;
        return axis;
    }

    std::size_t CoatingAnalysisPanel::periodicSectorCount() const
    {
        return static_cast<std::size_t>(m_periodicSectorCountSpinBox->value());
    }

    std::size_t CoatingAnalysisPanel::axisymmetricProfileSampleCount() const
    {
        return static_cast<std::size_t>(m_axisymmetricProfileSampleCountSpinBox->value());
    }

    QString CoatingAnalysisPanel::selectedWorkpieceId() const
    {
        return m_workpieceCombo->currentData().toString();
    }

    void CoatingAnalysisPanel::refreshDepositionCurve()
    {
        if(m_curveWidget != nullptr) {
            m_curveWidget->showPaperGaussian();
        }
    }

    void CoatingAnalysisPanel::emitLocalDebugVisibilityChanged()
    {
        emit localDebugVisibilityChanged(
            m_showCylindricalSurfaceCheckBox->isChecked(),
            m_showRotationAxisCheckBox->isChecked(),
            m_showLocalSectorCheckBox->isChecked(),
            m_showLocalSprayPointsCheckBox->isChecked());
    }
}
