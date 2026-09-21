#include "CoatingAnalysisPanel.h"

#include "DepositionCurveWidget.h"
#include "CoatingAnalysisLanguage.h"
#include "PublishedReproductionAdapter.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QTabBar>
#include <QVBoxLayout>
#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QtMath>

#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        enum LocalAxisId
        {
            PositiveX = 0,
            NegativeX,
            PositiveY,
            NegativeY,
            PositiveZ,
            NegativeZ
        };

        Eigen::Vector3d localAxisVector(int axisId)
        {
            switch(axisId) {
            case NegativeX:
                return -Eigen::Vector3d::UnitX();
            case PositiveY:
                return Eigen::Vector3d::UnitY();
            case NegativeY:
                return -Eigen::Vector3d::UnitY();
            case PositiveZ:
                return Eigen::Vector3d::UnitZ();
            case NegativeZ:
                return -Eigen::Vector3d::UnitZ();
            case PositiveX:
            default:
                return Eigen::Vector3d::UnitX();
            }
        }

        void populateLocalAxisCombo(QComboBox& combo, int defaultAxisId)
        {
            combo.addItem(QStringLiteral("+X"), PositiveX);
            combo.addItem(QStringLiteral("-X"), NegativeX);
            combo.addItem(QStringLiteral("+Y"), PositiveY);
            combo.addItem(QStringLiteral("-Y"), NegativeY);
            combo.addItem(QStringLiteral("+Z"), PositiveZ);
            combo.addItem(QStringLiteral("-Z"), NegativeZ);
            combo.setCurrentIndex(combo.findData(defaultAxisId));
            combo.setMinimumWidth(0);
            combo.setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        }
    }

    CoatingAnalysisPanel::CoatingAnalysisPanel(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(0, 0);
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        auto* rootLayout = new QVBoxLayout(this);
        rootLayout->setContentsMargins(6, 4, 6, 6);
        rootLayout->setSpacing(6);

        m_simulationGroup = new QGroupBox(QStringLiteral("Spray Simulation"), this);
        auto* simulationLayout = new QVBoxLayout(m_simulationGroup);
        simulationLayout->setContentsMargins(8, 6, 8, 6);
        simulationLayout->setSpacing(4);
        auto* simulationForm = new QFormLayout();
        simulationForm->setHorizontalSpacing(6);
        simulationForm->setVerticalSpacing(3);
        simulationForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        m_simulationKindCombo = new QComboBox(m_simulationGroup);
        m_simulationKindCombo->addItem(QStringLiteral("Point spray"),
            static_cast<int>(SimulationExperimentKind::PointSpray));
        m_simulationKindCombo->addItem(QStringLiteral("Line scan"),
            static_cast<int>(SimulationExperimentKind::LineScan));
        m_simulationKindCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        auto makeSimulationSpin = [this](double minimum, double maximum, double value,
                                         int decimals, double step, const QString& suffix) {
            auto* spin = new QDoubleSpinBox(m_simulationGroup);
            spin->setRange(minimum, maximum);
            spin->setDecimals(decimals);
            spin->setSingleStep(step);
            spin->setValue(value);
            spin->setSuffix(suffix);
            spin->setMinimumWidth(0);
            spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            return spin;
        };
        m_plateSideSpinBox = makeSimulationSpin(1.0, 100000.0, 200.0, 2, 10.0, QStringLiteral(" mm"));
        m_plateCellSpinBox = makeSimulationSpin(0.01, 10000.0, 2.0, 3, 0.5, QStringLiteral(" mm"));
        m_simulationDistanceSpinBox = makeSimulationSpin(0.01, 10000.0, 100.0, 2, 5.0, QStringLiteral(" mm"));
        m_trajectoryOverrunSpinBox = makeSimulationSpin(0.1, 100000.0, 20.0, 2, 5.0, QStringLiteral(" mm"));
        m_simulationIncidenceSpinBox = makeSimulationSpin(0.0, 90.0, 90.0, 1, 1.0, QStringLiteral(" deg"));
        m_simulationAzimuthSpinBox = makeSimulationSpin(-180.0, 180.0, 0.0, 1, 5.0, QStringLiteral(" deg"));
        m_simulationToolRollSpinBox = makeSimulationSpin(-180.0, 180.0, 0.0, 1, 5.0, QStringLiteral(" deg"));
        m_pointDurationSpinBox = makeSimulationSpin(0.001, 3600.0, 4.0, 3, 0.1, QStringLiteral(" s"));
        m_scanSpeedSpinBox = makeSimulationSpin(0.01, 100000.0, 20.0, 2, 1.0, QStringLiteral(" mm/s"));
        m_scanPassCountSpinBox = new QSpinBox(m_simulationGroup);
        m_scanPassCountSpinBox->setRange(1, 1000);
        m_scanPassCountSpinBox->setSingleStep(1);
        m_scanPassCountSpinBox->setValue(1);
        m_scanPassCountSpinBox->setSuffix(QStringLiteral(" passes"));
        m_scanPassCountSpinBox->setMinimumWidth(0);
        m_scanPassCountSpinBox->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        m_entrySpeedSpinBox = makeSimulationSpin(0.01, 100000.0, 20.0, 1, 1.0, QStringLiteral(" mm/s"));
        m_exitSpeedSpinBox = makeSimulationSpin(0.01, 100000.0, 20.0, 1, 1.0, QStringLiteral(" mm/s"));
        m_trajectoryPointIntervalSpinBox = makeSimulationSpin(0.001, 10.0, 0.02, 3, 0.01, QStringLiteral(" s"));
        m_scanStartXSpinBox = makeSimulationSpin(-100000.0, 100000.0, -50.0, 2, 5.0, QStringLiteral(" mm"));
        m_scanStartYSpinBox = makeSimulationSpin(-100000.0, 100000.0, 0.0, 2, 5.0, QStringLiteral(" mm"));
        m_scanEndXSpinBox = makeSimulationSpin(-100000.0, 100000.0, 50.0, 2, 5.0, QStringLiteral(" mm"));
        m_scanEndYSpinBox = makeSimulationSpin(-100000.0, 100000.0, 0.0, 2, 5.0, QStringLiteral(" mm"));
        const auto addSimulationRow = [simulationForm](const QString& label, QWidget* field) {
            simulationForm->addRow(label, field);
            return simulationForm->labelForField(field);
        };
        addSimulationRow(QStringLiteral("Experiment"), m_simulationKindCombo);
        addSimulationRow(QStringLiteral("Plate side"), m_plateSideSpinBox);
        addSimulationRow(QStringLiteral("Grid cell"), m_plateCellSpinBox);
        addSimulationRow(QStringLiteral("Spray distance"), m_simulationDistanceSpinBox);
        addSimulationRow(QStringLiteral("Trajectory overrun"), m_trajectoryOverrunSpinBox);
        addSimulationRow(QStringLiteral("Incidence angle"), m_simulationIncidenceSpinBox);
        addSimulationRow(QStringLiteral("Azimuth"), m_simulationAzimuthSpinBox);
        addSimulationRow(QStringLiteral("Tool roll (local Z)"), m_simulationToolRollSpinBox);
        QWidget* pointDurationLabel =
            addSimulationRow(QStringLiteral("Point duration"), m_pointDurationSpinBox);
        QWidget* scanSpeedLabel =
            addSimulationRow(QStringLiteral("Scan speed"), m_scanSpeedSpinBox);
        QWidget* scanPassCountLabel = addSimulationRow(
            QStringLiteral("Scan passes (round trips)"), m_scanPassCountSpinBox);
        QWidget* entrySpeedLabel =
            addSimulationRow(QStringLiteral("Entry speed"), m_entrySpeedSpinBox);
        QWidget* exitSpeedLabel =
            addSimulationRow(QStringLiteral("Exit speed"), m_exitSpeedSpinBox);
        addSimulationRow(QStringLiteral("Trajectory point interval"),
            m_trajectoryPointIntervalSpinBox);
        QWidget* scanStartXLabel =
            addSimulationRow(QStringLiteral("Scan start X"), m_scanStartXSpinBox);
        QWidget* scanStartYLabel =
            addSimulationRow(QStringLiteral("Scan start Y"), m_scanStartYSpinBox);
        QWidget* scanEndXLabel =
            addSimulationRow(QStringLiteral("Scan end X"), m_scanEndXSpinBox);
        QWidget* scanEndYLabel =
            addSimulationRow(QStringLiteral("Scan end Y"), m_scanEndYSpinBox);
        simulationLayout->addLayout(simulationForm);
        auto* simulationButtons = new QGridLayout();
        m_runSimulationButton = new QPushButton(QStringLiteral("Run simulation"), m_simulationGroup);
        m_exportSimulationButton = new QPushButton(QStringLiteral("Export CSV"), m_simulationGroup);
        for(QPushButton* button : { m_runSimulationButton, m_exportSimulationButton }) {
            button->setMinimumWidth(0);
            button->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        }
        simulationButtons->addWidget(m_runSimulationButton, 0, 0, 1, 2);
        simulationButtons->addWidget(m_exportSimulationButton, 1, 0, 1, 2);
        simulationLayout->addLayout(simulationButtons);
        m_simulationGroup->setMinimumSize(0, 0);
        m_simulationGroup->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        rootLayout->addWidget(m_simulationGroup);
        const auto simulationParameterChanged = [this]() { emit simulationParametersChanged(); };
        const auto updateSimulationModeUi = [this, simulationParameterChanged,
            pointDurationLabel, scanSpeedLabel, scanPassCountLabel,
            entrySpeedLabel, exitSpeedLabel,
            scanStartXLabel, scanStartYLabel, scanEndXLabel, scanEndYLabel](int) {
                const bool line = m_simulationKindCombo->currentData().toInt()
                    == static_cast<int>(SimulationExperimentKind::LineScan);
                const auto setRowVisible = [](QWidget* label, QWidget* field, bool visible) {
                    if(label != nullptr) {
                        label->setVisible(visible);
                    }
                    field->setVisible(visible);
                };
                setRowVisible(pointDurationLabel, m_pointDurationSpinBox, !line);
                setRowVisible(scanSpeedLabel, m_scanSpeedSpinBox, line);
                setRowVisible(scanPassCountLabel, m_scanPassCountSpinBox, line);
                // Both experiments start and stop spraying outside the plate.
                // Point spraying differs only in the center dwell, so its
                // entry/exit speeds are also part of the trajectory controls.
                setRowVisible(entrySpeedLabel, m_entrySpeedSpinBox, true);
                setRowVisible(exitSpeedLabel, m_exitSpeedSpinBox, true);
                setRowVisible(scanStartXLabel, m_scanStartXSpinBox, line);
                setRowVisible(scanStartYLabel, m_scanStartYSpinBox, line);
                setRowVisible(scanEndXLabel, m_scanEndXSpinBox, line);
                setRowVisible(scanEndYLabel, m_scanEndYSpinBox, line);
                simulationParameterChanged();
            };
        connect(m_simulationKindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, updateSimulationModeUi);
        connect(m_scanPassCountSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, simulationParameterChanged);
        for(QDoubleSpinBox* spin : { m_plateSideSpinBox, m_plateCellSpinBox,
            m_simulationDistanceSpinBox, m_trajectoryOverrunSpinBox,
            m_simulationIncidenceSpinBox,
            m_simulationAzimuthSpinBox, m_simulationToolRollSpinBox,
            m_pointDurationSpinBox, m_scanSpeedSpinBox,
            m_entrySpeedSpinBox, m_exitSpeedSpinBox, m_trajectoryPointIntervalSpinBox,
            m_scanStartXSpinBox,
            m_scanStartYSpinBox, m_scanEndXSpinBox, m_scanEndYSpinBox }) {
            connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, simulationParameterChanged);
        }
        connect(m_runSimulationButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::simulationPredictionRequested);
        connect(m_exportSimulationButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::simulationExportRequested);
        m_runSimulationButton->setEnabled(false);
        m_exportSimulationButton->setEnabled(false);
        m_simulationKindCombo->setCurrentIndex(0);
        updateSimulationModeUi(m_simulationKindCombo->currentIndex());

        m_reproductionGroup = new QGroupBox(
            QStringLiteral("Algorithm Reproduction"), this);
        auto* reproductionLayout = new QVBoxLayout(m_reproductionGroup);
        reproductionLayout->setContentsMargins(8, 6, 8, 6);
        reproductionLayout->setSpacing(4);
        auto* reproductionForm = new QFormLayout();
        reproductionForm->setHorizontalSpacing(6);
        reproductionForm->setVerticalSpacing(3);
        reproductionForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        m_reproductionAlgorithmCombo = new QComboBox(m_reproductionGroup);
        const auto addReproductionAlgorithm = [this](
                                                  spraythickness::ReproductionAlgorithmKind kind) {
            m_reproductionAlgorithmCombo->addItem(
                QString::fromLatin1(spraythickness::reproductionAlgorithmName(kind)),
                static_cast<int>(kind));
        };
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::CurrentMethod);
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::Tanaka2024);
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::Tzinava2020);
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::Wu2020);
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::Fuke2005);
        addReproductionAlgorithm(spraythickness::ReproductionAlgorithmKind::Vanerio2021);
        addReproductionAlgorithm(
            spraythickness::ReproductionAlgorithmKind::DynamicSurface2026);
        m_reproductionConfigurationEdit = new QLineEdit(m_reproductionGroup);
        m_reproductionConfigurationEdit->setReadOnly(true);
        m_reproductionConfigurationEdit->setPlaceholderText(
            QStringLiteral("Required paper-specific JSON calibration"));
        m_selectReproductionConfigurationButton = new QPushButton(
            QStringLiteral("Browse..."), m_reproductionGroup);
        m_createReproductionTemplateButton = new QPushButton(
            QStringLiteral("Create template..."), m_reproductionGroup);
        auto* configurationWidget = new QWidget(m_reproductionGroup);
        auto* configurationLayout = new QHBoxLayout(configurationWidget);
        configurationLayout->setContentsMargins(0, 0, 0, 0);
        configurationLayout->setSpacing(4);
        configurationLayout->addWidget(m_reproductionConfigurationEdit, 1);
        configurationLayout->addWidget(m_selectReproductionConfigurationButton);
        configurationLayout->addWidget(m_createReproductionTemplateButton);
        const auto addReproductionRow = [reproductionForm](
                                              const QString& label,
                                              QWidget* field) {
            reproductionForm->addRow(label, field);
            return reproductionForm->labelForField(field);
        };
        addReproductionRow(QStringLiteral("Published algorithm"),
            m_reproductionAlgorithmCombo);
        QWidget* configurationLabel = addReproductionRow(
            QStringLiteral("Calibration file"), configurationWidget);
        m_reproductionModelInputLabel = new QLabel(m_reproductionGroup);
        m_reproductionModelInputLabel->setWordWrap(true);
        m_reproductionTrajectoryInputLabel = new QLabel(m_reproductionGroup);
        m_reproductionTrajectoryInputLabel->setWordWrap(true);
        addReproductionRow(QStringLiteral("Model input"),
            m_reproductionModelInputLabel);
        addReproductionRow(QStringLiteral("Trajectory input"),
            m_reproductionTrajectoryInputLabel);

        m_tzinavaRotationWidget = new QWidget(m_reproductionGroup);
        auto* rotationForm = new QFormLayout(m_tzinavaRotationWidget);
        rotationForm->setContentsMargins(0, 0, 0, 0);
        rotationForm->setHorizontalSpacing(4);
        rotationForm->setVerticalSpacing(3);
        const auto makeRotationSpin = [this](double minimum, double maximum,
                                              double value, int decimals,
                                              const QString& suffix) {
            auto* spin = new QDoubleSpinBox(m_tzinavaRotationWidget);
            spin->setRange(minimum, maximum);
            spin->setDecimals(decimals);
            spin->setValue(value);
            spin->setSuffix(suffix);
            spin->setMinimumWidth(0);
            spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            return spin;
        };
        m_tzinavaRotationOriginXSpinBox = makeRotationSpin(
            -1000000.0, 1000000.0, 0.0, 3, QStringLiteral(" mm"));
        m_tzinavaRotationOriginYSpinBox = makeRotationSpin(
            -1000000.0, 1000000.0, 0.0, 3, QStringLiteral(" mm"));
        m_tzinavaRotationOriginZSpinBox = makeRotationSpin(
            -1000000.0, 1000000.0, 0.0, 3, QStringLiteral(" mm"));
        m_tzinavaRotationAxisXSpinBox = makeRotationSpin(
            -1.0, 1.0, 0.0, 4, QString());
        m_tzinavaRotationAxisYSpinBox = makeRotationSpin(
            -1.0, 1.0, 0.0, 4, QString());
        m_tzinavaRotationAxisZSpinBox = makeRotationSpin(
            -1.0, 1.0, 1.0, 4, QString());
        m_tzinavaAngularSpeedSpinBox = makeRotationSpin(
            -36000.0, 36000.0, 0.0, 3, QStringLiteral(" deg/s"));
        const auto vectorEditor = [this](QDoubleSpinBox* x,
                                      QDoubleSpinBox* y,
                                      QDoubleSpinBox* z) {
            auto* editor = new QWidget(m_tzinavaRotationWidget);
            auto* layout = new QHBoxLayout(editor);
            layout->setContentsMargins(0, 0, 0, 0);
            layout->setSpacing(3);
            layout->addWidget(x);
            layout->addWidget(y);
            layout->addWidget(z);
            return editor;
        };
        rotationForm->addRow(QStringLiteral("Origin XYZ"),
            vectorEditor(m_tzinavaRotationOriginXSpinBox,
                m_tzinavaRotationOriginYSpinBox,
                m_tzinavaRotationOriginZSpinBox));
        rotationForm->addRow(QStringLiteral("Axis XYZ"),
            vectorEditor(m_tzinavaRotationAxisXSpinBox,
                m_tzinavaRotationAxisYSpinBox,
                m_tzinavaRotationAxisZSpinBox));
        rotationForm->addRow(QStringLiteral("Angular speed"),
            m_tzinavaAngularSpeedSpinBox);
        m_tzinavaRotationLabel = addReproductionRow(
            QStringLiteral("Object rotation"), m_tzinavaRotationWidget);
        reproductionLayout->addLayout(reproductionForm);
        auto* reproductionButtons = new QGridLayout();
        m_runReproductionButton = new QPushButton(
            QStringLiteral("Run reproduction"), m_reproductionGroup);
        m_cancelReproductionButton = new QPushButton(
            QStringLiteral("Cancel"), m_reproductionGroup);
        m_exportReproductionButton = new QPushButton(
            QStringLiteral("Export result"), m_reproductionGroup);
        reproductionButtons->addWidget(m_runReproductionButton, 0, 0, 1, 2);
        reproductionButtons->addWidget(m_cancelReproductionButton, 1, 0);
        reproductionButtons->addWidget(m_exportReproductionButton, 1, 1);
        reproductionLayout->addLayout(reproductionButtons);
        m_reproductionProgressBar = new QProgressBar(m_reproductionGroup);
        m_reproductionProgressBar->setRange(0, 1000);
        m_reproductionProgressBar->setTextVisible(true);
        m_reproductionProgressBar->setVisible(false);
        reproductionLayout->addWidget(m_reproductionProgressBar);
        m_reproductionStatusLabel = new QLabel(
            QStringLiteral("No reproduction result yet."), m_reproductionGroup);
        m_reproductionStatusLabel->setWordWrap(true);
        reproductionLayout->addWidget(m_reproductionStatusLabel);
        m_reproductionGroup->setMinimumSize(0, 0);
        m_reproductionGroup->setSizePolicy(
            QSizePolicy::Ignored, QSizePolicy::Preferred);
        rootLayout->addWidget(m_reproductionGroup);
        connect(m_runReproductionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::reproductionRequested);
        connect(m_cancelReproductionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::cancelReproductionRequested);
        connect(m_exportReproductionButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::reproductionExportRequested);
        const auto updateReproductionParameters = [this,
                                                      configurationLabel,
                                                      configurationWidget](int) {
            const auto algorithm = reproductionAlgorithm();
            const int algorithmKey = static_cast<int>(algorithm);
            if(m_reproductionConfigurationAlgorithm >= 0) {
                m_reproductionConfigurationPaths.insert(
                    m_reproductionConfigurationAlgorithm,
                    m_reproductionConfigurationEdit->text());
            }
            m_reproductionConfigurationEdit->setText(
                m_reproductionConfigurationPaths.value(algorithmKey));
            m_reproductionConfigurationAlgorithm = algorithmKey;
            const bool current = algorithm
                == spraythickness::ReproductionAlgorithmKind::CurrentMethod;
            configurationLabel->setVisible(!current);
            configurationWidget->setVisible(!current);
            const bool configurationEnabled =
                !m_reproductionControlsLocked && !current;
            m_reproductionConfigurationEdit->setEnabled(configurationEnabled);
            m_selectReproductionConfigurationButton->setEnabled(
                configurationEnabled);
            m_createReproductionTemplateButton->setEnabled(
                configurationEnabled);
            updateReproductionInputUi();
            emit reproductionInputsChanged();
        };
        connect(m_reproductionAlgorithmCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, updateReproductionParameters);
        m_reproductionAlgorithmCombo->setCurrentIndex(1);
        updateReproductionParameters(m_reproductionAlgorithmCombo->currentIndex());
        connect(m_selectReproductionConfigurationButton,
            &QPushButton::clicked, this, [this]() {
                const QString path = QFileDialog::getOpenFileName(this,
                    QStringLiteral("Select paper calibration"),
                    m_reproductionConfigurationEdit->text(),
                    QStringLiteral("JSON files (*.json)"));
                if(!path.isEmpty()) {
                    setReproductionConfigurationPath(path);
                }
            });
        connect(m_createReproductionTemplateButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::reproductionTemplateRequested);
        for(QDoubleSpinBox* spin : {
            m_tzinavaRotationOriginXSpinBox,
            m_tzinavaRotationOriginYSpinBox,
            m_tzinavaRotationOriginZSpinBox,
            m_tzinavaRotationAxisXSpinBox,
            m_tzinavaRotationAxisYSpinBox,
            m_tzinavaRotationAxisZSpinBox,
            m_tzinavaAngularSpeedSpinBox }) {
            connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double) {
                    emit reproductionInputsChanged();
                });
        }

        m_modeTabBar = new QTabBar(this);
        m_modeTabBar->setMinimumWidth(0);
        m_modeTabBar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        m_modeTabBar->setExpanding(false);
        m_modeTabBar->setUsesScrollButtons(true);
        m_modeTabBar->setElideMode(Qt::ElideRight);
        m_modeTabBar->addTab(QStringLiteral("Thickness Prediction"));
        m_modeTabBar->addTab(QStringLiteral("Thickness Simulation"));
        m_modeTabBar->addTab(QStringLiteral("Algorithm Reproduction"));
        rootLayout->insertWidget(0, m_modeTabBar);

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
        m_workpieceCombo->setMinimumContentsLength(0);
        workpieceForm->addRow(QStringLiteral("Target"), m_workpieceCombo);
        rootLayout->addWidget(workpieceGroup);
        m_sharedSections.push_back(workpieceGroup);
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
        for(QPushButton* button : { m_openModelButton, m_openTrajectoryButton,
            m_selectModelButton, m_selectTrajectoryButton }) {
            button->setMinimumWidth(0);
            button->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        }
        dataLayout->addWidget(m_openModelButton, 0, 0);
        dataLayout->addWidget(m_openTrajectoryButton, 0, 1);
        dataLayout->addWidget(m_selectModelButton, 1, 0);
        dataLayout->addWidget(m_selectTrajectoryButton, 1, 1);
        rootLayout->addWidget(dataGroup);
        m_sharedSections.push_back(dataGroup);
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
        m_sprayDirectionCombo = new QComboBox(algorithmGroup);
        m_powderFeedDirectionCombo = new QComboBox(algorithmGroup);
        populateLocalAxisCombo(*m_sprayDirectionCombo, PositiveZ);
        populateLocalAxisCombo(*m_powderFeedDirectionCombo, PositiveY);
        auto* algorithmForm = new QFormLayout();
        algorithmForm->setHorizontalSpacing(6);
        algorithmForm->setVerticalSpacing(4);
        algorithmForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        algorithmForm->addRow(QStringLiteral("Model"), m_algorithmCombo);
        algorithmForm->addRow(QStringLiteral("Spray direction"), m_sprayDirectionCombo);
        algorithmForm->addRow(
            QStringLiteral("Powder feed direction"), m_powderFeedDirectionCombo);
        m_curveWidget = new DepositionCurveWidget(algorithmGroup);
        m_curveWidget->setFixedHeight(72);
        algorithmLayout->addLayout(algorithmForm);
        algorithmLayout->addWidget(m_curveWidget);
        rootLayout->addWidget(algorithmGroup);
        m_predictionSections.push_back(algorithmGroup);
        connect(m_algorithmCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &CoatingAnalysisPanel::refreshDepositionCurve);
        connect(m_sprayDirectionCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this]() {
                ensurePowderFeedDirectionValid();
                emit depositionDirectionsChanged();
            });
        connect(m_powderFeedDirectionCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this]() {
                ensurePowderFeedDirectionValid();
                emit depositionDirectionsChanged();
            });

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
        m_timeStepLabel = samplingForm->labelForField(m_timeStepSpinBox);
        rootLayout->addWidget(samplingGroup);
        m_sharedSections.push_back(samplingGroup);
        rootLayout->removeWidget(m_reproductionGroup);
        rootLayout->insertWidget(rootLayout->indexOf(samplingGroup) + 1,
            m_reproductionGroup);
        connect(m_trajectorySamplingCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                updateTrajectorySamplingUi();
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
        m_predictionSections.push_back(optionsGroup);

        auto* modeGroup = new QGroupBox(QStringLiteral("Prediction Input"), this);
        auto* modeLayout = new QVBoxLayout(modeGroup);
        modeLayout->setContentsMargins(8, 6, 8, 6);
        modeLayout->setSpacing(4);
        m_predictionModeCombo = new QComboBox(modeGroup);
        m_predictionModeCombo->setSizeAdjustPolicy(
            QComboBox::AdjustToMinimumContentsLengthWithIcon);
        m_predictionModeCombo->setMinimumContentsLength(0);
        m_predictionModeCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
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
        m_predictionModeCombo->setCurrentIndex(m_predictionModeCombo->findData(
            static_cast<int>(PredictionInputMode::CompleteSpatialFilteredSprayPoints)));
        m_predictionModeCombo->setToolTip(
            m_predictionModeCombo->currentData(Qt::ToolTipRole).toString());
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
        m_predictionSections.push_back(modeGroup);

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
        m_predictionSections.push_back(m_localConfigGroup);

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
            m_showLocalSectorCheckBox->setText(coatingAnalysisTranslate(m_languageCode,
                profileMode ? QStringLiteral("Profile line") : QStringLiteral("Local sector")));
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
        m_statusLabel->setTextFormat(Qt::PlainText);
        m_statusLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        m_statusLabel->setWordWrap(true);
        runLayout->addWidget(m_predictionButton);
        runLayout->addWidget(m_cancelButton);
        runLayout->addWidget(m_progressBar);
        runLayout->addWidget(m_statusLabel);
        rootLayout->addWidget(runGroup);
        m_predictionSections.push_back(runGroup);
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
        m_predictionSections.push_back(m_validationGroup);

        connect(m_setReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::setReferenceRequested);
        connect(m_clearReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::clearReferenceRequested);
        connect(m_checkReferenceButton, &QPushButton::clicked,
            this, &CoatingAnalysisPanel::checkReferenceRequested);

        connect(m_modeTabBar, &QTabBar::currentChanged,
            this, &CoatingAnalysisPanel::setModeTab);
        for(QWidget* section : m_predictionSections) {
            section->setMinimumSize(0, 0);
            section->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        }
        for(QWidget* section : m_sharedSections) {
            section->setMinimumSize(0, 0);
            section->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        }
        for(QFormLayout* form : findChildren<QFormLayout*>()) {
            form->setRowWrapPolicy(QFormLayout::WrapLongRows);
            form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        }
        setModeTab(0);

        refreshDepositionCurve();
        setLanguageCode(QStringLiteral("en"));
        updateReproductionInputUi();
        updateTrajectorySamplingUi();
    }

    void CoatingAnalysisPanel::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        const auto translate = [this](const QString& value) {
            return coatingAnalysisTranslate(m_languageCode, value);
        };
        for(QWidget* widget : findChildren<QWidget*>()) {
            if(auto* group = qobject_cast<QGroupBox*>(widget)) {
                group->setTitle(translate(group->title()));
            } else if(auto* button = qobject_cast<QAbstractButton*>(widget)) {
                button->setText(translate(button->text()));
                button->setToolTip(translate(button->toolTip()));
            } else if(auto* label = qobject_cast<QLabel*>(widget)) {
                label->setText(translate(label->text()));
                label->setToolTip(translate(label->toolTip()));
            } else if(auto* combo = qobject_cast<QComboBox*>(widget)) {
                for(int index = 0; index < combo->count(); ++index) {
                    combo->setItemText(index, translate(combo->itemText(index)));
                    combo->setItemData(index,
                        translate(combo->itemData(index, Qt::ToolTipRole).toString()),
                        Qt::ToolTipRole);
                }
                combo->setToolTip(translate(combo->toolTip()));
            } else if(auto* tabBar = qobject_cast<QTabBar*>(widget)) {
                for(int index = 0; index < tabBar->count(); ++index) {
                    tabBar->setTabText(index, translate(tabBar->tabText(index)));
                    tabBar->setTabToolTip(index, translate(tabBar->tabToolTip(index)));
                }
            }
        }
        if(m_statusLabel != nullptr) {
            m_statusLabel->setText(translate(m_statusLabel->text()));
        }
        if(m_referenceStatusLabel != nullptr) {
            m_referenceStatusLabel->setText(translate(m_referenceStatusLabel->text()));
        }
        if(m_rotationAxisStatusLabel != nullptr) {
            m_rotationAxisStatusLabel->setText(translate(m_rotationAxisStatusLabel->text()));
        }
        if(m_predictionModeCombo != nullptr) {
            m_predictionModeCombo->setToolTip(translate(
                m_predictionModeCombo->toolTip()));
        }
        updateReproductionInputUi();
    }

    void CoatingAnalysisPanel::setModeTab(int index)
    {
        const CoatingAnalysisMode nextMode = index == 1
            ? CoatingAnalysisMode::Simulation
            : (index == 2 ? CoatingAnalysisMode::Reproduction
                          : CoatingAnalysisMode::Prediction);
        const CoatingAnalysisMode previousMode = m_mode;
        m_mode = nextMode;
        const bool simulation = m_mode == CoatingAnalysisMode::Simulation;
        const bool reproduction = m_mode == CoatingAnalysisMode::Reproduction;
        m_simulationGroup->setVisible(simulation);
        m_reproductionGroup->setVisible(reproduction);
        updateReproductionInputUi();
        for(QWidget* section : m_sharedSections) {
            if(section != nullptr) {
                section->setVisible(!simulation);
            }
        }
        for(QWidget* section : m_predictionSections) {
            if(section != nullptr) {
                section->setVisible(m_mode == CoatingAnalysisMode::Prediction);
            }
        }
        if(previousMode == nextMode) {
            return;
        }
        if(previousMode == CoatingAnalysisMode::Simulation) {
            emit exitSimulationRequested();
        } else if(previousMode == CoatingAnalysisMode::Reproduction) {
            emit exitReproductionRequested();
        }
        if(nextMode == CoatingAnalysisMode::Simulation) {
            emit enterSimulationRequested();
        } else if(nextMode == CoatingAnalysisMode::Reproduction) {
            emit enterReproductionRequested();
        }
    }

    void CoatingAnalysisPanel::applyViewModel(const CoatingAnalysisViewModel& viewModel)
    {
        m_mode = viewModel.mode;
        const bool simulationActive = m_mode == CoatingAnalysisMode::Simulation;
        const bool reproductionActive = m_mode == CoatingAnalysisMode::Reproduction;
        if(m_modeTabBar != nullptr) {
            const QSignalBlocker tabBlocker(m_modeTabBar);
            m_modeTabBar->setCurrentIndex(static_cast<int>(m_mode));
            m_modeTabBar->setEnabled(!viewModel.predictionRunning);
            m_simulationGroup->setVisible(simulationActive);
            m_reproductionGroup->setVisible(reproductionActive);
            updateReproductionInputUi();
            for(QWidget* section : m_sharedSections) {
                if(section != nullptr) {
                    section->setVisible(!simulationActive);
                }
            }
            for(QWidget* section : m_predictionSections) {
                if(section != nullptr) {
                    // Set final visibility once. Showing a hidden local group on
                    // every progress update temporarily reserves its height and
                    // compresses the running status before the next layout pass.
                    bool visible = m_mode == CoatingAnalysisMode::Prediction;
                    if(section == m_localConfigGroup) {
                        visible = visible && (viewModel.localMode
                            || viewModel.axisymmetricProfileMode
                            || viewModel.adaptiveMeshMode
                            || viewModel.localCandidateVertexMode);
                    } else if(section == m_validationGroup) {
                        visible = visible && viewModel.hasModel && viewModel.hasTrajectory;
                    }
                    section->setVisible(visible);
                }
            }
        }
        const bool simulationLocked = viewModel.predictionRunning || viewModel.simulationRunning;
        m_runSimulationButton->setEnabled(viewModel.canRunSimulation && !simulationLocked);
        m_exportSimulationButton->setEnabled(viewModel.canExportSimulation && !simulationLocked);
        for(QDoubleSpinBox* spin : { m_plateSideSpinBox, m_plateCellSpinBox,
            m_simulationDistanceSpinBox, m_trajectoryOverrunSpinBox,
            m_simulationIncidenceSpinBox,
            m_simulationAzimuthSpinBox, m_simulationToolRollSpinBox,
            m_pointDurationSpinBox, m_scanSpeedSpinBox,
            m_entrySpeedSpinBox, m_exitSpeedSpinBox, m_trajectoryPointIntervalSpinBox,
            m_scanStartXSpinBox,
            m_scanStartYSpinBox, m_scanEndXSpinBox, m_scanEndYSpinBox }) {
            spin->setEnabled(simulationActive && !simulationLocked);
        }
        m_scanPassCountSpinBox->setEnabled(simulationActive && !simulationLocked);
        m_simulationKindCombo->setEnabled(simulationActive && !simulationLocked);
        const bool reproductionLocked = viewModel.predictionRunning
            || viewModel.reproductionRunning;
        m_reproductionControlsLocked = reproductionLocked;
        const bool currentReproduction = reproductionAlgorithm()
            == spraythickness::ReproductionAlgorithmKind::CurrentMethod;
        m_reproductionAlgorithmCombo->setEnabled(!reproductionLocked);
        m_reproductionConfigurationEdit->setEnabled(
            !reproductionLocked && !currentReproduction);
        m_selectReproductionConfigurationButton->setEnabled(
            !reproductionLocked && !currentReproduction);
        m_createReproductionTemplateButton->setEnabled(
            !reproductionLocked && !currentReproduction);
        for(QDoubleSpinBox* spin : {
            m_tzinavaRotationOriginXSpinBox,
            m_tzinavaRotationOriginYSpinBox,
            m_tzinavaRotationOriginZSpinBox,
            m_tzinavaRotationAxisXSpinBox,
            m_tzinavaRotationAxisYSpinBox,
            m_tzinavaRotationAxisZSpinBox,
            m_tzinavaAngularSpeedSpinBox }) {
            spin->setEnabled(!reproductionLocked);
        }
        const bool reproductionConfigurationReady = currentReproduction
            || !m_reproductionConfigurationEdit->text().trimmed().isEmpty();
        const bool reproductionRuntimeReady = reproductionAlgorithm()
                != spraythickness::ReproductionAlgorithmKind::Tzinava2020
            || reproductionRuntimeInputs().objectRotationAxis.squaredNorm()
                > 1.0e-12;
        m_runReproductionButton->setEnabled(
            viewModel.canRunReproduction && !reproductionLocked
            && reproductionConfigurationReady && reproductionRuntimeReady);
        m_cancelReproductionButton->setVisible(viewModel.reproductionRunning);
        m_reproductionProgressBar->setVisible(viewModel.reproductionRunning);
        m_reproductionProgressBar->setValue(
            static_cast<int>(viewModel.progress * 1000.0));
        m_exportReproductionButton->setEnabled(
            viewModel.canExportReproduction && !reproductionLocked);
        m_reproductionStatusLabel->setText(coatingAnalysisTranslate(
            m_languageCode, viewModel.reproductionDetails));
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
        const bool standardControlsEnabled = !viewModel.predictionRunning
            && !viewModel.simulationActive;
        m_openModelButton->setEnabled(standardControlsEnabled);
        m_openTrajectoryButton->setEnabled(standardControlsEnabled);
        m_selectModelButton->setEnabled(standardControlsEnabled);
        m_selectTrajectoryButton->setEnabled(standardControlsEnabled);
        const bool fixedReproductionTrajectory = reproductionActive
            && reproductionForcesOriginalTrajectoryPoints();
        m_trajectorySamplingCombo->setEnabled(
            standardControlsEnabled && !fixedReproductionTrajectory);
        m_timeStepSpinBox->setEnabled(
            standardControlsEnabled
            && !fixedReproductionTrajectory
            && trajectorySamplingMode()
                == spraythickness::TrajectorySamplingMode::ResampleByTimeStep);
        updateTrajectorySamplingUi();
        m_algorithmCombo->setEnabled(standardControlsEnabled);
        m_sprayDirectionCombo->setEnabled(standardControlsEnabled);
        m_powderFeedDirectionCombo->setEnabled(standardControlsEnabled);
        m_bvhCheckBox->setEnabled(standardControlsEnabled);
        m_historyCheckBox->setEnabled(standardControlsEnabled);
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
        m_rotationAxisStatusLabel->setText(coatingAnalysisTranslate(m_languageCode,
            QStringLiteral("Rotation axis: %1").arg(viewModel.rotationAxisSource)));
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
        m_previewLocalInputsButton->setText(coatingAnalysisTranslate(m_languageCode,
            viewModel.hasLocalPreview
                ? QStringLiteral("Refresh local input preview")
                : QStringLiteral("Preview local prediction inputs")));
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
        m_statusLabel->setText(coatingAnalysisTranslate(m_languageCode, viewModel.status));
        m_setReferenceButton->setVisible(true);
        m_clearReferenceButton->setVisible(true);
        m_checkReferenceButton->setVisible(true);
        m_setReferenceButton->setEnabled(
            viewModel.canSetReference && !viewModel.predictionRunning);
        m_clearReferenceButton->setEnabled(
            viewModel.canClearReference && !viewModel.predictionRunning);
        m_checkReferenceButton->setEnabled(
            viewModel.canCheckReference && !viewModel.predictionRunning);
        m_referenceStatusLabel->setText(
            coatingAnalysisTranslate(m_languageCode, viewModel.referenceStatus));
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

    SimulationExperimentParameters CoatingAnalysisPanel::simulationParameters() const
    {
        SimulationExperimentParameters parameters;
        parameters.kind = static_cast<SimulationExperimentKind>(
            m_simulationKindCombo->currentData().toInt());
        parameters.plateSideMillimeters = m_plateSideSpinBox->value();
        parameters.cellSizeMillimeters = m_plateCellSpinBox->value();
        parameters.sprayDistanceMillimeters = m_simulationDistanceSpinBox->value();
        parameters.trajectoryOverrunMillimeters = m_trajectoryOverrunSpinBox->value();
        parameters.incidenceAngleDegrees = m_simulationIncidenceSpinBox->value();
        parameters.azimuthDegrees = m_simulationAzimuthSpinBox->value();
        parameters.toolRollDegrees = m_simulationToolRollSpinBox->value();
        parameters.pointDurationSeconds = m_pointDurationSpinBox->value();
        parameters.scanSpeedMillimetersPerSecond = m_scanSpeedSpinBox->value();
        parameters.scanPassCount = m_scanPassCountSpinBox->value();
        parameters.entrySpeedMillimetersPerSecond = m_entrySpeedSpinBox->value();
        parameters.exitSpeedMillimetersPerSecond = m_exitSpeedSpinBox->value();
        parameters.trajectoryPointIntervalSeconds = m_trajectoryPointIntervalSpinBox->value();
        parameters.scanStartXMillimeters = m_scanStartXSpinBox->value();
        parameters.scanStartYMillimeters = m_scanStartYSpinBox->value();
        parameters.scanEndXMillimeters = m_scanEndXSpinBox->value();
        parameters.scanEndYMillimeters = m_scanEndYSpinBox->value();
        return parameters;
    }

    bool CoatingAnalysisPanel::simulationActive() const
    {
        return m_mode == CoatingAnalysisMode::Simulation;
    }

    CoatingAnalysisMode CoatingAnalysisPanel::mode() const
    {
        return m_mode;
    }

    spraythickness::ReproductionAlgorithmKind
    CoatingAnalysisPanel::reproductionAlgorithm() const
    {
        return static_cast<spraythickness::ReproductionAlgorithmKind>(
            m_reproductionAlgorithmCombo->currentData().toInt());
    }

    Eigen::Vector3d CoatingAnalysisPanel::sprayDirectionLocal() const
    {
        return localAxisVector(m_sprayDirectionCombo->currentData().toInt());
    }

    Eigen::Vector3d CoatingAnalysisPanel::powderFeedDirectionLocal() const
    {
        return localAxisVector(m_powderFeedDirectionCombo->currentData().toInt());
    }

    void CoatingAnalysisPanel::ensurePowderFeedDirectionValid()
    {
        if(std::abs(sprayDirectionLocal().dot(powderFeedDirectionLocal())) < 0.5) {
            return;
        }

        const int fallbackAxis = std::abs(sprayDirectionLocal().y()) < 0.5
            ? PositiveY : PositiveX;
        const QSignalBlocker blocker(m_powderFeedDirectionCombo);
        m_powderFeedDirectionCombo->setCurrentIndex(
            m_powderFeedDirectionCombo->findData(fallbackAxis));
    }

    PublishedReproductionRuntimeInputs
    CoatingAnalysisPanel::reproductionRuntimeInputs() const
    {
        PublishedReproductionRuntimeInputs inputs;
        inputs.objectRotationOriginMeters = 0.001 * Eigen::Vector3d(
            m_tzinavaRotationOriginXSpinBox->value(),
            m_tzinavaRotationOriginYSpinBox->value(),
            m_tzinavaRotationOriginZSpinBox->value());
        inputs.objectRotationAxis = Eigen::Vector3d(
            m_tzinavaRotationAxisXSpinBox->value(),
            m_tzinavaRotationAxisYSpinBox->value(),
            m_tzinavaRotationAxisZSpinBox->value());
        inputs.objectAngularSpeedRadiansPerSecond =
            qDegreesToRadians(m_tzinavaAngularSpeedSpinBox->value());
        return inputs;
    }

    bool CoatingAnalysisPanel::reproductionForcesOriginalTrajectoryPoints() const
    {
        return PublishedReproductionAdapter::inputProfile(
            reproductionAlgorithm()).forceOriginalTrajectoryPoints;
    }

    void CoatingAnalysisPanel::updateReproductionInputUi()
    {
        if(m_reproductionAlgorithmCombo == nullptr) {
            return;
        }
        const PublishedReproductionInputProfile profile =
            PublishedReproductionAdapter::inputProfile(reproductionAlgorithm());
        const auto translate = [this](const std::string& value) {
            return coatingAnalysisTranslate(
                m_languageCode, QString::fromStdString(value));
        };
        if(m_reproductionModelInputLabel != nullptr) {
            m_reproductionModelInputLabel->setText(
                translate(profile.modelInputName));
        }
        if(m_reproductionTrajectoryInputLabel != nullptr) {
            m_reproductionTrajectoryInputLabel->setText(
                translate(profile.trajectoryInputName));
        }
        if(m_tzinavaRotationLabel != nullptr) {
            m_tzinavaRotationLabel->setVisible(profile.hasObjectRotationInput);
        }
        if(m_tzinavaRotationWidget != nullptr) {
            m_tzinavaRotationWidget->setVisible(profile.hasObjectRotationInput);
        }
        if(m_selectModelButton != nullptr) {
            QString text = QStringLiteral("Select Model File");
            if(profile.requiresStlSurface) {
                text = QStringLiteral("Select STL Surface");
            } else if(reproductionAlgorithm()
                == spraythickness::ReproductionAlgorithmKind::Tanaka2024) {
                text = QStringLiteral("Select Point Source");
            } else if(reproductionAlgorithm()
                == spraythickness::ReproductionAlgorithmKind::Wu2020) {
                text = QStringLiteral("Select Substrate Model");
            } else if(reproductionAlgorithm()
                == spraythickness::ReproductionAlgorithmKind::Fuke2005) {
                text = QStringLiteral("Select Surface Mesh");
            }
            m_selectModelButton->setText(
                coatingAnalysisTranslate(m_languageCode, text));
            m_selectModelButton->setToolTip(translate(profile.modelDialogTitle));
        }
        if(m_selectTrajectoryButton != nullptr) {
            const QString text = reproductionAlgorithm()
                    == spraythickness::ReproductionAlgorithmKind::Fuke2005
                ? QStringLiteral("Select Pose Sequence")
                : QStringLiteral("Select Trajectory File");
            m_selectTrajectoryButton->setText(
                coatingAnalysisTranslate(m_languageCode, text));
            m_selectTrajectoryButton->setToolTip(
                translate(profile.trajectoryDialogTitle));
        }
        const bool publishedMethod = reproductionAlgorithm()
            != spraythickness::ReproductionAlgorithmKind::CurrentMethod;
        if(m_openModelButton != nullptr) {
            m_openModelButton->setVisible(!publishedMethod
                || m_mode != CoatingAnalysisMode::Reproduction);
        }
        if(m_openTrajectoryButton != nullptr) {
            m_openTrajectoryButton->setVisible(!publishedMethod
                || m_mode != CoatingAnalysisMode::Reproduction);
        }
        if(profile.forceOriginalTrajectoryPoints
            && m_trajectorySamplingCombo != nullptr) {
            const QSignalBlocker blocker(m_trajectorySamplingCombo);
            m_trajectorySamplingCombo->setCurrentIndex(
                m_trajectorySamplingCombo->findData(static_cast<int>(
                    spraythickness::TrajectorySamplingMode::OriginalPoints)));
        }
        updateTrajectorySamplingUi();
    }

    void CoatingAnalysisPanel::updateTrajectorySamplingUi()
    {
        if(m_trajectorySamplingCombo == nullptr) {
            return;
        }
        const bool visible = trajectorySamplingMode()
            == spraythickness::TrajectorySamplingMode::ResampleByTimeStep;
        if(m_timeStepLabel != nullptr) {
            m_timeStepLabel->setVisible(visible);
        }
        if(m_timeStepSpinBox != nullptr) {
            m_timeStepSpinBox->setVisible(visible);
        }
    }

    QString CoatingAnalysisPanel::reproductionConfigurationPath() const
    {
        return m_reproductionConfigurationEdit->text();
    }

    void CoatingAnalysisPanel::setReproductionConfigurationPath(
        const QString& path)
    {
        m_reproductionConfigurationEdit->setText(path);
        const int algorithmKey = static_cast<int>(reproductionAlgorithm());
        m_reproductionConfigurationPaths.insert(algorithmKey, path);
        m_reproductionConfigurationAlgorithm = algorithmKey;
        emit reproductionInputsChanged();
    }

    void CoatingAnalysisPanel::setPeriodicLocalPredictionEnabled(bool enabled)
    {
        if(m_predictionModeCombo != nullptr) {
            const auto mode = enabled ? PredictionInputMode::LocalAllSprayPoints
                : PredictionInputMode::CompleteSpatialFilteredSprayPoints;
            m_predictionModeCombo->setCurrentIndex(
                m_predictionModeCombo->findData(static_cast<int>(mode)));
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
