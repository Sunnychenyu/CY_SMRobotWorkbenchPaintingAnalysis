#include "CoatingAnalysisPanel.h"

#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    CoatingAnalysisPanel::CoatingAnalysisPanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* rootLayout = new QVBoxLayout(this);

        auto* modelGroup = new QGroupBox(QStringLiteral("Model"), this);
        auto* modelLayout = new QVBoxLayout(modelGroup);
        m_modelNameLabel = new QLabel(modelGroup);
        m_modelPathLabel = new QLabel(modelGroup);
        m_modelPathLabel->setWordWrap(true);
        m_modelPathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        m_openModelButton = new QPushButton(QStringLiteral("Open Model"), modelGroup);
        modelLayout->addWidget(m_modelNameLabel);
        modelLayout->addWidget(m_modelPathLabel);
        modelLayout->addWidget(m_openModelButton, 0, Qt::AlignHCenter);
        rootLayout->addWidget(modelGroup);

        auto* predictionGroup = new QGroupBox(QStringLiteral("Prediction"), this);
        auto* predictionLayout = new QVBoxLayout(predictionGroup);
        m_predictionButton = new QPushButton(QStringLiteral("Thickness Prediction"), predictionGroup);
        m_statusLabel = new QLabel(predictionGroup);
        m_statusLabel->setWordWrap(true);
        predictionLayout->addWidget(m_predictionButton, 0, Qt::AlignHCenter);
        predictionLayout->addWidget(m_statusLabel);
        rootLayout->addWidget(predictionGroup);

        auto* thicknessGroup = new QGroupBox(QStringLiteral("Thickness"), this);
        auto* thicknessLayout = new QVBoxLayout(thicknessGroup);
        m_showThicknessCheckBox = new QCheckBox(QStringLiteral("Show Thickness"), thicknessGroup);
        m_statisticsLabel = new QLabel(thicknessGroup);
        m_statisticsLabel->setWordWrap(true);
        m_currentLabel = new QLabel(QStringLiteral("Current: --"), thicknessGroup);
        thicknessLayout->addWidget(m_showThicknessCheckBox);
        thicknessLayout->addWidget(m_statisticsLabel);
        thicknessLayout->addWidget(m_currentLabel);
        rootLayout->addWidget(thicknessGroup);
        rootLayout->addStretch(1);

        connect(m_openModelButton, &QPushButton::clicked, this, &CoatingAnalysisPanel::openModelRequested);
        connect(m_predictionButton, &QPushButton::clicked, this, &CoatingAnalysisPanel::predictionRequested);
        connect(m_showThicknessCheckBox, &QCheckBox::toggled,
            this, &CoatingAnalysisPanel::showThicknessChanged);

        applyViewModel(CoatingAnalysisViewModel());
    }

    void CoatingAnalysisPanel::applyViewModel(const CoatingAnalysisViewModel& viewModel)
    {
        m_modelNameLabel->setText(viewModel.modelName);
        m_modelPathLabel->setText(viewModel.modelPath);
        m_modelPathLabel->setVisible(!viewModel.modelPath.isEmpty());
        m_predictionButton->setEnabled(viewModel.hasModel);
        m_statusLabel->setText(viewModel.status);
        m_showThicknessCheckBox->setEnabled(viewModel.hasResult);
        {
            const QSignalBlocker blocker(m_showThicknessCheckBox);
            m_showThicknessCheckBox->setChecked(viewModel.showThickness);
        }
        m_statisticsLabel->setVisible(viewModel.hasResult);
        m_statisticsLabel->setText(QStringLiteral("Min: %1 um\nMax: %2 um\nAverage: %3 um")
            .arg(viewModel.minimumMicrometers, 0, 'f', 1)
            .arg(viewModel.maximumMicrometers, 0, 'f', 1)
            .arg(viewModel.averageMicrometers, 0, 'f', 1));
        m_currentLabel->setVisible(viewModel.hasResult);
        m_currentLabel->setText(viewModel.hasCurrentThickness
            ? QStringLiteral("Current: %1 um").arg(viewModel.currentMicrometers, 0, 'f', 2)
            : QStringLiteral("Current: --"));
    }
}
