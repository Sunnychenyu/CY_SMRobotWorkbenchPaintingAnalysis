#pragma once

#include "CoatingAnalysisViewModel.h"

#include <QWidget>

class QCheckBox;
class QLabel;
class QPushButton;

namespace robot_qt_viewer
{
    class CoatingAnalysisPanel : public QWidget
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisPanel(QWidget* parent = nullptr);
        void applyViewModel(const CoatingAnalysisViewModel& viewModel);

    signals:
        void openModelRequested();
        void predictionRequested();
        void showThicknessChanged(bool enabled);

    private:
        QLabel* m_modelNameLabel = nullptr;
        QLabel* m_modelPathLabel = nullptr;
        QPushButton* m_openModelButton = nullptr;
        QPushButton* m_predictionButton = nullptr;
        QLabel* m_statusLabel = nullptr;
        QCheckBox* m_showThicknessCheckBox = nullptr;
        QLabel* m_statisticsLabel = nullptr;
        QLabel* m_currentLabel = nullptr;
    };
}
