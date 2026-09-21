#pragma once

#include "CoatingAnalysisViewModel.h"

#include <QWidget>

class QLabel;
class QToolButton;
class QVBoxLayout;

namespace robot_qt_viewer
{
    // Read-only readings panel under the analysis tree: model / trajectory /
    // deposition / thermal-history / thickness statistics. Values are shown in a
    // monospace face so they read like instrument readouts.
    class CoatingAnalysisInfoPanel : public QWidget
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisInfoPanel(QWidget* parent = nullptr);
        void applyInfo(const CoatingAnalysisInfoView& view);
        void setLanguageCode(const QString& languageCode);

    private:
        QToolButton* addSection(
            QVBoxLayout* layout,
            const QString& title,
            QLabel*& readout,
            QWidget*& content);
        void setSectionTitle(QToolButton* section, const QString& title);

        QToolButton* m_modelSection = nullptr;
        QToolButton* m_trajectorySection = nullptr;
        QToolButton* m_validationSection = nullptr;
        QToolButton* m_simulationSection = nullptr;
        QWidget* m_modelContent = nullptr;
        QWidget* m_trajectoryContent = nullptr;
        QWidget* m_validationContent = nullptr;
        QWidget* m_simulationContent = nullptr;
        QLabel* m_modelReadout = nullptr;
        QLabel* m_trajectoryReadout = nullptr;
        QLabel* m_depositionReadout = nullptr;
        QLabel* m_historyReadout = nullptr;
        QLabel* m_thicknessReadout = nullptr;
        QLabel* m_analysisReadout = nullptr;
        QLabel* m_computationReadout = nullptr;
        QLabel* m_validationReadout = nullptr;
        QLabel* m_simulationReadout = nullptr;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
