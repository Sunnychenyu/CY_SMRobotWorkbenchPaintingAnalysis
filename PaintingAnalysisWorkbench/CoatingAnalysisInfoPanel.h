#pragma once

#include "CoatingAnalysisViewModel.h"

#include <QWidget>

class QLabel;
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
        QLabel* addSection(QVBoxLayout* layout, const QString& title);
        QLabel* addReadout(QVBoxLayout* layout);

        QLabel* m_modelSection = nullptr;
        QLabel* m_trajectorySection = nullptr;
        QLabel* m_validationSection = nullptr;
        QLabel* m_simulationSection = nullptr;
        QLabel* m_modelReadout = nullptr;
        QLabel* m_trajectoryReadout = nullptr;
        QLabel* m_depositionReadout = nullptr;
        QLabel* m_historyReadout = nullptr;
        QLabel* m_thicknessReadout = nullptr;
        QLabel* m_computationReadout = nullptr;
        QLabel* m_validationReadout = nullptr;
        QLabel* m_simulationReadout = nullptr;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
