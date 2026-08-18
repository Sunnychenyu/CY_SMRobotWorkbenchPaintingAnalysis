#pragma once

#include "CoatingAnalysisViewModel.h"

#include <QWidget>

class QCheckBox;

namespace robot_qt_viewer
{
    // Bottom visualization strip: single source for which coating elements are
    // rendered. Every toggle forwards to the module controller, which is the
    // single source of truth for visibility state.
    class CoatingAnalysisVisibilityBar : public QWidget
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisVisibilityBar(QWidget* parent = nullptr);
        void applyVisibility(const CoatingAnalysisVisibilityView& view);

    signals:
        void showModelChanged(bool enabled);
        void showSprayPointsChanged(bool enabled);
        void showThicknessChanged(bool enabled);
        void thicknessPickChanged(bool enabled);

    private:
        QCheckBox* m_showModel = nullptr;
        QCheckBox* m_showSprayPoints = nullptr;
        QCheckBox* m_showThickness = nullptr;
        QCheckBox* m_thicknessPick = nullptr;
    };
}
