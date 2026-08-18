#pragma once

#include <SprayCore/SprayModel.h>

#include <QWidget>

namespace robot_qt_viewer
{
    // Signature element of the coating analysis workbench: a live preview of the
    // selected deposition model's lateral profile at reference distance. The curve
    // is informative only; it never feeds back into the prediction kernel.
    class DepositionCurveWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit DepositionCurveWidget(QWidget* parent = nullptr);

        // Plots the Paper Gaussian lateral cross-section derived from its angular
        // sigma at the reference distance.
        void showPaperGaussian();

        // Plots a generic SprayPattern (Uniform / Linear / Gaussian).
        void showSprayPattern(const spraycore::SprayPattern& pattern);

        QSize sizeHint() const override;
        QSize minimumSizeHint() const override;

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        double evaluate(double lateralMeters) const;

        bool m_usePaperGaussian = true;
        spraycore::SprayPattern m_pattern;
        double m_halfWidth = 0.01; // meters, symmetric plot extent
    };
}
