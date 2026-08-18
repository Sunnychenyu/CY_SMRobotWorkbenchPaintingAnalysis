#include "DepositionCurveWidget.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        double paperGaussianHalfWidthMeters()
        {
            const spraythickness::PaperGaussianParameters parameters;
            const double lateralSigma =
                parameters.referenceDistanceMeters * std::tan(parameters.sigmaPhiRadians);
            return lateralSigma * 2.5;
        }

        double paperGaussianValue(double lateralMeters)
        {
            const spraythickness::PaperGaussianParameters parameters;
            const double phi = std::atan(
                lateralMeters / std::max(parameters.referenceDistanceMeters, 1.0e-9));
            const double sigma = std::max(parameters.sigmaPhiRadians, 1.0e-9);
            return std::exp(-0.5 * (phi / sigma) * (phi / sigma));
        }
    }

    DepositionCurveWidget::DepositionCurveWidget(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumHeight(64);
    }

    void DepositionCurveWidget::showPaperGaussian()
    {
        m_usePaperGaussian = true;
        m_halfWidth = std::max(paperGaussianHalfWidthMeters(), 1.0e-4);
        update();
    }

    void DepositionCurveWidget::showSprayPattern(const spraycore::SprayPattern& pattern)
    {
        m_usePaperGaussian = false;
        m_pattern = pattern;
        m_halfWidth = std::max(pattern.width * 0.5, 1.0e-4);
        update();
    }

    QSize DepositionCurveWidget::sizeHint() const
    {
        return QSize(260, 96);
    }

    QSize DepositionCurveWidget::minimumSizeHint() const
    {
        return QSize(120, 64);
    }

    double DepositionCurveWidget::evaluate(double lateralMeters) const
    {
        if(m_usePaperGaussian) {
            return paperGaussianValue(lateralMeters);
        }
        return m_pattern.evaluate(lateralMeters);
    }

    void DepositionCurveWidget::paintEvent(QPaintEvent* event)
    {
        Q_UNUSED(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRectF plot = rect().adjusted(6, 4, -6, -16);
        if(plot.width() <= 1 || plot.height() <= 1) {
            return;
        }

        // Baseline and lateral extent.
        painter.setPen(QPen(palette().mid(), 1));
        painter.drawLine(QPointF(plot.left(), plot.bottom()),
            QPointF(plot.right(), plot.bottom()));
        painter.setPen(QPen(palette().dark(), 1));
        painter.drawLine(QPointF(plot.left() + plot.width() * 0.5, plot.bottom()),
            QPointF(plot.left() + plot.width() * 0.5, plot.top()));

        const double halfWidth = std::max(m_halfWidth, 1.0e-9);
        const int sampleCount = 160;
        QPainterPath path;
        for(int sample = 0; sample <= sampleCount; ++sample) {
            const double x = -halfWidth + (2.0 * halfWidth) * sample / sampleCount;
            const double y = std::clamp(evaluate(x), 0.0, 1.0);
            const double px = plot.left() + (x + halfWidth) / (2.0 * halfWidth) * plot.width();
            const double py = plot.bottom() - y * plot.height();
            if(sample == 0) {
                path.moveTo(px, py);
            } else {
                path.lineTo(px, py);
            }
        }

        // Fill under the curve with a translucent accent, then stroke the curve.
        QPainterPath fill = path;
        fill.lineTo(plot.right(), plot.bottom());
        fill.lineTo(plot.left(), plot.bottom());
        fill.closeSubpath();
        painter.fillPath(fill, QColor(38, 120, 210, 40));
        painter.setPen(QPen(QColor(38, 120, 210), 2));
        painter.drawPath(path);

        // Spray width boundary markers.
        painter.setPen(QPen(QColor(200, 90, 40, 160), 1, Qt::DashLine));
        const double leftBoundary = plot.left() + (0.0 + halfWidth) / (2.0 * halfWidth) * plot.width();
        const double rightBoundary = plot.left() + (2.0 * halfWidth) / (2.0 * halfWidth) * plot.width();
        painter.drawLine(QPointF(leftBoundary, plot.bottom()),
            QPointF(leftBoundary, plot.top()));
        painter.drawLine(QPointF(rightBoundary, plot.bottom()),
            QPointF(rightBoundary, plot.top()));

        painter.setPen(palette().mid().color());
        painter.drawText(QRectF(plot.left(), plot.bottom() + 2,
            plot.width(), height() - plot.bottom() - 2),
            Qt::AlignHCenter | Qt::AlignTop,
            QStringLiteral("lateral offset (mm)"));
    }
}
