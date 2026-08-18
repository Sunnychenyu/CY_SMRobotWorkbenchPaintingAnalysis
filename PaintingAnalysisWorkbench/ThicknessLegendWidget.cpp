#include "ThicknessLegendWidget.h"

#include <VisualizationSDK/ScalarColorMap.h>

#include <QPainter>
#include <QPaintEvent>

namespace robot_qt_viewer
{
    ThicknessLegendWidget::ThicknessLegendWidget(QWidget* parent)
        : QWidget(parent)
    {
        setFixedWidth(118);
        resize(width(), 360);
        setAutoFillBackground(false);
        setAttribute(Qt::WA_OpaquePaintEvent, true);
        setAttribute(Qt::WA_TransparentForMouseEvents, true);
    }

    void ThicknessLegendWidget::setRange(double minimumMicrometers, double maximumMicrometers)
    {
        m_minimumMicrometers = minimumMicrometers;
        m_maximumMicrometers = maximumMicrometers;
        update();
    }

    void ThicknessLegendWidget::paintEvent(QPaintEvent* event)
    {
        QWidget::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QColor background = palette().color(QPalette::Window);
        background.setAlpha(255);
        painter.fillRect(rect(), background);
        painter.setPen(palette().color(QPalette::Mid));
        painter.setBrush(background);
        painter.drawRoundedRect(rect().adjusted(1, 1, -2, -2), 6, 6);

        painter.setPen(palette().color(QPalette::Text));
        painter.drawText(QRect(8, 8, width() - 16, 38),
            Qt::AlignHCenter | Qt::AlignVCenter,
            QStringLiteral("Thick.\n(um)"));

        const QRect barRect(18, 58, 27, qMax(80, height() - 88));
        const smrobot::visualization::ScalarColorMap colorMap =
            smrobot::visualization::ScalarColorMap::heatMap();

        for(int y = barRect.top(); y <= barRect.bottom(); ++y) {
            const double normalized = barRect.height() > 1
                ? static_cast<double>(barRect.bottom() - y) /
                    static_cast<double>(barRect.height() - 1)
                : 0.0;
            const Eigen::Vector3f color = colorMap.sampleNormalized(normalized)
                .cwiseMax(0.0f)
                .cwiseMin(1.0f);
            const QColor displayColor(
                qRound(color.x() * 255.0f),
                qRound(color.y() * 255.0f),
                qRound(color.z() * 255.0f),
                255);
            painter.fillRect(
                QRect(barRect.left(), y, barRect.width(), 1),
                displayColor);
        }
        painter.setBrush(Qt::NoBrush);
        painter.setPen(palette().color(QPalette::Mid));
        painter.drawRect(barRect.adjusted(0, 0, -1, -1));

        painter.setPen(palette().color(QPalette::Text));
        const double thicknessSpan = m_maximumMicrometers - m_minimumMicrometers;
        for(const smrobot::visualization::ScalarColorStop& stop : colorMap.stops()) {
            const int tickY = barRect.bottom() -
                qRound(stop.position * static_cast<double>(barRect.height() - 1));
            const double thickness = m_minimumMicrometers + stop.position * thicknessSpan;
            painter.drawLine(barRect.right() + 2, tickY, barRect.right() + 9, tickY);
            painter.drawText(
                QRect(barRect.right() + 14, tickY - 11,
                    width() - barRect.right() - 18, 22),
                Qt::AlignLeft | Qt::AlignVCenter,
                QString::number(thickness, 'f', 1));
        }
    }
}
