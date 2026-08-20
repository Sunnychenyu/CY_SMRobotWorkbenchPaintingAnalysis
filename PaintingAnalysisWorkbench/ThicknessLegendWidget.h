#pragma once

#include <QWidget>

namespace robot_qt_viewer
{
    class ThicknessLegendWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit ThicknessLegendWidget(QWidget* parent = nullptr);
        void setRange(double minimumMicrometers, double maximumMicrometers);
        void setRelativeErrorMode(bool enabled);

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        double m_minimumMicrometers = 0.0;
        double m_maximumMicrometers = 0.0;
        bool m_relativeErrorMode = false;
    };
}
