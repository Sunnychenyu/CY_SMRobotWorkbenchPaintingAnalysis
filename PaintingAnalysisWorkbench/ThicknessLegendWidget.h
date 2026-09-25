#pragma once

#include <QWidget>
#include <QPointer>

class QDialog;
class QMouseEvent;

namespace robot_qt_viewer
{
    class ThicknessLegendWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit ThicknessLegendWidget(QWidget* parent = nullptr);
        void setRange(double minimumMicrometers, double maximumMicrometers);
        void setRelativeErrorMode(bool enabled);
        void setEditingEnabled(bool enabled);
        void setLanguageCode(const QString& languageCode);

    signals:
        void rangeChangeRequested(
            double minimumMicrometers,
            double maximumMicrometers);
        void automaticRangeRequested();

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mouseDoubleClickEvent(QMouseEvent* event) override;

    private:
        void showRangeEditor();

        double m_minimumMicrometers = 0.0;
        double m_maximumMicrometers = 0.0;
        bool m_relativeErrorMode = false;
        bool m_editingEnabled = false;
        QString m_languageCode{ QStringLiteral("en") };
        QPointer<QDialog> m_rangeEditor;
    };
}
