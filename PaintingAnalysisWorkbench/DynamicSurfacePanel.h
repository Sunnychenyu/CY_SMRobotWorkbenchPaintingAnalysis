#pragma once
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <QWidget>
#include <QColor>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QLabel;

namespace robot_qt_viewer
{
    class DynamicSurfaceHeader final : public QWidget
    {
        Q_OBJECT
    public:
        explicit DynamicSurfaceHeader(QWidget* viewport);
        void setSummary(const QString& model,int view);
    signals:
        void viewChanged(int view);
    protected:
        bool eventFilter(QObject* object,QEvent* event) override;
    private:
        QLabel* m_title;
        QComboBox* m_view;
    };

    struct DynamicSurfaceDisplay
    {
        int view = 0; // evolved / substrate / overlay
        int primitive = 0; // surface / wire / points
        int coloring = 0; // thickness / displacement / constant
        double opacity = 1.0;
        double lineWidth = 1.0;
        double pointSize = 5.0;
        double substrateOpacity = 0.25;
        QColor color{66,150,220};
    };

    class DynamicSurfacePanel final : public QWidget
    {
        Q_OBJECT
    public:
        explicit DynamicSurfacePanel(QWidget* parent = nullptr);
        spraythickness::DynamicSurfaceOptions options() const;
        DynamicSurfaceDisplay display() const;
        void setDisplay(const DynamicSurfaceDisplay& display);
        bool enabled() const;
        void setRunning(bool running);
        void setResult(const spraythickness::DynamicSurfaceSnapshot* result);
        void setView(int view);
    signals:
        void enabledChanged(bool enabled);
        void displayChanged();
        void saveRequested();
        void exportRequested();
    protected:
        bool eventFilter(QObject* object,QEvent* event) override;
    private:
        void refresh();
        QCheckBox* m_enabled;
        QComboBox* m_accuracy;
        QDoubleSpinBox *m_growth,*m_normal,*m_interval;
        QWidget *m_evolution,*m_custom,*m_display;
        QComboBox *m_view,*m_primitive,*m_coloring;
        QDoubleSpinBox *m_opacity,*m_lineWidth,*m_pointSize,*m_substrateOpacity;
        QPushButton *m_color,*m_save,*m_export;
        QLabel* m_status;
        QColor m_selectedColor{66,150,220};
        bool m_hasResult=false;
    };
}
