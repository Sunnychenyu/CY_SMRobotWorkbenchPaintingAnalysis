#include "DynamicSurfacePanel.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QColorDialog>
#include <QEvent>
#include <QSettings>
#include <QSignalBlocker>
#include <algorithm>

namespace robot_qt_viewer
{
    DynamicSurfaceHeader::DynamicSurfaceHeader(QWidget* viewport):QWidget(viewport)
    {
        auto* row=new QHBoxLayout(this);row->setContentsMargins(8,4,8,4);
        m_title=new QLabel(this);m_title->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
        m_view=new QComboBox(this);m_view->addItems({QStringLiteral("演化表面"),QStringLiteral("原始基体"),QStringLiteral("叠加对比")});
        m_view->setSizeAdjustPolicy(QComboBox::AdjustToContents);m_view->installEventFilter(this);
        row->addWidget(m_title,1);row->addWidget(m_view);
        setAutoFillBackground(true);setGeometry(8,8,std::max(160,viewport->width()-16),36);
        viewport->installEventFilter(this);hide();
        connect(m_view,QOverload<int>::of(&QComboBox::currentIndexChanged),this,&DynamicSurfaceHeader::viewChanged);
    }
    bool DynamicSurfaceHeader::eventFilter(QObject* o,QEvent* e) {
        if(o==parentWidget()&&e->type()==QEvent::Resize)setGeometry(8,8,std::max(160,parentWidget()->width()-16),36);
        if(o==m_view&&e->type()==QEvent::Wheel){e->ignore();return true;}
        return QWidget::eventFilter(o,e);
    }
    void DynamicSurfaceHeader::setSummary(const QString& model,int view) {
        m_title->setText(model+QStringLiteral(" · 动态表面沉积"));m_title->setToolTip(m_title->text());
        QSignalBlocker block(m_view);m_view->setCurrentIndex(view);raise();
    }

    DynamicSurfacePanel::DynamicSurfacePanel(QWidget* parent):QWidget(parent)
    {
        auto* root=new QVBoxLayout(this); root->setContentsMargins(0,0,0,0); root->setSpacing(4);
        m_enabled=new QCheckBox(QStringLiteral("动态表面沉积"),this);
        m_enabled->setToolTip(QStringLiteral("沿当前表面外法线生长；使用完整模型计算。"));
        root->addWidget(m_enabled);
        auto* evolution=new QGroupBox(QStringLiteral("表面演化"),this); m_evolution=evolution;
        auto* form=new QFormLayout(evolution); form->setContentsMargins(8,6,8,6); form->setSpacing(4);
        auto combo=[this](const QStringList& names) { auto* c=new QComboBox(this); c->addItems(names); c->setSizeAdjustPolicy(QComboBox::AdjustToContents); c->installEventFilter(this); return c; };
        auto number=[this](double min,double max,double value,int decimals,const QString& suffix) {
            auto* s=new QDoubleSpinBox(this); s->setDecimals(decimals); s->setRange(min,max); s->setValue(value); s->setSuffix(suffix); s->setKeyboardTracking(false); s->installEventFilter(this); return s;
        };
        m_accuracy=combo({QStringLiteral("快速"),QStringLiteral("均衡"),QStringLiteral("精细"),QStringLiteral("自定义")});
        QSettings settings;
        m_accuracy->setCurrentIndex(std::clamp(settings.value(QStringLiteral("coating/dynamic/accuracy"),1).toInt(),0,3));
        form->addRow(QStringLiteral("更新精度"),m_accuracy);
        m_custom=new QWidget(this); auto* custom=new QFormLayout(m_custom); custom->setContentsMargins(0,0,0,0); custom->setSpacing(4);
        m_growth=number(0.01,1000,settings.value(QStringLiteral("coating/dynamic/growthUm"),5).toDouble(),2,QStringLiteral(" μm"));
        m_normal=number(0.1,30,settings.value(QStringLiteral("coating/dynamic/normalDeg"),2).toDouble(),1,QStringLiteral(" °"));
        m_interval=number(0.001,10,settings.value(QStringLiteral("coating/dynamic/intervalS"),0.1).toDouble(),3,QStringLiteral(" s"));
        custom->addRow(QStringLiteral("单次最大生长量"),m_growth); custom->addRow(QStringLiteral("最大法向变化"),m_normal); custom->addRow(QStringLiteral("最大更新间隔"),m_interval);
        form->addRow(m_custom); root->addWidget(evolution);
        auto* display=new QGroupBox(QStringLiteral("表面显示"),this); m_display=display;
        auto* visual=new QFormLayout(display); visual->setContentsMargins(8,6,8,6); visual->setSpacing(4);
        m_view=combo({QStringLiteral("演化表面"),QStringLiteral("原始基体"),QStringLiteral("叠加对比")});
        m_primitive=combo({QStringLiteral("曲面"),QStringLiteral("线框"),QStringLiteral("点阵")});
        m_coloring=combo({QStringLiteral("累计沉积厚度"),QStringLiteral("相对基体位移"),QStringLiteral("纯色")});
        m_opacity=number(0,100,100,0,QStringLiteral(" %")); m_opacity->setToolTip(QStringLiteral("网点透明显示，保留正确深度遮挡。"));
        m_lineWidth=number(1,10,1,1,QStringLiteral(" px")); m_pointSize=number(1,30,5,1,QStringLiteral(" px"));
        m_substrateOpacity=number(0,100,25,0,QStringLiteral(" %"));
        m_color=new QPushButton(QStringLiteral("选择颜色"),this);
        visual->addRow(QStringLiteral("视图"),m_view); visual->addRow(QStringLiteral("绘制方式"),m_primitive); visual->addRow(QStringLiteral("着色"),m_coloring);
        visual->addRow(QStringLiteral("不透明度"),m_opacity); visual->addRow(QStringLiteral("线宽"),m_lineWidth); visual->addRow(QStringLiteral("点大小"),m_pointSize);
        visual->addRow(QStringLiteral("基体不透明度"),m_substrateOpacity); visual->addRow(QStringLiteral("表面颜色"),m_color);
        auto* buttons=new QHBoxLayout; m_save=new QPushButton(QStringLiteral("保存结果"),this); m_export=new QPushButton(QStringLiteral("导出表面"),this);
        buttons->addWidget(m_save); buttons->addWidget(m_export); visual->addRow(buttons);
        m_status=new QLabel(this); m_status->setWordWrap(true); visual->addRow(m_status); root->addWidget(display);
        connect(m_enabled,&QCheckBox::toggled,this,[this](bool on){refresh();emit enabledChanged(on);});
        connect(m_accuracy,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{refresh();QSettings().setValue(QStringLiteral("coating/dynamic/accuracy"),m_accuracy->currentIndex());});
        for(auto* spin:{m_growth,m_normal,m_interval}) connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{
            QSettings s; s.setValue(QStringLiteral("coating/dynamic/growthUm"),m_growth->value()); s.setValue(QStringLiteral("coating/dynamic/normalDeg"),m_normal->value()); s.setValue(QStringLiteral("coating/dynamic/intervalS"),m_interval->value());
        });
        for(auto* c:{m_view,m_primitive,m_coloring}) connect(c,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{refresh();emit displayChanged();});
        for(auto* spin:{m_opacity,m_lineWidth,m_pointSize,m_substrateOpacity}) connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this]{emit displayChanged();});
        connect(m_color,&QPushButton::clicked,this,[this]{auto c=QColorDialog::getColor(m_selectedColor,this,QStringLiteral("表面颜色"));if(c.isValid()){m_selectedColor=c;emit displayChanged();}});
        connect(m_save,&QPushButton::clicked,this,&DynamicSurfacePanel::saveRequested); connect(m_export,&QPushButton::clicked,this,&DynamicSurfacePanel::exportRequested);
        refresh();
    }
    bool DynamicSurfacePanel::eventFilter(QObject* o,QEvent* e) { if(e->type()==QEvent::Wheel){e->ignore();return true;} return QWidget::eventFilter(o,e); }
    bool DynamicSurfacePanel::enabled() const {return m_enabled->isChecked();}
    void DynamicSurfacePanel::refresh() {
        m_evolution->setVisible(enabled()); m_custom->setVisible(m_accuracy->currentIndex()==3); m_display->setVisible(enabled()||m_hasResult);
        m_lineWidth->setEnabled(m_primitive->currentIndex()==1); m_pointSize->setEnabled(m_primitive->currentIndex()==2);
        m_substrateOpacity->setEnabled(m_view->currentIndex()==2); m_color->setEnabled(m_coloring->currentIndex()==2);
        m_save->setEnabled(m_hasResult); m_export->setEnabled(m_hasResult);
    }
    void DynamicSurfacePanel::setRunning(bool running) {m_enabled->setEnabled(!running);m_evolution->setEnabled(!running);m_save->setEnabled(!running&&m_hasResult);m_export->setEnabled(!running&&m_hasResult);}
    void DynamicSurfacePanel::setResult(const spraythickness::DynamicSurfaceSnapshot* s) {
        m_hasResult=s!=nullptr;
        m_status->setText(s?QStringLiteral("模拟 %1 s · 耗时 %2 s · 更新 %3 次").arg(s->simulationSeconds,0,'f',2).arg(s->elapsedSeconds,0,'f',1).arg(static_cast<qulonglong>(s->acceptedSteps)):QString());refresh();
    }
    void DynamicSurfacePanel::setView(int view){m_view->setCurrentIndex(view);}
    spraythickness::DynamicSurfaceOptions DynamicSurfacePanel::options() const {
        spraythickness::DynamicSurfaceOptions o;o.enabled=enabled();
        switch(m_accuracy->currentIndex()) {
        case 0:o.maximumGrowthMeters=20e-6;o.maximumNormalChangeDegrees=5;o.maximumIntervalSeconds=0.25;break;
        case 2:o.maximumGrowthMeters=1e-6;o.maximumNormalChangeDegrees=0.5;o.maximumIntervalSeconds=0.025;break;
        case 3:o.maximumGrowthMeters=m_growth->value()*1e-6;o.maximumNormalChangeDegrees=m_normal->value();o.maximumIntervalSeconds=m_interval->value();break;
        default:break;
        }return o;
    }
    void DynamicSurfacePanel::setDisplay(const DynamicSurfaceDisplay& d)
    {
        const QSignalBlocker view(m_view), primitive(m_primitive), coloring(m_coloring),
            opacity(m_opacity), width(m_lineWidth), size(m_pointSize), substrate(m_substrateOpacity);
        m_view->setCurrentIndex(d.view);m_primitive->setCurrentIndex(d.primitive);m_coloring->setCurrentIndex(d.coloring);
        m_opacity->setValue(d.opacity*100);m_lineWidth->setValue(d.lineWidth);m_pointSize->setValue(d.pointSize);
        m_substrateOpacity->setValue(d.substrateOpacity*100);m_selectedColor=d.color;
        refresh();
    }
    DynamicSurfaceDisplay DynamicSurfacePanel::display() const {
        DynamicSurfaceDisplay d;d.view=m_view->currentIndex();d.primitive=m_primitive->currentIndex();d.coloring=m_coloring->currentIndex();d.opacity=m_opacity->value()/100;d.lineWidth=m_lineWidth->value();d.pointSize=m_pointSize->value();d.substrateOpacity=m_substrateOpacity->value()/100;d.color=m_selectedColor;return d;
    }
}
