#include "CoatingAnalysisVisibilityBar.h"
#include "CoatingAnalysisLanguage.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>

namespace robot_qt_viewer
{
    CoatingAnalysisVisibilityBar::CoatingAnalysisVisibilityBar(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QHBoxLayout(this);
        layout->setContentsMargins(8, 4, 8, 4);
        layout->setSpacing(12);

        auto* title = new QLabel(QStringLiteral("Visualize"), this);
        title->setStyleSheet(QStringLiteral("font-weight: bold;"));
        layout->addWidget(title);

        m_showModel = new QCheckBox(QStringLiteral("Model"), this);
        m_showSprayPoints = new QCheckBox(QStringLiteral("Spray Points"), this);
        m_showThickness = new QCheckBox(QStringLiteral("Thickness Cloud"), this);
        m_thicknessPick = new QCheckBox(QStringLiteral("Thickness Pick"), this);

        layout->addWidget(m_showModel);
        layout->addWidget(m_showSprayPoints);
        layout->addWidget(m_showThickness);
        layout->addWidget(m_thicknessPick);
        layout->addStretch(1);

        connect(m_showModel, &QCheckBox::toggled, this, &CoatingAnalysisVisibilityBar::showModelChanged);
        connect(m_showSprayPoints, &QCheckBox::toggled,
            this, &CoatingAnalysisVisibilityBar::showSprayPointsChanged);
        connect(m_showThickness, &QCheckBox::toggled,
            this, &CoatingAnalysisVisibilityBar::showThicknessChanged);
        connect(m_thicknessPick, &QCheckBox::toggled,
            this, &CoatingAnalysisVisibilityBar::thicknessPickChanged);
        setLanguageCode(QStringLiteral("en"));
    }

    void CoatingAnalysisVisibilityBar::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        for(QWidget* widget : findChildren<QWidget*>()) {
            if(auto* button = qobject_cast<QAbstractButton*>(widget)) {
                button->setText(coatingAnalysisTranslate(m_languageCode, button->text()));
            } else if(auto* label = qobject_cast<QLabel*>(widget)) {
                label->setText(coatingAnalysisTranslate(m_languageCode, label->text()));
            }
        }
    }

    void CoatingAnalysisVisibilityBar::applyVisibility(const CoatingAnalysisVisibilityView& view)
    {
        m_showModel->setEnabled(view.hasModel);
        m_showSprayPoints->setEnabled(view.hasTrajectory);
        m_showThickness->setEnabled(view.hasThickness);
        m_thicknessPick->setEnabled(view.hasThickness && view.showThickness);
        {
            const QSignalBlocker modelBlocker(m_showModel);
            const QSignalBlocker sprayBlocker(m_showSprayPoints);
            const QSignalBlocker thicknessBlocker(m_showThickness);
            const QSignalBlocker pickBlocker(m_thicknessPick);
            m_showModel->setChecked(view.showModel);
            m_showSprayPoints->setChecked(view.showSprayPoints);
            m_showThickness->setChecked(view.showThickness);
            m_thicknessPick->setChecked(view.thicknessPickEnabled);
        }
    }
}
