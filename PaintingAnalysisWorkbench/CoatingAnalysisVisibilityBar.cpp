#include "CoatingAnalysisVisibilityBar.h"
#include "CoatingAnalysisLanguage.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    CoatingAnalysisVisibilityBar::CoatingAnalysisVisibilityBar(QWidget* parent)
        : QWidget(parent)
    {
        auto* outerLayout = new QVBoxLayout(this);
        outerLayout->setContentsMargins(0, 0, 0, 0);
        outerLayout->setSpacing(4);
        auto* layout = new QHBoxLayout();
        outerLayout->addLayout(layout);
        layout->setContentsMargins(8, 4, 8, 4);
        layout->setSpacing(12);

        auto* title = new QLabel(QStringLiteral("Visualize"), this);
        title->setStyleSheet(QStringLiteral("font-weight: bold;"));
        layout->addWidget(title);

        m_showModel = new QCheckBox(QStringLiteral("Model"), this);
        m_showTrajectory = new QCheckBox(QStringLiteral("Trajectory"), this);
        m_showSprayPoints = new QCheckBox(QStringLiteral("Spray Points"), this);
        m_showThickness = new QCheckBox(QStringLiteral("Thickness Cloud"), this);
        m_thicknessPick = new QCheckBox(QStringLiteral("Thickness Pick"), this);

        layout->addWidget(m_showModel);
        layout->addWidget(m_showTrajectory);
        layout->addWidget(m_showSprayPoints);
        layout->addWidget(m_showThickness);
        layout->addWidget(m_thicknessPick);
        layout->addStretch(1);

        m_exportThicknessPickReport = new QPushButton(
            QStringLiteral("Export Picked Thickness Report (YZ, x=0)"), this);
        m_exportThicknessPickReport->setObjectName(
            QStringLiteral("exportThicknessPickReport"));
        m_exportThicknessPickReport->setToolTip(QStringLiteral(
            "Select the target contour in world YZ (x=0). Keep its zero thickness points; "
            "export Excel with an adjustable equally spaced thickness diagram."));
        outerLayout->addWidget(m_exportThicknessPickReport);
        m_thicknessPickReportLink = new QLabel(this);
        m_thicknessPickReportLink->setObjectName(
            QStringLiteral("thicknessPickReportLink"));
        m_thicknessPickReportLink->setOpenExternalLinks(true);
        m_thicknessPickReportLink->hide();
        outerLayout->addWidget(m_thicknessPickReportLink);
        connect(m_exportThicknessPickReport, &QPushButton::clicked,
            this, &CoatingAnalysisVisibilityBar::thicknessPickReportRequested);

        connect(m_showModel, &QCheckBox::toggled, this, &CoatingAnalysisVisibilityBar::showModelChanged);
        connect(m_showTrajectory, &QCheckBox::toggled,
            this, &CoatingAnalysisVisibilityBar::showTrajectoryChanged);
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
        m_exportThicknessPickReport->setEnabled(view.canExportThicknessPickReport);
        m_exportThicknessPickReport->setText(coatingAnalysisTranslate(m_languageCode,
            QStringLiteral("Export Picked Thickness Report (YZ, x=0)")));
        m_exportThicknessPickReport->setToolTip(coatingAnalysisTranslate(m_languageCode,
            view.canExportThicknessPickReport
                ? QStringLiteral("Select the target contour in world YZ (x=0). Keep its zero thickness points; "
                    "export Excel with an adjustable equally spaced thickness diagram.")
                : QStringLiteral("Available after thickness prediction completes; "
                    "online GPU-only fields cannot be exported.")));
        m_thicknessPickReportLink->setVisible(!view.thicknessPickReportPath.isEmpty());
        m_thicknessPickReportLink->setToolTip(view.thicknessPickReportPath);
        m_thicknessPickReportLink->setText(QStringLiteral("<a href=\"%1\">%2</a>")
            .arg(QUrl::fromLocalFile(view.thicknessPickReportPath).toString(QUrl::FullyEncoded).toHtmlEscaped(),
                coatingAnalysisTranslate(m_languageCode,
                    QStringLiteral("Open Picked Thickness Report")).toHtmlEscaped()));
        m_showModel->setEnabled(view.hasModel);
        m_showTrajectory->setEnabled(view.hasTrajectory);
        m_showSprayPoints->setEnabled(view.hasTrajectory);
        m_showThickness->setEnabled(view.hasThickness);
        m_thicknessPick->setEnabled(view.hasThickness && view.showThickness);
        {
            const QSignalBlocker modelBlocker(m_showModel);
            const QSignalBlocker trajectoryBlocker(m_showTrajectory);
            const QSignalBlocker sprayBlocker(m_showSprayPoints);
            const QSignalBlocker thicknessBlocker(m_showThickness);
            const QSignalBlocker pickBlocker(m_thicknessPick);
            m_showModel->setChecked(view.showModel);
            m_showTrajectory->setChecked(view.showTrajectory);
            m_showSprayPoints->setChecked(view.showSprayPoints);
            m_showThickness->setChecked(view.showThickness);
            m_thicknessPick->setChecked(view.thicknessPickEnabled);
        }
    }
}
