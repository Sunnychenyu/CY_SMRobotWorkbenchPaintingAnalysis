#include "CoatingAnalysisWaypointDialog.h"
#include "CoatingAnalysisLanguage.h"

#include <Eigen/Geometry>

#include <QLabel>
#include <QVBoxLayout>

#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kRadToDeg = 57.295779513082320876798154814105;
    }

    CoatingAnalysisWaypointDialog::CoatingAnalysisWaypointDialog(QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(QStringLiteral("Waypoint"));
        setMinimumWidth(360);
        auto* layout = new QVBoxLayout(this);
        m_textLabel = new QLabel(this);
        m_textLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        m_textLabel->setStyleSheet(QStringLiteral("font-family: Consolas, monospace;"));
        layout->addWidget(m_textLabel);
        setLanguageCode(QStringLiteral("en"));
    }

    void CoatingAnalysisWaypointDialog::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        setWindowTitle(coatingAnalysisTranslate(m_languageCode, windowTitle()));
        m_textLabel->setText(coatingAnalysisTranslate(m_languageCode, m_textLabel->text()));
    }

    void CoatingAnalysisWaypointDialog::setWaypoint(
        const spraytrajectory::SprayPathPoint& point,
        std::size_t index,
        double durationToNextSeconds)
    {
        const Eigen::Vector3d translation = point.tcpPose.translation();
        const Eigen::Vector3d rpyDegrees =
            point.tcpPose.linear().eulerAngles(2, 1, 0) * kRadToDeg;

        QString text;
        text += QStringLiteral("Index        : #%1\n").arg(index);
        text += QStringLiteral("Time         : %1 s\n").arg(point.time, 0, 'f', 4);
        if(durationToNextSeconds >= 0.0) {
            text += QStringLiteral("Duration next: %1 s\n")
                .arg(durationToNextSeconds, 0, 'f', 4);
        } else {
            text += QStringLiteral("Duration next: (last waypoint)\n");
        }
        text += QStringLiteral("Position     : %1, %2, %3 mm\n")
            .arg(translation.x() * 1000.0, 0, 'f', 3)
            .arg(translation.y() * 1000.0, 0, 'f', 3)
            .arg(translation.z() * 1000.0, 0, 'f', 3);
        text += QStringLiteral("Orientation  : Yaw %1 deg, Pitch %2 deg, Roll %3 deg\n")
            .arg(rpyDegrees.x(), 0, 'f', 3)
            .arg(rpyDegrees.y(), 0, 'f', 3)
            .arg(rpyDegrees.z(), 0, 'f', 3);
        text += QStringLiteral("Spray        : %1\n")
            .arg(point.sprayEnabled ? QStringLiteral("On") : QStringLiteral("Off"));
        text += QStringLiteral("Process ID   : %1\n")
            .arg(point.processId.empty()
                ? QStringLiteral("(none)")
                : QString::fromStdString(point.processId));
        text += QStringLiteral("Target dist. : %1 m\n")
            .arg(point.targetDistance, 0, 'f', 4);
        text += QStringLiteral("Target normal: %1, %2, %3\n")
            .arg(point.targetNormal.x(), 0, 'f', 3)
            .arg(point.targetNormal.y(), 0, 'f', 3)
            .arg(point.targetNormal.z(), 0, 'f', 3);
        text += QStringLiteral("Region ID    : %1\n")
            .arg(point.workpieceRegionId >= 0
                ? QString::number(point.workpieceRegionId)
                : QStringLiteral("(none)"));
        m_textLabel->setText(coatingAnalysisTranslate(m_languageCode, text));
    }
}
