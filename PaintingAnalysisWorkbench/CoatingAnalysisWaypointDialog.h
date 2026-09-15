#pragma once

#include <SprayTrajectoryCore/SprayTrajectory.h>

#include <QDialog>

class QLabel;

namespace robot_qt_viewer
{
    class CoatingAnalysisWaypointDialog : public QDialog
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisWaypointDialog(QWidget* parent = nullptr);
        void setLanguageCode(const QString& languageCode);

        void setWaypoint(
            const spraytrajectory::SprayPathPoint& point,
            std::size_t index,
            double durationToNextSeconds);

    private:
        QLabel* m_textLabel = nullptr;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
