#pragma once

#include <QString>

class QWidget;

namespace robot_qt_viewer
{
    class PaintingAnalysisDialogService
    {
    public:
        static QString selectModelFile(
            QWidget* parent,
            const QString& title = QStringLiteral("Open Coating Analysis Model"),
            const QString& filter = QStringLiteral(
                "Mesh Models (*.stl *.obj *.dae *.ply);;All Files (*.*)"));
        static bool selectModelUnitScale(
            QWidget* parent,
            const QString& modelPath,
            double& scaleToMeters);
        static QString selectTrajectoryFile(
            QWidget* parent,
            const QString& title = QStringLiteral("Open Spray Trajectory"));
    };
}
