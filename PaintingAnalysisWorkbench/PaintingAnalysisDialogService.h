#pragma once

#include <QString>

class QWidget;

namespace robot_qt_viewer
{
    class PaintingAnalysisDialogService
    {
    public:
        static QString selectModelFile(QWidget* parent);
        static bool selectModelUnitScale(
            QWidget* parent,
            const QString& modelPath,
            double& scaleToMeters);
        static QString selectTrajectoryFile(QWidget* parent);
    };
}
