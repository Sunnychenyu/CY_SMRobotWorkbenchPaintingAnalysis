#pragma once

#include <QString>

namespace robot_qt_viewer
{
    struct CoatingAnalysisViewModel
    {
        QString modelName = QStringLiteral("No model loaded");
        QString modelPath;
        QString status = QStringLiteral("Open a mesh model to begin.");
        bool hasModel = false;
        bool hasResult = false;
        bool showThickness = false;
        double minimumMicrometers = 0.0;
        double midpointMicrometers = 0.0;
        double maximumMicrometers = 0.0;
        double averageMicrometers = 0.0;
        bool hasCurrentThickness = false;
        double currentMicrometers = 0.0;
    };
}
