#pragma once

#include "CoatingAnalysisSession.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <QHash>
#include <QString>
#include <QVector>

namespace robot_qt_viewer
{
    struct CoatingAnalysisWorkpieceItem
    {
        QString id;
        QString name;
    };

    struct CoatingAnalysisViewModel
    {
        QVector<CoatingAnalysisWorkpieceItem> workpieces;
        QString selectedWorkpieceId;
        QString modelName = QStringLiteral("No model loaded");
        QString modelPath;
        QString modelDetails;
        QString trajectoryName = QStringLiteral("No trajectory loaded");
        QString trajectoryPath;
        QString trajectoryDetails;
        QString status = QStringLiteral("Open a mesh model to begin.");
        bool hasModel = false;
        bool hasTrajectory = false;
        bool hasResult = false;
        bool predictionRunning = false;
        bool localMode = false;
        bool axisymmetricProfileMode = false;
        bool hasEffectiveRotationAxis = false;
        bool hasAxisymmetricProfileSelection = false;
        bool canStartPrediction = false;
        bool canPreviewLocalInputs = false;
        bool canSelectProfileRegion = false;
        bool hasLocalPreview = false;
        QString localPreviewDetails;
        QString axisymmetricProfileDetails;
        QString rotationAxisSource = QStringLiteral("Not configured");
        bool showModel = true;
        bool showSprayPoints = true;
        bool showThickness = false;
        bool thicknessPickEnabled = false;
        bool showCylindricalSurface = true;
        bool showRotationAxis = true;
        bool showLocalSector = true;
        bool showLocalSprayPoints = true;
        double progress = 0.0;
        double minimumMicrometers = 0.0;
        double midpointMicrometers = 0.0;
        double maximumMicrometers = 0.0;
        double averageMicrometers = 0.0;
        bool hasCurrentThickness = false;
        double currentMicrometers = 0.0;
        bool referenceAvailable = false;
        bool canSetReference = false;
        bool canClearReference = false;
        bool canCheckReference = false;
        QString referenceStatus = QStringLiteral("Reference: not set");
    };

    struct CoatingAnalysisInfoView
    {
        bool hasModel = false;
        QString modelName;
        QString modelPath;
        CoatingAnalysisModelInfo modelInfo;
        bool hasTrajectory = false;
        QString trajectoryName;
        CoatingAnalysisTrajectoryInfo trajectoryInfo;
        bool hasThickness = false;
        double predictionElapsedSeconds = 0.0;
        spraythickness::ThicknessMetrics thicknessMetrics;
        spraythickness::ThicknessPredictionTiming predictionTiming;
        QString validationDetails = QStringLiteral("No reference result.");
    };

    struct CoatingAnalysisVisibilityView
    {
        bool hasModel = false;
        bool hasTrajectory = false;
        bool hasThickness = false;
        bool showModel = true;
        bool showSprayPoints = true;
        bool showThickness = false;
        bool thicknessPickEnabled = false;
    };

    struct CoatingAnalysisTreeView
    {
        QString trajectoryName;
        CoatingAnalysisTrajectoryInfo trajectoryInfo;
        QVector<CoatingAnalysisWorkpieceItem> workpieces;
        QString selectedWorkpieceId;
        bool showModel = true;
        bool hasThickness = false;
        spraythickness::ThicknessMetrics thicknessMetrics;
        QHash<QString, bool> modelVisibility;
    };
}
