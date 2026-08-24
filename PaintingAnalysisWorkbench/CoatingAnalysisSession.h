#pragma once

#include "PaintingAnalysisMeshAdapter.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <QString>

#include <cstddef>
#include <memory>
#include <vector>

namespace robot_qt_viewer
{
    struct CoatingAnalysisModelInfo
    {
        std::size_t subMeshCount = 0;
        std::size_t vertexCount = 0;
        std::size_t triangleCount = 0;
        double sizeXMeters = 0.0;
        double sizeYMeters = 0.0;
        double sizeZMeters = 0.0;
        bool valid = false;
    };

    struct CoatingAnalysisTrajectoryInfo
    {
        std::size_t pointCount = 0;
        std::size_t warningCount = 0;
        double startTimeSeconds = 0.0;
        double endTimeSeconds = 0.0;
        double durationSeconds = 0.0;
        double pathLengthMeters = 0.0;
        double averageSpeedMetersPerSecond = 0.0;
        double rangeXMeters = 0.0;
        double rangeYMeters = 0.0;
        double rangeZMeters = 0.0;
        bool valid = false;
    };

    struct CoatingAnalysisSession
    {
        QString objectId;
        QString modelName;
        QString sourcePath;
        QString trajectoryName;
        QString trajectoryPath;
        CoatingAnalysisModelInfo modelInfo;
        CoatingAnalysisTrajectoryInfo trajectoryInfo;
        spraytrajectory::SprayTrajectory trajectory;
        std::vector<spraytrajectory::SprayPathPoint> waypoints;
        PaintingAnalysisMeshBinding binding;
        std::shared_ptr<assetcore::ModelDesc> predictionDisplayModel;
        spraythickness::ThicknessPredictionResult prediction;
        smrobot::visualization::SurfaceScalarOverlay overlay;
        smrobot::visualization::SurfaceScalarOverlay thicknessOverlay;
        double predictionElapsedSeconds = 0.0;
        bool hasResult = false;
        bool showModel = true;
        bool showSprayPoints = true;
        bool showThickness = false;
        bool thicknessPickEnabled = false;
        bool showRelativeError = false;

        void clearResult()
        {
            binding.sampleIndicesBySubMesh.clear();
            predictionDisplayModel.reset();
            prediction = spraythickness::ThicknessPredictionResult();
            overlay = smrobot::visualization::SurfaceScalarOverlay();
            thicknessOverlay = smrobot::visualization::SurfaceScalarOverlay();
            predictionElapsedSeconds = 0.0;
            hasResult = false;
            showThickness = false;
            thicknessPickEnabled = false;
            showRelativeError = false;
        }

        void clear()
        {
            const bool preservedShowModel = showModel;
            const bool preservedShowSprayPoints = showSprayPoints;
            objectId.clear();
            modelName.clear();
            sourcePath.clear();
            trajectoryName.clear();
            trajectoryPath.clear();
            modelInfo = CoatingAnalysisModelInfo();
            trajectoryInfo = CoatingAnalysisTrajectoryInfo();
            trajectory = spraytrajectory::SprayTrajectory();
            waypoints.clear();
            clearResult();
            showModel = preservedShowModel;
            showSprayPoints = preservedShowSprayPoints;
        }
    };
}
