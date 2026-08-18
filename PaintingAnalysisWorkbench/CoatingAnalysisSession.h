#pragma once

#include "PaintingAnalysisMeshAdapter.h"

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <QString>

namespace robot_qt_viewer
{
    struct CoatingAnalysisSession
    {
        QString objectId;
        QString modelName;
        QString sourcePath;
        PaintingAnalysisMeshBinding binding;
        spraythickness::ThicknessPredictionResult prediction;
        smrobot::visualization::SurfaceScalarOverlay overlay;
        bool hasResult = false;
        bool showThickness = false;

        void clearResult()
        {
            binding.sampleIndicesBySubMesh.clear();
            prediction = spraythickness::ThicknessPredictionResult();
            overlay = smrobot::visualization::SurfaceScalarOverlay();
            hasResult = false;
            showThickness = false;
        }

        void clear()
        {
            objectId.clear();
            modelName.clear();
            sourcePath.clear();
            clearResult();
        }
    };
}
