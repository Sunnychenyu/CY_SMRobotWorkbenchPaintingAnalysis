#pragma once
#include "DynamicSurfacePanel.h"
#include "PaintingAnalysisMeshAdapter.h"
#include <VisualizationSDK/SurfaceScalarOverlay.h>

namespace robot_qt_viewer
{
    struct CoatingAnalysisSession;
    struct DynamicSurfaceView
    {
        std::shared_ptr<assetcore::ModelDesc> model;
        PaintingAnalysisMeshBinding binding;
        smrobot::visualization::SurfaceScalarOverlay overlay;
    };
    DynamicSurfaceView makeDynamicSurfaceView(const QString& objectId,
        const spraythickness::DynamicSurfaceSnapshot& snapshot,
        const Eigen::Isometry3d& worldFromModel,const DynamicSurfaceDisplay& display);
    bool saveDynamicSurface(const QString& path,const CoatingAnalysisSession& session,
        bool ply,QString* error);
}
