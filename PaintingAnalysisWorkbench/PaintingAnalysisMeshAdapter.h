#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <AssetCore/ModelDesc.h>

#include <cstddef>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace robot_qt_viewer
{
    struct PaintingAnalysisMeshBinding
    {
        std::vector<std::vector<std::size_t>> sampleIndicesBySubMesh;
    };

    struct PaintingAnalysisMeshData
    {
        sprayworkpiece::WorkpieceModel workpiece;
        PaintingAnalysisMeshBinding binding;
        std::vector<std::string> warnings;
    };

    class PaintingAnalysisMeshAdapter
    {
    public:
        static PaintingAnalysisMeshData build(
            const assetcore::ModelDesc& model,
            const std::string& name,
            const std::string& sourcePath,
            const Eigen::Isometry3d& worldFromModel = Eigen::Isometry3d::Identity());

        static smrobot::visualization::SurfaceScalarOverlay makeOverlay(
            const std::string& objectId,
            const PaintingAnalysisMeshBinding& binding,
            const spraythickness::ThicknessPredictionResult& prediction);

        static smrobot::visualization::SurfaceScalarOverlay makeRelativeErrorOverlay(
            const std::string& objectId,
            const PaintingAnalysisMeshBinding& binding,
            const spraythickness::ThicknessField& reference,
            const spraythickness::ThicknessField& candidate);
    };
}
