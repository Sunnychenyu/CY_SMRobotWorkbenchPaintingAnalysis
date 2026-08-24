#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <AssetCore/ModelDesc.h>

#include <cstddef>
#include <cstdint>
#include <Eigen/Geometry>
#include <memory>
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
        std::shared_ptr<assetcore::ModelDesc> displayModel;
        std::vector<std::string> warnings;
    };

    struct AdaptiveMeshOptions
    {
        Eigen::Vector3d axisOrigin = Eigen::Vector3d::Zero();
        Eigen::Vector3d axisDirection = Eigen::Vector3d::UnitZ();
        Eigen::Vector2d selectionMinimum = Eigen::Vector2d::Zero();
        Eigen::Vector2d selectionMaximum = Eigen::Vector2d::Zero();
        std::vector<Eigen::Vector2d> selectionPolygon;
        // Percentage of non-protected vertices retained outside the dense region.
        double simplificationPercent = 30.0;
    };

    class PaintingAnalysisMeshAdapter
    {
    public:
        static PaintingAnalysisMeshData build(
            const assetcore::ModelDesc& model,
            const std::string& name,
            const std::string& sourcePath,
            const Eigen::Isometry3d& worldFromModel = Eigen::Isometry3d::Identity());

        static PaintingAnalysisMeshData buildAdaptive(
            const assetcore::ModelDesc& model,
            const std::string& name,
            const std::string& sourcePath,
            const Eigen::Isometry3d& worldFromModel,
            const AdaptiveMeshOptions& options);

        static std::vector<std::uint32_t> selectVerticesInRegion(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const AdaptiveMeshOptions& options);

        static smrobot::visualization::SurfaceScalarOverlay makeOverlay(
            const std::string& objectId,
            const PaintingAnalysisMeshBinding& binding,
            const spraythickness::ThicknessPredictionResult& prediction);

        static smrobot::visualization::SurfaceScalarOverlay makeRelativeErrorOverlay(
            const std::string& objectId,
            const PaintingAnalysisMeshBinding& binding,
            const spraythickness::ThicknessField& reference,
            const spraythickness::ThicknessField& candidate,
            const std::vector<std::uint8_t>* comparisonMask = nullptr);
    };
}
