#pragma once

#include "ThicknessPickReport.h"

#include <Eigen/Geometry>

#include <vector>

namespace assetcore
{
    class ModelDesc;
}

namespace smrobot::visualization
{
    struct SurfaceScalarOverlay;
}

namespace robot_qt_viewer
{
    // Sample the original vertex thickness field on the world plane X = 0.
    // Adjacent points on each triangle intersection are at most 0.2 mm apart.
    // Coplanar triangles contribute their three mesh edges, not an area grid.
    std::vector<ThicknessPickReportSample> sampleYzThickness(
        const assetcore::ModelDesc& model,
        const smrobot::visualization::SurfaceScalarOverlay& thicknessOverlay,
        const Eigen::Isometry3d& worldFromModel);
    // Exact world X=0 intersections. Coplanar faces contribute their boundary only.
    struct ThicknessPickSectionSegment
    {
        ThicknessPickReportSample first;
        ThicknessPickReportSample second;
    };

    std::vector<ThicknessPickSectionSegment> extractYzThicknessSegments(
        const assetcore::ModelDesc& model,
        const smrobot::visualization::SurfaceScalarOverlay& thicknessOverlay,
        const Eigen::Isometry3d& worldFromModel);

    // Polygon coordinates are world Y/Z in meters. Valid empty selections return empty.
    std::vector<ThicknessPickSectionSegment> clipYzThicknessSegments(
        const std::vector<ThicknessPickSectionSegment>& segments,
        const std::vector<Eigen::Vector2d>& polygon);

    // Connect the selected boundary geometrically. Disconnected components remain
    // independent profiles; ambiguous branches or conflicting duplicate faces fail.
    // Detail spacing is at most 0.2 mm; annotations use exact arc-length steps,
    // followed by the final endpoint when the length is not an exact multiple.
    ThicknessPickReportSection buildSelectedYzThicknessSection(
        const std::vector<ThicknessPickSectionSegment>& segments,
        const std::vector<Eigen::Vector2d>& polygon,
        double annotationSpacingMeters);
}
