#pragma once

#include "ThicknessPickReport.h"

#include <Eigen/Core>

#include <vector>
#include <stdexcept>

namespace robot_qt_viewer
{
    class ThicknessDisplayMultiplierError : public std::runtime_error
    {
    public:
        ThicknessDisplayMultiplierError()
            : std::runtime_error("The fixed thickness display multiplier creates a crossing or folded coating outline. Choose Automatic or a smaller fixed multiplier.") {}
    };

    struct ThicknessSectionSchematicPoint
    {
        Eigen::Vector2d baseYZ = Eigen::Vector2d::Zero();
        Eigen::Vector2d coatingYZ = Eigen::Vector2d::Zero();
        Eigen::Vector2d normalYZ = Eigen::Vector2d::Zero();
        double arcLengthMeters{ 0.0 };
    };

    struct ThicknessSectionSchematicGeometry
    {
        std::vector<ThicknessSectionSchematicPoint> points;
        std::vector<ThicknessSectionSchematicPoint> annotations;
        double thicknessMultiplier{ 1.0 };
        double smoothingWindowMeters{ 0.0 };
    };

    // Drawing-only geometry. Input field values, annotation values and base
    // coordinates are never modified. Build once per full profile, then crop
    // its returned curve for detail panels so all panels use one multiplier.
    // Every annotation is an exact vertex of the returned coating curve.
    ThicknessSectionSchematicGeometry buildThicknessSectionSchematic(
        const std::vector<ThicknessPickReportSample>& profile,
        const std::vector<ThicknessPickReportSample>& annotations,
        const ThicknessSectionSchematicOptions& options = {});
}
