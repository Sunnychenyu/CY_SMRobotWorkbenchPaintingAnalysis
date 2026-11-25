#pragma once

#include <Eigen/Core>
#include <QString>

#include <cstddef>
#include <vector>

namespace robot_qt_viewer
{
    struct ThicknessPickReportSample
    {
        std::size_t triangleIndex{ 0 };
        Eigen::Vector3d positionMeters = Eigen::Vector3d::Zero();
        double thicknessMeters{ 0.0 };
        Eigen::Vector3d normalWorld = Eigen::Vector3d::Zero();
        double arcLengthMeters{ 0.0 };
        std::size_t profileIndex{ 0 };
    };

    struct ThicknessSectionSchematicOptions
    {
        bool automaticThicknessMultiplier{ true };
        double fixedThicknessMultiplier{ 25.0 };
    };

    struct ThicknessPickReportSection
    {
        std::vector<std::vector<ThicknessPickReportSample>> profiles;
        std::vector<ThicknessPickReportSample> annotations;
        double annotationSpacingMeters{ 0.01 };
        ThicknessSectionSchematicOptions schematicOptions;
    };

    struct ThicknessPickReportExportResult
    {
        QString filePath;
        std::size_t sampleCount{ 0 };
    };

    ThicknessPickReportExportResult exportThicknessPickReport(
        const std::vector<ThicknessPickReportSample>& samples,
        const QString& modelName,
        const QString& outputDirectory,
        const ThicknessPickReportSection* section = nullptr);
}
