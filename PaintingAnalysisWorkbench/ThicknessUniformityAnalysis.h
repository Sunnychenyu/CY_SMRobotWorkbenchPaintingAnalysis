#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <cstddef>

namespace robot_qt_viewer
{
    struct ThicknessUniformityStatistics
    {
        std::size_t totalVertexCount = 0;
        std::size_t includedVertexCount = 0;
        double includedRatio = 0.0;
        double minimumThicknessMeters = 0.0;
        double maximumThicknessMeters = 0.0;
        double meanThicknessMeters = 0.0;
        double varianceSquareMeters = 0.0;
        double standardDeviationMeters = 0.0;
        double coefficientOfVariation = 0.0;
        bool coefficientOfVariationValid = false;
        bool valid = false;
    };

    ThicknessUniformityStatistics calculateThicknessUniformityStatistics(
        const spraythickness::ThicknessField& field,
        double minimumThicknessMeters,
        double maximumThicknessMeters);
}
