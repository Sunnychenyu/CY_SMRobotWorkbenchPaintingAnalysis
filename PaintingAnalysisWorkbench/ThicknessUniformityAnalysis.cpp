#include "ThicknessUniformityAnalysis.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_qt_viewer
{
    ThicknessUniformityStatistics calculateThicknessUniformityStatistics(
        const spraythickness::ThicknessField& field,
        double minimumThicknessMeters,
        double maximumThicknessMeters)
    {
        ThicknessUniformityStatistics statistics;
        statistics.totalVertexCount = field.results.size();
        if(!std::isfinite(minimumThicknessMeters)
            || !std::isfinite(maximumThicknessMeters)
            || minimumThicknessMeters > maximumThicknessMeters) {
            return statistics;
        }

        double mean = 0.0;
        double squaredDeviationSum = 0.0;
        double includedMinimum = std::numeric_limits<double>::infinity();
        double includedMaximum = -std::numeric_limits<double>::infinity();
        for(const spraythickness::ThicknessSampleResult& result : field.results) {
            const double value = result.thickness;
            if(!std::isfinite(value)
                || value < minimumThicknessMeters
                || value > maximumThicknessMeters) {
                continue;
            }

            ++statistics.includedVertexCount;
            const double count = static_cast<double>(statistics.includedVertexCount);
            const double delta = value - mean;
            mean += delta / count;
            squaredDeviationSum += delta * (value - mean);
            includedMinimum = std::min(includedMinimum, value);
            includedMaximum = std::max(includedMaximum, value);
        }

        if(statistics.includedVertexCount == 0) {
            return statistics;
        }

        const double count = static_cast<double>(statistics.includedVertexCount);
        statistics.includedRatio = statistics.totalVertexCount > 0
            ? count / static_cast<double>(statistics.totalVertexCount)
            : 0.0;
        statistics.minimumThicknessMeters = includedMinimum;
        statistics.maximumThicknessMeters = includedMaximum;
        statistics.meanThicknessMeters = mean;
        statistics.varianceSquareMeters = squaredDeviationSum / count;
        statistics.standardDeviationMeters = std::sqrt(
            std::max(0.0, statistics.varianceSquareMeters));
        constexpr double kMeanEpsilonMeters = 1.0e-15;
        if(std::abs(mean) > kMeanEpsilonMeters) {
            statistics.coefficientOfVariation =
                statistics.standardDeviationMeters / std::abs(mean);
            statistics.coefficientOfVariationValid = true;
        }
        statistics.valid = true;
        return statistics;
    }
}
