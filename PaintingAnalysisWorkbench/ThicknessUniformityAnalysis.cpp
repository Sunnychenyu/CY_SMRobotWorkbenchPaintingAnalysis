#include "ThicknessUniformityAnalysis.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <thread>
#ifdef _MSC_VER
#include <execution>
#endif

namespace robot_qt_viewer
{
    template<typename ThicknessAt>
    static ThicknessUniformityStatistics calculateUniformity(
        std::size_t vertexCount, ThicknessAt thicknessAt,
        double minimumThicknessMeters,
        double maximumThicknessMeters)
    {
        ThicknessUniformityStatistics statistics;
        statistics.totalVertexCount = vertexCount;
        if(!std::isfinite(minimumThicknessMeters)
            || !std::isfinite(maximumThicknessMeters)
            || minimumThicknessMeters > maximumThicknessMeters) {
            return statistics;
        }

        struct Moments
        {
            std::size_t count = 0;
            double mean = 0.0;
            double squaredDeviationSum = 0.0;
            double minimum = std::numeric_limits<double>::infinity();
            double maximum = -std::numeric_limits<double>::infinity();
        };
        constexpr std::size_t maximumBlocks = 8;
        std::size_t blocks = 1;
#ifdef _MSC_VER
        static const std::size_t cpuBudget = std::max(1u, std::thread::hardware_concurrency() / 2);
        blocks = std::min({ maximumBlocks, cpuBudget,
            std::max<std::size_t>(1, vertexCount / 65536) });
#endif
        std::array<Moments, maximumBlocks> partials;
#ifdef _MSC_VER
        std::array<std::size_t, maximumBlocks> blockIndices{ 0, 1, 2, 3, 4, 5, 6, 7 };
#endif
        const auto accumulateBlock = [&](std::size_t block) {
            Moments local;
            for(std::size_t index = vertexCount * block / blocks;
                index < vertexCount * (block + 1) / blocks; ++index) {
                const double value = thicknessAt(index);
                if(!std::isfinite(value) || value < minimumThicknessMeters
                    || value > maximumThicknessMeters) continue;
                ++local.count;
                const double delta = value - local.mean;
                local.mean += delta / static_cast<double>(local.count);
                local.squaredDeviationSum += delta * (value - local.mean);
                local.minimum = std::min(local.minimum, value);
                local.maximum = std::max(local.maximum, value);
            }
            partials[block] = local;
        };
#ifdef _MSC_VER
        if(blocks > 1) {
            std::for_each(std::execution::par, blockIndices.begin(),
                blockIndices.begin() + blocks, accumulateBlock);
        } else
#endif
        {
            accumulateBlock(0);
        }
        Moments combined;
        for(std::size_t block = 0; block < blocks; ++block) {
            const auto& part = partials[block];
            if(part.count == 0) continue;
            if(combined.count == 0) { combined = part; continue; }
            // Chan's merge preserves Welford's numerical stability; no E[x^2]
            // subtraction for a nearly uniform coating. Merge in fixed order.
            const double count = static_cast<double>(combined.count + part.count);
            const double delta = part.mean - combined.mean;
            combined.squaredDeviationSum += part.squaredDeviationSum
                + delta * delta * (static_cast<double>(combined.count) * part.count / count);
            combined.mean += delta * (static_cast<double>(part.count) / count);
            combined.count += part.count;
            combined.minimum = std::min(combined.minimum, part.minimum);
            combined.maximum = std::max(combined.maximum, part.maximum);
        }
        statistics.includedVertexCount = combined.count;

        if(statistics.includedVertexCount == 0) {
            return statistics;
        }

        const double count = static_cast<double>(statistics.includedVertexCount);
        statistics.includedRatio = statistics.totalVertexCount > 0
            ? count / static_cast<double>(statistics.totalVertexCount)
            : 0.0;
        statistics.minimumThicknessMeters = combined.minimum;
        statistics.maximumThicknessMeters = combined.maximum;
        statistics.meanThicknessMeters = combined.mean;
        statistics.varianceSquareMeters = combined.squaredDeviationSum / count;
        statistics.standardDeviationMeters = std::sqrt(
            std::max(0.0, statistics.varianceSquareMeters));
        constexpr double kMeanEpsilonMeters = 1.0e-15;
        if(std::abs(combined.mean) > kMeanEpsilonMeters) {
            statistics.coefficientOfVariation =
                statistics.standardDeviationMeters / std::abs(combined.mean);
            statistics.coefficientOfVariationValid = true;
        }
        statistics.valid = true;
        return statistics;
    }

    ThicknessUniformityStatistics calculateThicknessUniformityStatistics(
        const spraythickness::ThicknessField& field,
        double minimumThicknessMeters, double maximumThicknessMeters)
    {
        return calculateUniformity(field.results.size(),
            [&](std::size_t index) { return field.results[index].thickness; },
            minimumThicknessMeters, maximumThicknessMeters);
    }

    ThicknessUniformityStatistics calculateThicknessUniformityStatistics(
        const spraythickness::OnlineThicknessSnapshot& snapshot,
        double minimumThicknessMeters, double maximumThicknessMeters)
    {
        if(snapshot.residentVertexCount) {
            ThicknessUniformityStatistics statistics;
            statistics.totalVertexCount = snapshot.size();
            // Online mode uses the automatic full range. A narrower range needs
            // a new GPU reduction, never an implicit full-array CPU download.
            if(snapshot.finiteVertexCount == 0
                || minimumThicknessMeters > snapshot.metrics.minThickness
                || maximumThicknessMeters < snapshot.metrics.maxThickness) return statistics;
            statistics.includedVertexCount = snapshot.finiteVertexCount;
            statistics.includedRatio = static_cast<double>(snapshot.finiteVertexCount) / snapshot.size();
            statistics.minimumThicknessMeters = snapshot.metrics.minThickness;
            statistics.maximumThicknessMeters = snapshot.metrics.maxThickness;
            statistics.meanThicknessMeters = snapshot.metrics.averageThickness;
            statistics.varianceSquareMeters = snapshot.varianceSquareMeters;
            statistics.standardDeviationMeters = std::sqrt(std::max(0.0, snapshot.varianceSquareMeters));
            statistics.coefficientOfVariationValid = std::abs(statistics.meanThicknessMeters) > 1.0e-15;
            if(statistics.coefficientOfVariationValid) {
                statistics.coefficientOfVariation = statistics.standardDeviationMeters
                    / std::abs(statistics.meanThicknessMeters);
            }
            statistics.valid = true;
            return statistics;
        }
        return calculateUniformity(snapshot.size(),
            [&](std::size_t index) { return snapshot.thicknessMeters(index); },
            minimumThicknessMeters, maximumThicknessMeters);
    }
}
