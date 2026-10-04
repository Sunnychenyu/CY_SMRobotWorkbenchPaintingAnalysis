#include "ThicknessUniformityAnalysis.h"

#include <cmath>
#include <limits>

namespace
{
    bool nearlyEqual(double left, double right, double tolerance = 1.0e-12)
    {
        return std::abs(left - right) <= tolerance;
    }

    spraythickness::ThicknessSampleResult sample(std::size_t index, double thickness)
    {
        spraythickness::ThicknessSampleResult result;
        result.sampleIndex = index;
        result.thickness = thickness;
        return result;
    }
}

int main()
{
    spraythickness::ThicknessField field;
    field.results = {
        sample(0, 0.0),
        sample(1, 1.0e-6),
        sample(2, 2.0e-6),
        sample(3, 3.0e-6),
        sample(4, std::numeric_limits<double>::quiet_NaN())
    };

    const robot_qt_viewer::ThicknessUniformityStatistics statistics =
        robot_qt_viewer::calculateThicknessUniformityStatistics(
            field, 1.0e-6, 3.0e-6);
    if(!statistics.valid
        || statistics.totalVertexCount != 5
        || statistics.includedVertexCount != 3
        || !nearlyEqual(statistics.includedRatio, 0.6)
        || !nearlyEqual(statistics.minimumThicknessMeters, 1.0e-6)
        || !nearlyEqual(statistics.maximumThicknessMeters, 3.0e-6)
        || !nearlyEqual(statistics.meanThicknessMeters, 2.0e-6)
        || !nearlyEqual(
            statistics.varianceSquareMeters, 2.0e-12 / 3.0, 1.0e-24)
        || !nearlyEqual(
            statistics.standardDeviationMeters, std::sqrt(2.0e-12 / 3.0))
        || !statistics.coefficientOfVariationValid
        || !nearlyEqual(
            statistics.coefficientOfVariation,
            std::sqrt(2.0e-12 / 3.0) / 2.0e-6)) {
        return 1;
    }

    const robot_qt_viewer::ThicknessUniformityStatistics empty =
        robot_qt_viewer::calculateThicknessUniformityStatistics(
            field, 4.0e-6, 5.0e-6);
    if(empty.valid || empty.includedVertexCount != 0) {
        return 2;
    }

    const robot_qt_viewer::ThicknessUniformityStatistics zeroMean =
        robot_qt_viewer::calculateThicknessUniformityStatistics(field, 0.0, 0.0);
    if(!zeroMean.valid || zeroMean.includedVertexCount != 1
        || zeroMean.coefficientOfVariationValid) {
        return 3;
    }
    // Exercise the compact units, closed boundaries and the large-field block
    // reduction with a known population distribution: 1, 2 and 3 millimetres.
    spraythickness::OnlineThicknessSnapshot compact;
    compact.thicknessMillimeters.resize(262144);
    for(std::size_t i = 0; i < compact.size(); ++i) {
        compact.thicknessMillimeters[i] = static_cast<float>(i % 4);
    }
    const auto compactStatistics = robot_qt_viewer::calculateThicknessUniformityStatistics(
        compact, compact.thicknessMeters(1), compact.thicknessMeters(3));
    if(!compactStatistics.valid || compactStatistics.includedVertexCount != 196608
        || !nearlyEqual(compactStatistics.meanThicknessMeters, 0.002)
        || !nearlyEqual(compactStatistics.varianceSquareMeters, 2.0e-6 / 3.0, 1.0e-18)
        || !nearlyEqual(compactStatistics.includedRatio, 0.75)) return 4;
    compact.thicknessMillimeters.assign(10, std::numeric_limits<float>::quiet_NaN());
    if(robot_qt_viewer::calculateThicknessUniformityStatistics(compact, 0.0, 1.0).valid)
        return 5;

    spraythickness::OnlineThicknessSnapshot resident;
    resident.residentVertexCount = 4;
    resident.finiteVertexCount = 3;
    resident.metrics.minThickness = 0.001;
    resident.metrics.maxThickness = 0.003;
    resident.metrics.averageThickness = 0.002;
    resident.varianceSquareMeters = 2.0e-6 / 3.0;
    const auto gpuStatistics = robot_qt_viewer::calculateThicknessUniformityStatistics(resident, 0.001, 0.003);
    if(!gpuStatistics.valid || !nearlyEqual(gpuStatistics.includedRatio, 0.75)
        || !nearlyEqual(gpuStatistics.varianceSquareMeters, resident.varianceSquareMeters)
        || !gpuStatistics.coefficientOfVariationValid || !resident.thicknessMillimeters.empty()) return 7;
    if(robot_qt_viewer::calculateThicknessUniformityStatistics(resident, 0.002, 0.003).valid) return 8;

    spraythickness::ThicknessMetricsAccumulator all, first, second, emptyMetrics;
    for(std::size_t i = 0; i < 101; ++i) {
        auto value = sample(i, static_cast<double>(i) * 0.001);
        value.targetThickness = 0.05;
        value.error = value.thickness - value.targetThickness;
        all.add(value);
        (i < 37 ? first : second).add(value);
    }
    first.merge(emptyMetrics);
    first.merge(second);
    const auto expected = all.metrics();
    const auto merged = first.metrics();
    if(!nearlyEqual(expected.averageThickness, merged.averageThickness)
        || !nearlyEqual(expected.meanError, merged.meanError)
        || expected.minThickness != merged.minThickness
        || expected.maxThickness != merged.maxThickness
        || expected.maxAbsError != merged.maxAbsError
        || expected.coverageRatio != merged.coverageRatio
        || expected.underCoatedRatio != merged.underCoatedRatio
        || expected.overCoatedRatio != merged.overCoatedRatio) return 6;
    return 0;
}
