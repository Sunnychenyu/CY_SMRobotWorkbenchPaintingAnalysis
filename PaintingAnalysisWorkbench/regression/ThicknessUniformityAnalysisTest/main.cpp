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
    return 0;
}
