#pragma once

#include "PaintingAnalysisMeshAdapter.h"

#include <SprayThicknessPrediction/AlgorithmReproduction.h>

#include <memory>
#include <string>

namespace robot_qt_viewer
{
    struct PublishedReproductionDisplayData
    {
        std::shared_ptr<assetcore::ModelDesc> displayModel;
        PaintingAnalysisMeshBinding binding;
        spraythickness::ThicknessPredictionResult scalarField;
        std::string domainLabel;
    };

    class PublishedReproductionDisplayAdapter
    {
    public:
        static PublishedReproductionDisplayData build(
            const spraythickness::AlgorithmReproductionResult& result,
            const sprayworkpiece::WorkpieceModel& originalWorkpiece,
            const PaintingAnalysisMeshBinding& originalBinding);
    };
}
