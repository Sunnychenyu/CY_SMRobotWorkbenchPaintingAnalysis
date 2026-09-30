#pragma once

#include <SprayThicknessPrediction/AlgorithmReproduction.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <filesystem>
#include <optional>
#include <string>

namespace robot_qt_viewer
{
    struct PublishedReproductionInputProfile
    {
        std::string modelInputName;
        std::string trajectoryInputName;
        std::string modelDialogTitle;
        std::string modelDialogFilter;
        std::string trajectoryDialogTitle;
        bool requiresStlSurface{ false };
        bool forceOriginalTrajectoryPoints{ false };
    };

    struct PublishedReproductionRuntimeInputs
    {
        std::filesystem::path modelSourcePath;
        bool generatedPlateStack{ false };
    };

    struct ReproductionTrajectoryPreset
    {
        double plateSideMillimeters{ 0.0 };
        int plateCount{ 0 };
        double plateSpacingMillimeters{ 0.0 };
        double cellSizeMillimeters{ 0.0 };
        double sprayDistanceMillimeters{ 0.0 };
        double incidenceAngleDegrees{ 0.0 };
        double scanSpeedMillimetersPerSecond{ 0.0 };
        double pointIntervalSeconds{ 0.0 };
        double overrunMillimeters{ 0.0 };
        int scanPassCount{ 0 };
    };

    class PublishedReproductionAdapter
    {
    public:
        static PublishedReproductionInputProfile inputProfile(
            spraythickness::ReproductionAlgorithmKind algorithm);

        static std::optional<ReproductionTrajectoryPreset> trajectoryPreset(
            spraythickness::ReproductionAlgorithmKind algorithm,
            const std::filesystem::path& configurationPath);

        static void writeConfigurationTemplate(
            spraythickness::ReproductionAlgorithmKind algorithm,
            const std::filesystem::path& path);

        static spraythickness::AlgorithmReproductionTask buildTask(
            spraythickness::ReproductionAlgorithmKind algorithm,
            const sprayworkpiece::WorkpieceModel& workpiece,
            const spraytrajectory::SprayTrajectory& trajectory,
            const spraycore::SprayTool& tool,
            spraythickness::TrajectorySamplingMode samplingMode,
            double timeStepSeconds,
            const std::filesystem::path& configurationPath,
            const PublishedReproductionRuntimeInputs& runtimeInputs = {});
    };
}
