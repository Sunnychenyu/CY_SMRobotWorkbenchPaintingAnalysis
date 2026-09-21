#pragma once

#include <SprayThicknessPrediction/AlgorithmReproduction.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <filesystem>
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
        bool hasObjectRotationInput{ false };
    };

    struct PublishedReproductionRuntimeInputs
    {
        std::filesystem::path modelSourcePath;
        Eigen::Vector3d objectRotationOriginMeters = Eigen::Vector3d::Zero();
        Eigen::Vector3d objectRotationAxis = Eigen::Vector3d::UnitZ();
        double objectAngularSpeedRadiansPerSecond{ 0.0 };
    };

    class PublishedReproductionAdapter
    {
    public:
        static PublishedReproductionInputProfile inputProfile(
            spraythickness::ReproductionAlgorithmKind algorithm);

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
