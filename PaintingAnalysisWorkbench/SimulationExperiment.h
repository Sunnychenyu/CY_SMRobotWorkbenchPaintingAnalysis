#pragma once

#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <WorkpieceCore/WorkpieceModel.h>
#include <AssetCore/ModelDesc.h>

#include <QString>

#include <memory>

namespace robot_qt_viewer
{
    enum class SimulationExperimentKind
    {
        PointSpray = 0,
        LineScan = 1
    };

    struct SimulationExperimentParameters
    {
        SimulationExperimentKind kind{ SimulationExperimentKind::PointSpray };
        double plateSideMillimeters{ 200.0 };
        double cellSizeMillimeters{ 2.0 };
        double sprayDistanceMillimeters{ 100.0 };
        double trajectoryOverrunMillimeters{ 20.0 };
        double incidenceAngleDegrees{ 90.0 };
        double azimuthDegrees{ 0.0 };
        double toolRollDegrees{ 0.0 };
        double pointDurationSeconds{ 4.0 };
        double scanSpeedMillimetersPerSecond{ 20.0 };
        int scanPassCount{ 1 };
        double entrySpeedMillimetersPerSecond{ 20.0 };
        double exitSpeedMillimetersPerSecond{ 20.0 };
        double trajectoryPointIntervalSeconds{ 0.01 };
        double scanStartXMillimeters{ -50.0 };
        double scanStartYMillimeters{ 0.0 };
        double scanEndXMillimeters{ 50.0 };
        double scanEndYMillimeters{ 0.0 };
    };

    struct SimulationExperimentData
    {
        SimulationExperimentParameters parameters;
        std::shared_ptr<assetcore::ModelDesc> displayModel;
        sprayworkpiece::WorkpieceModel workpiece;
        spraytrajectory::SprayTrajectory trajectory;
        std::size_t rowCount{ 0 };
        std::size_t columnCount{ 0 };
        double actualCellSizeMeters{ 0.0 };
        double thicknessVolumeCubicMillimeters{ 0.0 };
    };

    struct PlateStackParameters
    {
        double plateSideMillimeters{ 200.0 };
        double cellSizeMillimeters{ 2.0 };
        int plateCount{ 2 };
        double plateSpacingMillimeters{ 15.0 };
    };

    struct PlateStackData
    {
        PlateStackParameters parameters;
        std::shared_ptr<assetcore::ModelDesc> displayModel;
        sprayworkpiece::WorkpieceModel workpiece;
        sprayworkpiece::WorkpieceModel raycastWorkpiece;
        std::size_t rowCount{ 0 };
        std::size_t columnCount{ 0 };
        double actualCellSizeMeters{ 0.0 };
    };

    class SimulationExperiment
    {
    public:
        static bool build(
            const SimulationExperimentParameters& parameters,
            SimulationExperimentData& output,
            QString* errorMessage = nullptr);

        static bool buildPlateStack(
            const PlateStackParameters& parameters,
            PlateStackData& output,
            QString* errorMessage = nullptr);

        static bool buildTrajectory(
            const SimulationExperimentParameters& parameters,
            spraytrajectory::SprayTrajectory& output,
            QString* errorMessage = nullptr);

        static bool exportContourCsv(
            const QString& filePath,
            const SimulationExperimentData& experiment,
            const spraythickness::ThicknessPredictionResult& prediction,
            QString* errorMessage = nullptr);
    };
}
