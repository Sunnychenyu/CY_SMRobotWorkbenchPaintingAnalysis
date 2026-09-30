#include "SimulationExperiment.h"

#include <QFile>
#include <QTextStream>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kMillimetersToMeters = 1.0e-3;
        constexpr double kDegreesToRadians = 0.01745329251994329576923690768489;

        Eigen::Matrix3d makeToolRotation(
            double incidenceDegrees,
            double azimuthDegrees,
            double toolRollDegrees)
        {
            const double incidence = incidenceDegrees * kDegreesToRadians;
            const double azimuth = azimuthDegrees * kDegreesToRadians;
            Eigen::Vector3d direction(
                std::cos(incidence) * std::cos(azimuth),
                std::cos(incidence) * std::sin(azimuth),
                -std::sin(incidence));
            if(direction.norm() < 1.0e-12) {
                direction = -Eigen::Vector3d::UnitZ();
            }
            direction.normalize();
            // The simulation gun uses its local +Z axis as the nozzle axis.
            // For the default plate setup this axis points down toward -Z.
            const Eigen::Quaterniond alignment = Eigen::Quaterniond::FromTwoVectors(
                Eigen::Vector3d::UnitZ(), direction);
            const Eigen::AngleAxisd roll(
                toolRollDegrees * kDegreesToRadians,
                Eigen::Vector3d::UnitZ());
            // Roll is a local-axis rotation, so it is applied on the right
            // after aligning local +Z with the world spray direction.
            return alignment.normalized().toRotationMatrix() * roll.toRotationMatrix();
        }

        spraytrajectory::SprayPathPoint makePoint(
            double time,
            const Eigen::Vector3d& position,
            const Eigen::Matrix3d& rotation,
            double targetDistance,
            bool sprayEnabled)
        {
            spraytrajectory::SprayPathPoint point;
            point.time = time;
            point.tcpPose = Eigen::Isometry3d::Identity();
            point.tcpPose.linear() = rotation;
            point.tcpPose.translation() = position;
            point.sprayEnabled = sprayEnabled;
            point.processId = "simulation";
            point.targetDistance = targetDistance;
            point.targetNormal = Eigen::Vector3d::UnitZ();
            return point;
        }
    }

    bool SimulationExperiment::build(
        const SimulationExperimentParameters& parameters,
        SimulationExperimentData& output,
        QString* errorMessage)
    {
        if(parameters.plateSideMillimeters <= 0.0 || parameters.cellSizeMillimeters <= 0.0
            || !std::isfinite(parameters.trajectoryPointIntervalSeconds)
            || parameters.trajectoryPointIntervalSeconds <= 0.0) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "Plate side, cell size and trajectory point interval must be positive.");
            }
            return false;
        }

        const std::size_t columns = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::llround(
                parameters.plateSideMillimeters / parameters.cellSizeMillimeters)));
        const std::size_t rows = columns;
        const double sideMeters = parameters.plateSideMillimeters * kMillimetersToMeters;
        const double cellMeters = sideMeters / static_cast<double>(columns);

        auto model = std::make_shared<assetcore::ModelDesc>();
        assetcore::SubMeshDesc subMesh;
        subMesh.name = "Simulation plate";
        const std::size_t vertexColumns = columns + 1;
        const std::size_t vertexRows = rows + 1;
        subMesh.geometry.positions.reserve(vertexColumns * vertexRows);
        subMesh.geometry.normals.reserve(vertexColumns * vertexRows);
        subMesh.geometry.colors.reserve(vertexColumns * vertexRows);
        for(std::size_t row = 0; row < vertexRows; ++row) {
            for(std::size_t column = 0; column < vertexColumns; ++column) {
                const double x = (static_cast<double>(column) / columns - 0.5) * sideMeters;
                const double y = (static_cast<double>(row) / rows - 0.5) * sideMeters;
                subMesh.geometry.positions.emplace_back(
                    static_cast<float>(x), static_cast<float>(y), 0.0f);
                subMesh.geometry.normals.emplace_back(0.0f, 0.0f, 1.0f);
                subMesh.geometry.colors.emplace_back(0.78f, 0.78f, 0.78f);
            }
        }
        subMesh.geometry.indices.reserve(columns * rows * 6);
        for(std::size_t row = 0; row < rows; ++row) {
            for(std::size_t column = 0; column < columns; ++column) {
                const std::uint32_t a = static_cast<std::uint32_t>(row * vertexColumns + column);
                const std::uint32_t b = a + 1;
                const std::uint32_t c = static_cast<std::uint32_t>((row + 1) * vertexColumns + column);
                const std::uint32_t d = c + 1;
                subMesh.geometry.indices.insert(
                    subMesh.geometry.indices.end(), { a, b, c, b, d, c });
            }
        }
        model->addSubMesh(subMesh);

        sprayworkpiece::WorkpieceModel workpiece;
        workpiece.name = "Simulation plate";
        workpiece.sourceMeshPath = "simulation://plate";
        workpiece.samples.reserve(vertexColumns * vertexRows);
        for(std::size_t row = 0; row < vertexRows; ++row) {
            for(std::size_t column = 0; column < vertexColumns; ++column) {
                sprayworkpiece::SurfaceSample sample;
                const double x = (static_cast<double>(column) / columns - 0.5) * sideMeters;
                const double y = (static_cast<double>(row) / rows - 0.5) * sideMeters;
                sample.position = Eigen::Vector3d(x, y, 0.0);
                sample.normal = Eigen::Vector3d::UnitZ();
                sample.areaWeight = cellMeters * cellMeters;
                workpiece.addSample(sample);
            }
        }
        workpiece.triangleIndices = subMesh.geometry.indices;

        const Eigen::Matrix3d rotation = makeToolRotation(
            parameters.incidenceAngleDegrees,
            parameters.azimuthDegrees,
            parameters.toolRollDegrees);
        const double distanceMeters = parameters.sprayDistanceMillimeters * kMillimetersToMeters;
        const double overrunMeters = std::max(0.0,
            parameters.trajectoryOverrunMillimeters * kMillimetersToMeters);
        const Eigen::Vector3d sprayDirection =
            (rotation * Eigen::Vector3d::UnitZ()).normalized();
        spraytrajectory::SprayTrajectory trajectory;
        trajectory.name = parameters.kind == SimulationExperimentKind::PointSpray
            ? "Point spray simulation" : "Line scan simulation";
        spraytrajectory::SpraySegment segment;
        segment.processId = "simulation";
        segment.sprayEnabled = true;
        if(parameters.kind == SimulationExperimentKind::PointSpray) {
            const double duration = std::max(0.001, parameters.pointDurationSeconds);
            const Eigen::Vector3d targetPoint = Eigen::Vector3d::Zero();
            // Keep the nozzle at the configured distance while it travels
            // along the plate plane: enter from outside the left edge, dwell
            // at the center, then leave beyond the right edge.
            const Eigen::Vector3d pathDirection = Eigen::Vector3d::UnitX();
            const double travelDistance = parameters.plateSideMillimeters * 0.5
                * kMillimetersToMeters + overrunMeters;
            const double entrySpeed = std::max(0.0,
                parameters.entrySpeedMillimetersPerSecond * kMillimetersToMeters);
            const double exitSpeed = std::max(0.0,
                parameters.exitSpeedMillimetersPerSecond * kMillimetersToMeters);
            const double entryDuration = entrySpeed > 0.0
                ? travelDistance / entrySpeed : 0.0;
            const double exitDuration = exitSpeed > 0.0
                ? travelDistance / exitSpeed : 0.0;
            const double shutdownDistance = overrunMeters;
            const double shutdownDuration = exitSpeed > 0.0
                ? shutdownDistance / exitSpeed : 0.0;
            const Eigen::Vector3d entryTarget =
                targetPoint - pathDirection * travelDistance;
            const Eigen::Vector3d exitTarget =
                targetPoint + pathDirection * travelDistance;
            const Eigen::Vector3d entryPosition =
                entryTarget - sprayDirection * distanceMeters;
            const Eigen::Vector3d pointPosition =
                targetPoint - sprayDirection * distanceMeters;
            const Eigen::Vector3d exitPosition =
                exitTarget - sprayDirection * distanceMeters;
            const Eigen::Vector3d shutdownPosition =
                (exitTarget + pathDirection * shutdownDistance)
                - sprayDirection * distanceMeters;
            segment.points.push_back(makePoint(
                0.0, entryPosition, rotation, distanceMeters, true));
            segment.points.push_back(makePoint(
                entryDuration, pointPosition, rotation, distanceMeters, true));
            segment.points.push_back(makePoint(
                entryDuration + duration, pointPosition, rotation, distanceMeters, true));
            segment.points.push_back(makePoint(
                entryDuration + duration + exitDuration,
                exitPosition, rotation, distanceMeters, true));
            segment.points.push_back(makePoint(
                entryDuration + duration + exitDuration + shutdownDuration,
                shutdownPosition, rotation, distanceMeters, false));
        } else {
            const Eigen::Vector3d start(
                parameters.scanStartXMillimeters * kMillimetersToMeters,
                parameters.scanStartYMillimeters * kMillimetersToMeters,
                0.0);
            const Eigen::Vector3d end(
                parameters.scanEndXMillimeters * kMillimetersToMeters,
                parameters.scanEndYMillimeters * kMillimetersToMeters,
                0.0);
            const double scanLength = (end - start).norm();
            if(scanLength <= 1.0e-12) {
                if(errorMessage != nullptr) {
                    *errorMessage = QStringLiteral(
                        "Line scan start and end points must be different.");
                }
                return false;
            }
            if(!std::isfinite(parameters.scanSpeedMillimetersPerSecond)
                || parameters.scanSpeedMillimetersPerSecond <= 0.0) {
                if(errorMessage != nullptr) {
                    *errorMessage = QStringLiteral("Scan speed must be positive.");
                }
                return false;
            }
            const double speed = parameters.scanSpeedMillimetersPerSecond
                * kMillimetersToMeters;
            const double overrunDuration = overrunMeters / speed;
            const double scanDuration = scanLength / speed;
            const Eigen::Vector3d scanDirection = (end - start).normalized();
            const int passCount = std::max(1, parameters.scanPassCount);
            double currentTime = 0.0;
            bool firstPoint = true;
            for(int pass = 0; pass < passCount; ++pass) {
                // One pass is a complete round trip. Each pass therefore
                // contains a forward and a reverse scan across the plate.
                for(int leg = 0; leg < 2; ++leg) {
                    const bool forward = (leg == 0);
                    const Eigen::Vector3d legStart = forward ? start : end;
                    const Eigen::Vector3d legEnd = forward ? end : start;
                    const Eigen::Vector3d legDirection = forward
                        ? scanDirection : -scanDirection;
                    const Eigen::Vector3d entryTarget =
                        legStart - legDirection * overrunMeters;
                    const Eigen::Vector3d exitTarget =
                        legEnd + legDirection * overrunMeters;
                    const Eigen::Vector3d entryPosition =
                        entryTarget - sprayDirection * distanceMeters;
                    const Eigen::Vector3d exitPosition =
                        exitTarget - sprayDirection * distanceMeters;
                    if(firstPoint) {
                        segment.points.push_back(makePoint(
                            currentTime, entryPosition, rotation, distanceMeters, true));
                        firstPoint = false;
                    }
                    // The entire leg has one speed; plate-edge knots would add
                    // extra deposition samples when each interval is resampled.
                    currentTime += scanDuration + 2.0 * overrunDuration;
                    segment.points.push_back(makePoint(
                        currentTime, exitPosition, rotation, distanceMeters, true));
                }
            }
            // A round trip already ends at the start-side overrun. Turn off
            // spray there without extending the path beyond that endpoint.
            segment.points.back().sprayEnabled = false;
        }
        trajectory.segments.push_back(std::move(segment));

        // The points above describe the experiment phases.  Materialize a
        // uniformly timed trajectory so the same points drive prediction,
        // viewport preview and export.
        const auto samples = spraytrajectory::SprayTrajectorySampler::sample(
            trajectory, parameters.trajectoryPointIntervalSeconds);
        if(samples.empty()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Failed to sample the simulation trajectory.");
            }
            return false;
        }
        spraytrajectory::SprayTrajectory sampledTrajectory;
        sampledTrajectory.name = trajectory.name;
        spraytrajectory::SpraySegment sampledSegment;
        sampledSegment.processId = "simulation";
        sampledSegment.sprayEnabled = true;
        sampledSegment.passIndex = trajectory.segments.front().passIndex;
        sampledSegment.points.reserve(samples.size());
        for(const auto& sample : samples) {
            spraytrajectory::SprayPathPoint point;
            point.time = sample.time;
            point.tcpPose = sample.tcpPose;
            point.jointValues = sample.jointValues;
            point.sprayEnabled = sample.sprayEnabled;
            point.processId = sample.processId;
            point.targetDistance = sample.targetDistance;
            point.targetNormal = sample.targetNormal;
            point.workpieceRegionId = sample.workpieceRegionId;
            sampledSegment.points.push_back(std::move(point));
        }
        sampledTrajectory.segments.push_back(std::move(sampledSegment));

        output.parameters = parameters;
        output.displayModel = std::move(model);
        output.workpiece = std::move(workpiece);
        output.trajectory = std::move(sampledTrajectory);
        output.rowCount = rows;
        output.columnCount = columns;
        output.actualCellSizeMeters = cellMeters;
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    bool SimulationExperiment::buildPlateStack(
        const PlateStackParameters& parameters,
        PlateStackData& output,
        QString* errorMessage)
    {
        if(parameters.plateSideMillimeters <= 0.0
            || parameters.cellSizeMillimeters <= 0.0
            || parameters.plateCount <= 0
            || (parameters.plateCount > 1
                && parameters.plateSpacingMillimeters <= 0.0)) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "Plate side, grid cell, plate count and spacing must be positive.");
            }
            return false;
        }

        const std::size_t columns = std::max<std::size_t>(
            1, static_cast<std::size_t>(std::llround(
                parameters.plateSideMillimeters / parameters.cellSizeMillimeters)));
        const std::size_t rows = columns;
        const std::size_t vertexColumns = columns + 1;
        const std::size_t vertexRows = rows + 1;
        constexpr std::size_t maximumVertexCount = 20000000;
        const long double requestedVertexCount =
            static_cast<long double>(vertexColumns)
            * static_cast<long double>(vertexRows)
            * static_cast<long double>(parameters.plateCount);
        if(requestedVertexCount > maximumVertexCount
            || requestedVertexCount
                > static_cast<long double>(std::numeric_limits<std::uint32_t>::max())) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "The plate stack exceeds the 20,000,000 vertex safety limit. "
                    "Increase the grid cell size or reduce the plate count.");
            }
            return false;
        }

        const std::size_t verticesPerPlate = vertexColumns * vertexRows;
        const std::size_t totalVertexCount =
            verticesPerPlate * static_cast<std::size_t>(parameters.plateCount);
        const std::size_t totalIndexCount = columns * rows * 6
            * static_cast<std::size_t>(parameters.plateCount);
        const double sideMeters = parameters.plateSideMillimeters
            * kMillimetersToMeters;
        const double cellMeters = sideMeters / static_cast<double>(columns);
        const double spacingMeters = parameters.plateSpacingMillimeters
            * kMillimetersToMeters;

        auto model = std::make_shared<assetcore::ModelDesc>();
        assetcore::SubMeshDesc subMesh;
        subMesh.name = "Algorithm reproduction plate stack";
        subMesh.geometry.positions.reserve(totalVertexCount);
        subMesh.geometry.normals.reserve(totalVertexCount);
        subMesh.geometry.colors.reserve(totalVertexCount);
        subMesh.geometry.indices.reserve(totalIndexCount);

        sprayworkpiece::WorkpieceModel workpiece;
        workpiece.name = "Algorithm reproduction plate stack";
        workpiece.sourceMeshPath =
            "simulation://algorithm-reproduction-plate-stack.stl";
        workpiece.samples.reserve(totalVertexCount);
        workpiece.triangleIndices.reserve(totalIndexCount);
        sprayworkpiece::WorkpieceModel raycastWorkpiece;
        raycastWorkpiece.name = "Algorithm reproduction square surfaces";
        raycastWorkpiece.sourceMeshPath = workpiece.sourceMeshPath;
        raycastWorkpiece.samples.reserve(
            4 * static_cast<std::size_t>(parameters.plateCount));
        raycastWorkpiece.triangleIndices.reserve(
            6 * static_cast<std::size_t>(parameters.plateCount));

        for(int plateIndex = 0; plateIndex < parameters.plateCount; ++plateIndex) {
            const double z = -static_cast<double>(plateIndex) * spacingMeters;
            const std::uint32_t cornerOffset = static_cast<std::uint32_t>(
                raycastWorkpiece.samples.size());
            for(const Eigen::Vector2d& corner : {
                    Eigen::Vector2d(-0.5, -0.5),
                    Eigen::Vector2d(0.5, -0.5),
                    Eigen::Vector2d(-0.5, 0.5),
                    Eigen::Vector2d(0.5, 0.5) }) {
                sprayworkpiece::SurfaceSample sample;
                sample.position = Eigen::Vector3d(
                    corner.x() * sideMeters, corner.y() * sideMeters, z);
                sample.normal = Eigen::Vector3d::UnitZ();
                sample.areaWeight = sideMeters * sideMeters * 0.25;
                raycastWorkpiece.addSample(sample);
            }
            raycastWorkpiece.triangleIndices.insert(
                raycastWorkpiece.triangleIndices.end(),
                { cornerOffset, cornerOffset + 1, cornerOffset + 2,
                  cornerOffset + 1, cornerOffset + 3, cornerOffset + 2 });
            const std::uint32_t vertexOffset = static_cast<std::uint32_t>(
                static_cast<std::size_t>(plateIndex) * verticesPerPlate);
            for(std::size_t row = 0; row < vertexRows; ++row) {
                for(std::size_t column = 0; column < vertexColumns; ++column) {
                    const double x =
                        (static_cast<double>(column) / columns - 0.5) * sideMeters;
                    const double y =
                        (static_cast<double>(row) / rows - 0.5) * sideMeters;
                    subMesh.geometry.positions.emplace_back(
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z));
                    subMesh.geometry.normals.emplace_back(0.0f, 0.0f, 1.0f);
                    subMesh.geometry.colors.emplace_back(0.78f, 0.78f, 0.78f);

                    sprayworkpiece::SurfaceSample sample;
                    sample.position = Eigen::Vector3d(x, y, z);
                    sample.normal = Eigen::Vector3d::UnitZ();
                    sample.areaWeight = cellMeters * cellMeters;
                    workpiece.addSample(sample);
                }
            }
            for(std::size_t row = 0; row < rows; ++row) {
                for(std::size_t column = 0; column < columns; ++column) {
                    const std::uint32_t a = vertexOffset
                        + static_cast<std::uint32_t>(row * vertexColumns + column);
                    const std::uint32_t b = a + 1;
                    const std::uint32_t c = vertexOffset
                        + static_cast<std::uint32_t>((row + 1) * vertexColumns + column);
                    const std::uint32_t d = c + 1;
                    subMesh.geometry.indices.insert(
                        subMesh.geometry.indices.end(), { a, b, c, b, d, c });
                    workpiece.triangleIndices.insert(
                        workpiece.triangleIndices.end(), { a, b, c, b, d, c });
                }
            }
        }

        model->addSubMesh(std::move(subMesh));
        output.parameters = parameters;
        output.displayModel = std::move(model);
        output.workpiece = std::move(workpiece);
        output.raycastWorkpiece = std::move(raycastWorkpiece);
        output.rowCount = rows;
        output.columnCount = columns;
        output.actualCellSizeMeters = cellMeters;
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }

    bool SimulationExperiment::buildTrajectory(
        const SimulationExperimentParameters& parameters,
        spraytrajectory::SprayTrajectory& output,
        QString* errorMessage)
    {
        SimulationExperimentParameters lightweight = parameters;
        lightweight.cellSizeMillimeters = std::max(
            parameters.plateSideMillimeters, 1.0);
        SimulationExperimentData experiment;
        if(!build(lightweight, experiment, errorMessage)) {
            return false;
        }
        output = std::move(experiment.trajectory);
        return true;
    }

    bool SimulationExperiment::exportContourCsv(
        const QString& filePath,
        const SimulationExperimentData& experiment,
        const spraythickness::ThicknessPredictionResult& prediction,
        QString* errorMessage)
    {
        if(filePath.isEmpty() || prediction.field.results.size() != experiment.workpiece.samples.size()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("The simulation result cannot be exported as a grid.");
            }
            return false;
        }
        QFile file(filePath);
        if(!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if(errorMessage != nullptr) {
                *errorMessage = file.errorString();
            }
            return false;
        }
        QTextStream stream(&file);
        stream.setRealNumberNotation(QTextStream::FixedNotation);
        stream.setRealNumberPrecision(6);
        stream << experiment.actualCellSizeMeters * 1.0e6 << '\n';
        const std::size_t vertexColumns = experiment.columnCount + 1;
        for(std::size_t row = 0; row <= experiment.rowCount; ++row) {
            for(std::size_t column = 0; column <= experiment.columnCount; ++column) {
                if(column != 0) {
                    stream << ',';
                }
                const std::size_t index = row * vertexColumns + column;
                stream << prediction.field.results[index].thickness * 1.0e6;
            }
            stream << '\n';
        }
        if(errorMessage != nullptr) {
            errorMessage->clear();
        }
        return true;
    }
}
