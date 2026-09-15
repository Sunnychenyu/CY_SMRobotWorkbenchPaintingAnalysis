#include "SimulationExperiment.h"

#include <QFile>
#include <QTextStream>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

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
            const double speed = std::max(0.001,
                parameters.scanSpeedMillimetersPerSecond * kMillimetersToMeters);
            const double entry = std::max(0.0, parameters.entrySpeedMillimetersPerSecond
                * kMillimetersToMeters);
            const double exit = std::max(0.0, parameters.exitSpeedMillimetersPerSecond
                * kMillimetersToMeters);
            const double entryDuration = entry > 0.0 ? overrunMeters / entry : 0.0;
            const double scanDuration = std::max(0.001, scanLength / speed);
            const double exitDuration = exit > 0.0 ? overrunMeters / exit : 0.0;
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
                    const Eigen::Vector3d legStartPosition =
                        legStart - sprayDirection * distanceMeters;
                    const Eigen::Vector3d legEndPosition =
                        legEnd - sprayDirection * distanceMeters;
                    const Eigen::Vector3d exitPosition =
                        exitTarget - sprayDirection * distanceMeters;
                    if(firstPoint) {
                        segment.points.push_back(makePoint(
                            currentTime, entryPosition, rotation, distanceMeters, true));
                        firstPoint = false;
                    }
                    currentTime += entryDuration;
                    segment.points.push_back(makePoint(
                        currentTime, legStartPosition, rotation, distanceMeters, true));
                    currentTime += scanDuration;
                    segment.points.push_back(makePoint(
                        currentTime, legEndPosition, rotation, distanceMeters, true));
                    currentTime += exitDuration;
                    segment.points.push_back(makePoint(
                        currentTime, exitPosition, rotation, distanceMeters, true));
                }
            }
            // Every configured pass contains a forward and a reverse leg, so
            // the final leg always returns to the start side of the plate.
            const Eigen::Vector3d finalDirection = -scanDirection;
            const Eigen::Vector3d finalExitTarget =
                start + finalDirection * overrunMeters;
            const Eigen::Vector3d shutdownTarget =
                finalExitTarget + finalDirection * overrunMeters;
            const double shutdownDuration = exit > 0.0
                ? overrunMeters / exit : 0.0;
            currentTime += shutdownDuration;
            segment.points.push_back(makePoint(
                currentTime,
                shutdownTarget - sprayDirection * distanceMeters,
                rotation,
                distanceMeters,
                false));
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
