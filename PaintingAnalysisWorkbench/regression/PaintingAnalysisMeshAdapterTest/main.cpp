#include <PaintingAnalysisMeshAdapter.h>
#include <PublishedReproductionAdapter.h>
#include <PublishedReproductionDisplayAdapter.h>
#include <SimulationExperiment.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <variant>

namespace
{
    std::filesystem::path temporaryJsonPath(const char* name)
    {
        const auto suffix = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        return std::filesystem::temp_directory_path()
            / (std::string(name) + '_' + std::to_string(suffix) + ".json");
    }

    void writeText(const std::filesystem::path& path, const std::string& text)
    {
        std::ofstream output(path);
        output << text;
    }

    sprayworkpiece::WorkpieceModel makeWorkpiece()
    {
        sprayworkpiece::WorkpieceModel workpiece;
        workpiece.samples.push_back({ Eigen::Vector3d::Zero(),
            Eigen::Vector3d::UnitZ() });
        return workpiece;
    }

    sprayworkpiece::WorkpieceModel makeTriangleWorkpiece()
    {
        sprayworkpiece::WorkpieceModel workpiece;
        workpiece.samples = {
            { Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::UnitZ() },
            { Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d::UnitZ() },
            { Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d::UnitZ() }
        };
        workpiece.triangleIndices = { 0, 1, 2 };
        return workpiece;
    }

    spraytrajectory::SprayTrajectory makeTrajectory()
    {
        spraytrajectory::SprayTrajectory trajectory;
        spraytrajectory::SpraySegment segment;
        segment.sprayEnabled = true;
        for(double time : { 0.0, 1.0 }) {
            spraytrajectory::SprayPathPoint point;
            point.time = time;
            point.tcpPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.1);
            point.sprayEnabled = true;
            segment.points.push_back(point);
        }
        trajectory.segments.push_back(std::move(segment));
        return trajectory;
    }

    bool displayMatchesOverlay(
        const robot_qt_viewer::PublishedReproductionDisplayData& display,
        const assetcore::ModelDesc& fallbackModel)
    {
        const assetcore::ModelDesc& model = display.displayModel
            ? *display.displayModel : fallbackModel;
        const auto overlay = robot_qt_viewer::PaintingAnalysisMeshAdapter::makeOverlay(
            "reproduction", display.binding, display.scalarField);
        if(overlay.subMeshes.size() != model.subMeshes().size()) {
            return false;
        }
        for(std::size_t index = 0; index < overlay.subMeshes.size(); ++index) {
            const auto& values = overlay.subMeshes[index].values;
            if(values.size() != model.subMeshes()[index].geometry.positions.size()
                || !std::all_of(values.begin(), values.end(), [](double value) {
                    return std::isfinite(value) && value >= 0.0;
                })) {
                return false;
            }
        }
        return true;
    }

    bool testTzinavaAndCurrentDisplay()
    {
        const sprayworkpiece::WorkpieceModel workpiece = makeTriangleWorkpiece();
        assetcore::ModelDesc originalModel;
        assetcore::SubMeshDesc subMesh;
        for(const auto& sample : workpiece.samples) {
            subMesh.geometry.positions.push_back(sample.position.cast<float>());
        }
        subMesh.geometry.indices = { 0, 1, 2 };
        originalModel.addSubMesh(subMesh);
        robot_qt_viewer::PaintingAnalysisMeshBinding originalBinding;
        originalBinding.sampleIndicesBySubMesh = { { 0, 1, 2 } };

        spraythickness::AlgorithmReproductionResult current;
        current.algorithm = spraythickness::ReproductionAlgorithmKind::CurrentMethod;
        spraythickness::CurrentMethodReproductionResult currentNative;
        currentNative.prediction.field.resizeFromWorkpiece(workpiece);
        currentNative.prediction.field.results[0].thickness = 1.0e-6;
        current.nativeResult = currentNative;
        const auto currentDisplay =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                current, workpiece, originalBinding);
        if(!displayMatchesOverlay(currentDisplay, originalModel)) {
            return false;
        }

        spraythickness::AlgorithmReproductionResult tzinava;
        tzinava.algorithm = spraythickness::ReproductionAlgorithmKind::Tzinava2020;
        spraythickness::published::TzinavaResult tzinavaNative;
        for(const auto& sample : workpiece.samples) {
            tzinavaNative.subdividedMesh.vertices.push_back(sample.position);
        }
        tzinavaNative.subdividedMesh.faces.push_back({ 0, 1, 2 });
        tzinavaNative.faceThicknessMeters.push_back(2.0e-6);
        tzinava.nativeResult = std::move(tzinavaNative);
        const auto tzinavaDisplay =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                tzinava, workpiece, originalBinding);
        return tzinavaDisplay.displayModel
            && displayMatchesOverlay(tzinavaDisplay, originalModel)
            && tzinavaDisplay.scalarField.field.results[0].thickness == 2.0e-6;
    }

    bool testLineScanOverrun()
    {
        for(int passCount : { 1, 2 }) {
            robot_qt_viewer::SimulationExperimentParameters parameters;
            parameters.kind = robot_qt_viewer::SimulationExperimentKind::LineScan;
            parameters.scanPassCount = passCount;
            parameters.trajectoryPointIntervalSeconds = 0.5;
            spraytrajectory::SprayTrajectory trajectory;
            if(!robot_qt_viewer::SimulationExperiment::buildTrajectory(
                    parameters, trajectory)) {
                return false;
            }

            const auto points = trajectory.flattenedPoints();
            if(points.size() < 2) {
                return false;
            }
            const double expectedStartX = 0.001 *
                (parameters.scanStartXMillimeters
                    - parameters.trajectoryOverrunMillimeters);
            const double expectedEndX = 0.001 *
                (parameters.scanEndXMillimeters
                    + parameters.trajectoryOverrunMillimeters);
            const double expectedDuration = 2.0 * passCount *
                (parameters.scanEndXMillimeters - parameters.scanStartXMillimeters
                    + 2.0 * parameters.trajectoryOverrunMillimeters)
                / parameters.scanSpeedMillimetersPerSecond;
            if(std::abs(points.front().tcpPose.translation().x() - expectedStartX)
                    > 1.0e-9
                || std::abs(points.back().tcpPose.translation().x() - expectedStartX)
                    > 1.0e-9
                || std::abs(points.back().time - expectedDuration) > 1.0e-9
                || !points.front().sprayEnabled
                || points.back().sprayEnabled) {
                return false;
            }
            for(const auto& point : points) {
                const double x = point.tcpPose.translation().x();
                if(x < expectedStartX - 1.0e-9 || x > expectedEndX + 1.0e-9) {
                    return false;
                }
            }
        }
        return true;
    }

    bool testLineScanUniformSamplingAcrossPlateEdges()
    {
        robot_qt_viewer::SimulationExperimentParameters parameters;
        parameters.kind = robot_qt_viewer::SimulationExperimentKind::LineScan;
        parameters.plateSideMillimeters = 20.0;
        parameters.scanStartXMillimeters = -10.0;
        parameters.scanEndXMillimeters = 10.0;
        parameters.trajectoryOverrunMillimeters = 7.0;
        parameters.scanSpeedMillimetersPerSecond = 60.0;
        parameters.trajectoryPointIntervalSeconds = 0.01;
        spraytrajectory::SprayTrajectory trajectory;
        if(!robot_qt_viewer::SimulationExperiment::buildTrajectory(
                parameters, trajectory)) {
            return false;
        }
        const auto points = trajectory.flattenedPoints();
        std::size_t plateIntervals = 0;
        for(std::size_t index = 1; index < points.size(); ++index) {
            const double middleX = 0.5 * (
                points[index - 1].tcpPose.translation().x()
                + points[index].tcpPose.translation().x());
            if(std::abs(middleX) > 0.0105) {
                continue;
            }
            ++plateIntervals;
            const double interval = points[index].time - points[index - 1].time;
            if(std::abs(interval - parameters.trajectoryPointIntervalSeconds)
                > 1.0e-9) {
                return false;
            }
        }
        return plateIntervals > 0;
    }

    bool buildFailsWith(const std::filesystem::path& path,
        const std::string& json, const std::string& expected)
    {
        writeText(path, json);
        try {
            spraycore::SprayTool tool;
            robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Wu2020,
                makeTriangleWorkpiece(), makeTrajectory(), tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                0.1, path);
        } catch(const std::exception& exception) {
            std::filesystem::remove(path);
            return std::string(exception.what()).find(expected)
                != std::string::npos;
        }
        std::filesystem::remove(path);
        return false;
    }

    bool testPublishedReproductionConfiguration()
    {
        using spraythickness::ReproductionAlgorithmKind;
        for(const ReproductionAlgorithmKind algorithm : {
                ReproductionAlgorithmKind::Tzinava2020,
                ReproductionAlgorithmKind::Wu2020,
                ReproductionAlgorithmKind::Fuke2005,
                ReproductionAlgorithmKind::Vanerio2021,
                ReproductionAlgorithmKind::DynamicSurface2026 }) {
            const std::filesystem::path path =
                temporaryJsonPath(spraythickness::reproductionAlgorithmId(algorithm));
            robot_qt_viewer::PublishedReproductionAdapter::writeConfigurationTemplate(
                algorithm, path);
            std::ifstream input(path);
            const std::string text((std::istreambuf_iterator<char>(input)),
                std::istreambuf_iterator<char>());
            input.close();
            std::filesystem::remove(path);
            if(text.find(spraythickness::reproductionAlgorithmId(algorithm))
                == std::string::npos) {
                return false;
            }
        }
        const std::filesystem::path templatePath =
            temporaryJsonPath("wu_template");
        robot_qt_viewer::PublishedReproductionAdapter::writeConfigurationTemplate(
            ReproductionAlgorithmKind::Wu2020, templatePath);
        std::ifstream templateInput(templatePath);
        const std::string templateText((std::istreambuf_iterator<char>(templateInput)),
            std::istreambuf_iterator<char>());
        templateInput.close();
        std::filesystem::remove(templatePath);
        if(templateText.find("\"peak_cylinder_height_m\": null")
                == std::string::npos
            || templateText.find("\"reference_pose_duration_s\": null")
                == std::string::npos) {
            return false;
        }
        const auto tzinavaProfile =
            robot_qt_viewer::PublishedReproductionAdapter::inputProfile(
                ReproductionAlgorithmKind::Tzinava2020);
        const auto wuProfile =
            robot_qt_viewer::PublishedReproductionAdapter::inputProfile(
                ReproductionAlgorithmKind::Wu2020);
        if(!tzinavaProfile.requiresStlSurface
            || !tzinavaProfile.forceOriginalTrajectoryPoints
            || wuProfile.requiresStlSurface
            || !wuProfile.forceOriginalTrajectoryPoints) {
            return false;
        }

        const std::filesystem::path tzinavaPath =
            temporaryJsonPath("tzinava_complete");
        writeText(tzinavaPath, R"({
  "algorithm": "tzinava_2020",
  "beam_kind": "cylindrical_conical",
  "beam_radius_m": 0.01,
  "cylindrical_length_m": 0.0,
  "cone_half_angle_rad": 0.1,
  "gaussian_sigma_m": 0.005,
  "gaussian_radial_profile": true,
  "speed_coefficient_b": 1e-6,
  "speed_coefficient_c": 1e-9,
  "reference_spot_speed_mm_per_s": 100.0,
  "time_step_overlap_factor": 0.5,
  "stationary_time_step_s": 0.02,
  "lookup_reference_dwell_s": 1.0,
  "thickness_table": {
    "stand_off_distances_m": [0.1, 0.2],
    "impact_angles_deg": [45.0, 90.0],
    "thickness_m": [1e-6, 2e-6, 5e-7, 1e-6]
  }
})");
        robot_qt_viewer::PublishedReproductionRuntimeInputs runtimeInputs;
        runtimeInputs.modelSourcePath = "workpiece.stl";
        spraycore::SprayTool tzinavaTool;
        spraythickness::AlgorithmReproductionTask tzinavaTask;
        try {
            tzinavaTask = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                ReproductionAlgorithmKind::Tzinava2020,
                makeTriangleWorkpiece(), makeTrajectory(), tzinavaTool,
                spraythickness::TrajectorySamplingMode::ResampleByTimeStep,
                0.1, tzinavaPath, runtimeInputs);
        } catch(const std::exception& exception) {
            std::cerr << "Tzinava build failed: " << exception.what() << '\n'
                      << std::flush;
            std::filesystem::remove(tzinavaPath);
            return false;
        }
        std::filesystem::remove(tzinavaPath);
        const auto* tzinavaInput = std::get_if<
            spraythickness::published::TzinavaInputModel>(&tzinavaTask.input);
        if(tzinavaInput == nullptr
            || !tzinavaInput->objectRotationOrigin.isZero()
            || !tzinavaInput->objectRotationAxis.isApprox(Eigen::Vector3d::UnitZ())
            || tzinavaInput->objectAngularSpeedRadiansPerSecond != 0.0
            || tzinavaInput->gunTrajectory.size() != 2) {
            return false;
        }

        const std::filesystem::path missingPath = temporaryJsonPath("wu_missing");
        if(!buildFailsWith(missingPath, R"({"algorithm":"wu_2020"})",
                "peak_cylinder_height_m")) {
            return false;
        }
        const std::filesystem::path mismatchPath = temporaryJsonPath("wu_mismatch");
        if(!buildFailsWith(mismatchPath, R"({"algorithm":"fuke_2005"})",
                "wu_2020")) {
            return false;
        }
        const std::filesystem::path durationPath =
            temporaryJsonPath("wu_duration");
        writeText(durationPath, R"({
  "algorithm": "wu_2020",
  "peak_cylinder_height_m": 2e-6,
  "reference_pose_duration_s": 0.01,
  "gaussian_sigma_m": 0.003,
  "maximum_deflection_rad": 0.1,
  "ray_angular_step_rad": 0.01,
  "cylinder_radius_m": 0.00002,
  "spray_angle_rde_coefficients": [1, 0, 0, 0, 0],
  "spray_distance_rde_coefficients": [1, 0, 0, 0],
  "traverse_speed_pcf_coefficients": [1, 0]
})");
        spraythickness::AlgorithmReproductionTask wuTask;
        try {
            spraycore::SprayTool tool;
            wuTask = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                ReproductionAlgorithmKind::Wu2020,
                makeTriangleWorkpiece(), makeTrajectory(), tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                0.01, durationPath);
        } catch(const std::exception& exception) {
            std::cerr << "Wu duration calibration failed: "
                      << exception.what() << '\n';
            std::filesystem::remove(durationPath);
            return false;
        }
        std::filesystem::remove(durationPath);
        const auto* wuParameters = std::get_if<
            spraythickness::published::WuParameters>(&wuTask.parameters);
        if(wuParameters == nullptr
            || std::abs(wuParameters->referencePoseDurationSeconds - 0.01)
                > 1.0e-12) {
            return false;
        }
        return true;
    }

    bool testPlateStackSquareSurfaces()
    {
        robot_qt_viewer::PlateStackParameters parameters;
        parameters.plateSideMillimeters = 40.0;
        parameters.cellSizeMillimeters = 10.0;
        parameters.plateCount = 2;
        parameters.plateSpacingMillimeters = 15.0;
        robot_qt_viewer::PlateStackData stack;
        if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                parameters, stack)) {
            return false;
        }
        spraythickness::published::TriangleMesh raycastMesh;
        for(const sprayworkpiece::SurfaceSample& sample :
            stack.raycastWorkpiece.samples) {
            raycastMesh.vertices.push_back(sample.position);
        }
        for(std::size_t offset = 0;
            offset < stack.raycastWorkpiece.triangleIndices.size(); offset += 3) {
            raycastMesh.faces.push_back({
                stack.raycastWorkpiece.triangleIndices[offset],
                stack.raycastWorkpiece.triangleIndices[offset + 1],
                stack.raycastWorkpiece.triangleIndices[offset + 2] });
        }
        spraythickness::published::RayHit topHit;
        spraythickness::published::RayHit lowerHit;
        const bool hitTop = spraythickness::published::nearestRayHit(
            raycastMesh, Eigen::Vector3d(0.0, 0.0, 0.1),
            -Eigen::Vector3d::UnitZ(), topHit);
        const bool hitLower = spraythickness::published::nearestRayHit(
            raycastMesh, Eigen::Vector3d(0.0, 0.0, -0.005),
            -Eigen::Vector3d::UnitZ(), lowerHit);
        return hitTop && hitLower
            && std::abs(topHit.position.z()) < 1.0e-12
            && std::abs(lowerHit.position.z() + 0.015) < 1.0e-12
            && stack.workpiece.samples.size() == 50
            && stack.raycastWorkpiece.samples.size() == 8
            && stack.raycastWorkpiece.triangleIndices.size() == 12
            && stack.raycastWorkpiece.samples[0].position.isApprox(
                Eigen::Vector3d(-0.02, -0.02, 0.0))
            && stack.raycastWorkpiece.samples[4].position.isApprox(
                Eigen::Vector3d(-0.02, -0.02, -0.015))
            && stack.raycastWorkpiece.samples[7].normal.isApprox(
                Eigen::Vector3d::UnitZ());
    }

    bool testWuCumulativeDisplaySurface()
    {
        using namespace spraythickness::published;
        spraythickness::AlgorithmReproductionResult result;
        result.algorithm = spraythickness::ReproductionAlgorithmKind::Wu2020;
        WuResult wu;
        wu.substrate.vertices = {
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.1, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.1, 0.0)
        };
        wu.substrate.faces = { { 0, 1, 2 } };
        wu.depositedCylinders.push_back({
            Eigen::Vector3d(0.02, 0.02, 0.0),
            Eigen::Vector3d::UnitZ(), 0.001, 2.0e-6, 0 });
        wu.depositedCylinders.push_back({
            Eigen::Vector3d(0.02, 0.02, 2.0e-6),
            Eigen::Vector3d::UnitZ(), 0.001, 2.0e-6, 0 });
        result.nativeResult = std::move(wu);
        sprayworkpiece::WorkpieceModel workpiece = makeTriangleWorkpiece();
        workpiece.samples[0].position = Eigen::Vector3d(0.02, 0.02, 0.0);
        const std::size_t lowerOffset = workpiece.samples.size();
        for(std::size_t index = 0; index < lowerOffset; ++index) {
            auto sample = workpiece.samples[index];
            sample.position.z() = -0.015;
            workpiece.samples.push_back(sample);
        }
        workpiece.triangleIndices.insert(workpiece.triangleIndices.end(),
            { 3, 4, 5 });
        const auto display =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, workpiece, {});
        if(!display.displayModel
            || display.displayModel->subMeshes().size() != 1
            || !displayMatchesOverlay(display, *display.displayModel)) {
            return false;
        }
        const auto& geometry = display.displayModel->subMeshes()[0].geometry;
        const auto& values = display.scalarField.field.results;
        const auto enlarged =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, workpiece, {}, 100.0);
        const auto& enlargedGeometry =
            enlarged.displayModel->subMeshes()[0].geometry;
        const auto& enlargedValues = enlarged.scalarField.field.results;
        return geometry.positions.size() == 72
            && geometry.indices.size() == 294
            && values.size() == 72
            && std::abs(values[0].thickness) < 1.0e-12
            && std::abs(values[3].thickness) < 1.0e-12
            && std::abs(geometry.positions[0].z()) < 1.0e-9f
            && std::abs(geometry.positions[3].z() + 0.015f) < 1.0e-9f
            && std::abs(values[38].thickness - 2.0e-6) < 1.0e-12
            && std::abs(values[39].thickness - 2.0e-6) < 1.0e-12
            && std::abs(values[71].thickness - 4.0e-6) < 1.0e-12
            && std::abs(geometry.positions[71].z() - 4.0e-6f) < 1.0e-9f
            && std::abs(enlargedGeometry.positions[71].z() - 4.0e-4f) < 1.0e-8f
            && std::abs(enlargedValues[71].thickness
                - values[71].thickness) < 1.0e-12;
    }

    bool testWuObliqueCylinderDisplay()
    {
        using namespace spraythickness::published;
        spraythickness::AlgorithmReproductionResult result;
        result.algorithm = spraythickness::ReproductionAlgorithmKind::Wu2020;
        WuResult wu;
        wu.substrate.vertices = {
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.1, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.1, 0.0)
        };
        wu.substrate.faces = { { 0, 1, 2 } };
        const Eigen::Vector3d axis = Eigen::Vector3d(0.1, 0.0, 1.0).normalized();
        const Eigen::Vector3d center(0.02, 0.02, 0.0);
        constexpr double radius = 0.00015;
        constexpr double height = 5.0e-6;
        wu.depositedCylinders.push_back({ center, axis, radius, height, 0 });
        result.nativeResult = std::move(wu);
        const auto display =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, makeTriangleWorkpiece(), {}, 100.0);
        const auto& geometry = display.displayModel->subMeshes()[0].geometry;
        const auto& values = display.scalarField.field.results;
        const std::size_t first = makeTriangleWorkpiece().samples.size();
        const std::size_t segments = (geometry.positions.size() - first - 1) / 2;
        if(segments < 12 || values.size() != geometry.positions.size()) {
            return false;
        }
        const double physicalTop = height * axis.z();
        for(std::size_t segment = 0; segment < segments; ++segment) {
            const Eigen::Vector3f bottom = geometry.positions[first + segment];
            const Eigen::Vector3f top = geometry.positions[first + segments + segment];
            if(std::abs(bottom.z()) > 1.0e-8f
                || std::abs(top.z() - 100.0 * physicalTop) > 1.0e-7f
                || std::abs((bottom.head<2>().cast<double>()
                        - center.head<2>()).norm() - radius) > 1.0e-7
                || std::abs(values[first + segment].thickness) > 1.0e-12
                || std::abs(values[first + segments + segment].thickness
                    - physicalTop) > 1.0e-12) {
                return false;
            }
        }
        return std::abs(geometry.positions.back().z()
                - 100.0 * physicalTop) <= 1.0e-7f
            && std::abs(values.back().thickness - physicalTop) <= 1.0e-12;
    }

    bool testWuCylinderPlateDisplay()
    {
        using namespace spraythickness::published;
        robot_qt_viewer::PlateStackParameters parameters;
        parameters.plateSideMillimeters = 10.0;
        parameters.cellSizeMillimeters = 1.0;
        parameters.plateCount = 2;
        parameters.plateSpacingMillimeters = 10.0;
        robot_qt_viewer::PlateStackData stack;
        if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                parameters, stack)) {
            return false;
        }

        WuResult wu;
        for(const auto& sample : stack.raycastWorkpiece.samples) {
            wu.substrate.vertices.push_back(sample.position);
        }
        for(std::size_t offset = 0;
            offset < stack.raycastWorkpiece.triangleIndices.size(); offset += 3) {
            wu.substrate.faces.push_back({
                stack.raycastWorkpiece.triangleIndices[offset],
                stack.raycastWorkpiece.triangleIndices[offset + 1],
                stack.raycastWorkpiece.triangleIndices[offset + 2] });
        }
        RayHit hit;
        if(!nearestRayHit(wu.substrate,
                Eigen::Vector3d(0.0002, 0.0002, 0.03),
                -Eigen::Vector3d::UnitZ(), hit)) {
            return false;
        }
        wu.depositedCylinders.push_back({
            hit.position, Eigen::Vector3d::UnitZ(), 0.02e-3, 10.0e-6,
            hit.faceIndex });
        wu.depositedCylinders.push_back({
            hit.position + Eigen::Vector3d(0.0, 0.0, 10.0e-6),
            Eigen::Vector3d::UnitZ(), 0.02e-3, 10.0e-6,
            hit.faceIndex });
        spraythickness::AlgorithmReproductionResult result;
        result.algorithm = spraythickness::ReproductionAlgorithmKind::Wu2020;
        result.nativeResult = std::move(wu);
        const auto display =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, stack.workpiece, {});
        if(!display.displayModel
            || !displayMatchesOverlay(display, *display.displayModel)) {
            return false;
        }
        const auto& geometry = display.displayModel->subMeshes()[0].geometry;
        const auto& values = display.scalarField.field.results;
        const std::size_t base = stack.workpiece.samples.size();
        return geometry.positions.size() == base + 66
            && geometry.indices.size()
                == stack.workpiece.triangleIndices.size() + 288
            && values.size() == base + 66
            && std::abs(values[base + 32].thickness - 10.0e-6) < 1.0e-12
            && std::abs(values[base + 33].thickness - 10.0e-6) < 1.0e-12
            && std::abs(values[base + 65].thickness - 20.0e-6) < 1.0e-12
            && std::all_of(values.begin(), values.begin() + base,
                [](const auto& value) { return value.thickness == 0.0; });
    }

    bool testWuEmptyDepositionDisplay()
    {
        spraythickness::AlgorithmReproductionResult result;
        result.algorithm = spraythickness::ReproductionAlgorithmKind::Wu2020;
        spraythickness::published::WuResult native;
        for(const auto& sample : makeTriangleWorkpiece().samples) {
            native.substrate.vertices.push_back(sample.position);
        }
        native.substrate.faces.push_back({ 0, 1, 2 });
        result.nativeResult = std::move(native);
        const auto display =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, makeTriangleWorkpiece(), {});
        if(!display.displayModel
            || display.displayModel->subMeshes().size() != 1
            || display.scalarField.field.results.size() != 3) {
            return false;
        }
        const auto overlay = robot_qt_viewer::PaintingAnalysisMeshAdapter::makeOverlay(
            "wu", display.binding, display.scalarField);
        const Eigen::Vector3f zeroColor = overlay.colorMap.sample(
            0.0, overlay.range);
        return display.displayModel->subMeshes()[0].geometry.positions.size() == 3
            && overlay.subMeshes.size() == 1
            && overlay.subMeshes[0].values.size() == 3
            && zeroColor.allFinite()
            && std::all_of(overlay.subMeshes[0].values.begin(),
                overlay.subMeshes[0].values.end(), [](double value) {
                    return value == 0.0;
                });
    }

    bool testFukeDisabledTrajectory()
    {
        const std::filesystem::path path = temporaryJsonPath("fuke_disabled");
        writeText(path, R"({
  "algorithm": "fuke_2005",
  "reference_thickness_rate_m_per_s": 0.0001,
  "reference_distance_m": 0.1,
  "plume_exponent": 2
})");
        spraytrajectory::SprayTrajectory trajectory = makeTrajectory();
        trajectory.segments.front().sprayEnabled = false;
        spraycore::SprayTool tool;
        tool.sprayDirectionLocal = -Eigen::Vector3d::UnitZ();
        bool passed = false;
        try {
            const auto task = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Fuke2005,
                makeTriangleWorkpiece(), trajectory, tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                0.1, path);
            const auto result = spraythickness::AlgorithmReproducer::run(task);
            const auto& native = std::get<spraythickness::published::FukeResult>(
                result.nativeResult);
            passed = native.polygonThicknessMeters.size() == 1
                && native.polygonThicknessMeters.front() == 0.0;
            if(passed) {
                trajectory.segments.front().sprayEnabled = true;
                const auto activeTask =
                    robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                        spraythickness::ReproductionAlgorithmKind::Fuke2005,
                        makeTriangleWorkpiece(), trajectory, tool,
                        spraythickness::TrajectorySamplingMode::OriginalPoints,
                        0.1, path);
                const auto active = spraythickness::AlgorithmReproducer::run(
                    activeTask);
                const auto& activeNative =
                    std::get<spraythickness::published::FukeResult>(
                        active.nativeResult);
                passed = activeNative.polygonThicknessMeters.size() == 1
                    && activeNative.polygonThicknessMeters.front() > 0.0;
            }
        } catch(const std::exception& exception) {
            std::cerr << "Fuke disabled-trajectory setup: "
                      << exception.what() << '\n';
        }
        std::filesystem::remove(path);
        return passed;
    }

    bool testFukeFaceDisplay()
    {
        sprayworkpiece::WorkpieceModel workpiece = makeTriangleWorkpiece();
        workpiece.samples.push_back({ Eigen::Vector3d(0.1, 0.1, 0.0),
            Eigen::Vector3d::UnitZ() });
        workpiece.triangleIndices.insert(workpiece.triangleIndices.end(),
            { 1, 3, 2 });
        spraythickness::AlgorithmReproductionResult result;
        result.algorithm = spraythickness::ReproductionAlgorithmKind::Fuke2005;
        spraythickness::published::FukeResult native;
        native.polygonThicknessMeters = { 2.0e-6, 0.0 };
        result.nativeResult = native;
        const auto display =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                result, workpiece, {});
        if(!display.displayModel
            || display.displayModel->subMeshes().size() != 1
            || !displayMatchesOverlay(display, *display.displayModel)) {
            return false;
        }
        const auto& geometry = display.displayModel->subMeshes()[0].geometry;
        const auto& values = display.scalarField.field.results;
        return geometry.positions.size() == 6 && values.size() == 6
            && values[0].thickness == 2.0e-6
            && values[3].thickness == 0.0;
    }

    bool testEvolvedSurfaceThickness()
    {
        sprayworkpiece::WorkpieceModel workpiece = makeTriangleWorkpiece();
        spraythickness::published::TriangleMesh mesh;
        for(const auto& sample : workpiece.samples) {
            mesh.vertices.push_back(sample.position);
        }
        mesh.vertices.push_back(Eigen::Vector3d(0.05, 0.0, 0.0));
        mesh.vertices.push_back(Eigen::Vector3d(0.05, 0.0, 2.0e-6));
        mesh.faces = { { 0, 3, 2 }, { 3, 1, 2 } };
        for(const auto kind : {
                spraythickness::ReproductionAlgorithmKind::Vanerio2021,
                spraythickness::ReproductionAlgorithmKind::DynamicSurface2026 }) {
            spraythickness::AlgorithmReproductionResult result;
            result.algorithm = kind;
            if(kind == spraythickness::ReproductionAlgorithmKind::Vanerio2021) {
                spraythickness::published::VanerioResult native;
                native.evolvedStlSurface = mesh;
                native.vertexThicknessMeters = { 0.0, 0.0, 0.0, 0.0, 7.0e-6 };
                result.nativeResult = std::move(native);
            } else {
                spraythickness::published::DynamicSurfaceResult native;
                native.evolvedStlSurface = mesh;
                result.nativeResult = std::move(native);
            }
            const auto display =
                robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                    result, workpiece, {});
            if(!display.displayModel
                || !displayMatchesOverlay(display, *display.displayModel)) {
                return false;
            }
            const auto& values = display.scalarField.field.results;
            const double expected = kind
                == spraythickness::ReproductionAlgorithmKind::Vanerio2021
                ? 7.0e-6 : 2.0e-6;
            if(values.size() != 5 || std::abs(values[3].thickness) > 1.0e-12
                || std::abs(values[4].thickness - expected) > 1.0e-12) {
                return false;
            }
        }
        workpiece.samples[1].position = workpiece.samples[0].position;
        spraythickness::AlgorithmReproductionResult degenerate;
        degenerate.algorithm =
            spraythickness::ReproductionAlgorithmKind::Vanerio2021;
        spraythickness::published::VanerioResult degenerateNative;
        degenerateNative.evolvedStlSurface.vertices = {
            Eigen::Vector3d(0.0, 0.05, 2.0e-6),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.1, 0.0)
        };
        degenerateNative.evolvedStlSurface.faces.push_back({ 0, 1, 2 });
        degenerateNative.vertexThicknessMeters = { 4.0e-6, 0.0, 0.0 };
        degenerate.nativeResult = std::move(degenerateNative);
        const auto degenerateDisplay =
            robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                degenerate, workpiece, {});
        if(std::abs(degenerateDisplay.scalarField.field.results[0].thickness
                - 4.0e-6) > 1.0e-12) {
            return false;
        }
        return true;
    }

    bool testGeneratedReproductionProducesThickness()
    {
        robot_qt_viewer::PlateStackParameters plateParameters;
        plateParameters.plateSideMillimeters = 100.0;
        plateParameters.cellSizeMillimeters = 5.0;
        plateParameters.plateCount = 2;
        plateParameters.plateSpacingMillimeters = 15.0;
        robot_qt_viewer::PlateStackData stack;
        if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                plateParameters, stack)) {
            return false;
        }

        robot_qt_viewer::SimulationExperimentParameters trajectoryParameters;
        trajectoryParameters.kind =
            robot_qt_viewer::SimulationExperimentKind::LineScan;
        trajectoryParameters.plateSideMillimeters = 100.0;
        trajectoryParameters.sprayDistanceMillimeters = 120.0;
        trajectoryParameters.trajectoryOverrunMillimeters = 5.0;
        trajectoryParameters.incidenceAngleDegrees = 90.0;
        trajectoryParameters.scanSpeedMillimetersPerSecond = 500.0;
        trajectoryParameters.scanPassCount = 1;
        trajectoryParameters.trajectoryPointIntervalSeconds = 0.001;
        trajectoryParameters.scanStartXMillimeters = -50.0;
        trajectoryParameters.scanStartYMillimeters = 0.0;
        trajectoryParameters.scanEndXMillimeters = 50.0;
        trajectoryParameters.scanEndYMillimeters = 0.0;
        spraytrajectory::SprayTrajectory trajectory;
        if(!robot_qt_viewer::SimulationExperiment::buildTrajectory(
                trajectoryParameters, trajectory)) {
            return false;
        }

        const std::filesystem::path configurationPath =
            temporaryJsonPath("fuke_generated_scene");
        writeText(configurationPath, R"({
  "algorithm": "fuke_2005",
  "reference_thickness_rate_m_per_s": 0.0001,
  "reference_distance_m": 0.12,
  "plume_exponent": 2
})");
        bool passed = false;
        try {
            spraycore::SprayTool tool;
            tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
            const auto task = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Fuke2005,
                stack.workpiece,
                trajectory,
                tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                trajectoryParameters.trajectoryPointIntervalSeconds,
                configurationPath);
            const auto result = spraythickness::AlgorithmReproducer::run(task);
            const auto display =
                robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                    result, stack.workpiece, {});
            std::cout << "Default-speed Fuke range (um): "
                      << display.scalarField.metrics.minThickness * 1.0e6
                      << " to "
                      << display.scalarField.metrics.maxThickness * 1.0e6 << '\n';
            passed = display.scalarField.metrics.maxThickness > 0.0
                && display.scalarField.metrics.maxThickness
                    > display.scalarField.metrics.minThickness;
        } catch(const std::exception& exception) {
            std::cerr << "Generated reproduction setup: "
                      << exception.what() << '\n';
        }
        std::filesystem::remove(configurationPath);
        if(!passed) {
            return false;
        }

        const std::filesystem::path tzinavaConfigurationPath =
            temporaryJsonPath("tzinava_generated_scene");
        writeText(tzinavaConfigurationPath, R"({
  "algorithm": "tzinava_2020",
  "beam_kind": "cylindrical_conical",
  "beam_radius_m": 0.0027,
  "cylindrical_length_m": 0.0,
  "cone_half_angle_rad": 0.067,
  "gaussian_sigma_m": 0.0036,
  "gaussian_radial_profile": true,
  "speed_coefficient_b": 0.0000048,
  "speed_coefficient_c": 0.00035,
  "reference_spot_speed_mm_per_s": 502.0,
  "time_step_overlap_factor": 0.5,
  "stationary_time_step_s": 0.02,
  "lookup_reference_dwell_s": 1.0,
  "thickness_table": {
    "stand_off_distances_m": [0.05, 0.2],
    "impact_angles_deg": [0.0, 90.0],
    "thickness_m": [0.0, 0.0001, 0.0, 0.0001]
  }
})");
        passed = false;
        try {
            spraycore::SprayTool tool;
            tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
            robot_qt_viewer::PublishedReproductionRuntimeInputs runtimeInputs;
            runtimeInputs.modelSourcePath = "generated-plate-stack.stl";
            const auto task = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Tzinava2020,
                stack.workpiece,
                trajectory,
                tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                trajectoryParameters.trajectoryPointIntervalSeconds,
                tzinavaConfigurationPath,
                runtimeInputs);
            const auto result = spraythickness::AlgorithmReproducer::run(task);
            const auto display =
                robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                    result, stack.workpiece, {});
            std::cout << "Default-speed Tzinava range (um): "
                      << display.scalarField.metrics.minThickness * 1.0e6
                      << " to "
                      << display.scalarField.metrics.maxThickness * 1.0e6 << '\n';
            passed = display.scalarField.metrics.maxThickness > 0.0
                && display.scalarField.metrics.maxThickness
                    > display.scalarField.metrics.minThickness;
        } catch(const std::exception& exception) {
            std::cerr << "Generated Tzinava reproduction setup: "
                      << exception.what() << '\n';
        }
        std::filesystem::remove(tzinavaConfigurationPath);
        return passed;
    }

    bool testGeneratedVanerioDeposition()
    {
        robot_qt_viewer::PlateStackParameters plateParameters;
        plateParameters.plateSideMillimeters = 100.0;
        plateParameters.cellSizeMillimeters = 1.0;
        plateParameters.plateCount = 2;
        plateParameters.plateSpacingMillimeters = 20.0;
        robot_qt_viewer::PlateStackData stack;
        if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                plateParameters, stack)) {
            return false;
        }
        robot_qt_viewer::SimulationExperimentParameters parameters;
        parameters.kind = robot_qt_viewer::SimulationExperimentKind::LineScan;
        parameters.plateSideMillimeters = 100.0;
        parameters.sprayDistanceMillimeters = 120.0;
        parameters.trajectoryOverrunMillimeters = 20.0;
        parameters.incidenceAngleDegrees = 90.0;
        parameters.scanSpeedMillimetersPerSecond = 500.0;
        parameters.scanPassCount = 1;
        parameters.trajectoryPointIntervalSeconds = 0.001;
        parameters.scanStartXMillimeters = -50.0;
        parameters.scanEndXMillimeters = 50.0;
        spraytrajectory::SprayTrajectory trajectory;
        if(!robot_qt_viewer::SimulationExperiment::buildTrajectory(
                parameters, trajectory)) {
            return false;
        }
        const std::filesystem::path calibration =
            temporaryJsonPath("vanerio_generated_scene");
        writeText(calibration, R"({
  "algorithm": "vanerio_2021",
  "growth_rate_coefficient_m_per_s": 0.000148831322775,
  "jet_radius_m": 0.010768971450986448,
  "jet_shape_coefficient_k2": 4.36,
  "maximum_mesh_edge_m": 0.001,
  "shadow_grid_step_m": 0.0005,
  "deposition_efficiency_by_tangent_angle": {
    "arguments": [0.0, 0.17632698070846498, 0.36397023426620234,
      0.5773502691896257, 0.8390996311772799, 1.19175359259421,
      1.7320508075688767, 2.7474774194546216, 5.671281819617707,
      57.289961630759144, 572.9572133543032, 5729.577893128937,
      1000000000000.0],
    "values": [1.0, 0.9407141792249564, 0.8797586543487659,
      0.8171334253714285, 0.7528384922929442, 0.686873855113313,
      0.619239513832535, 0.5499354684506098, 0.478961718967538,
      0.4136577474262926, 0.40705296495446186, 0.4063917428541019,
      0.4063182653833191]
  },
  "deposition_efficiency_by_distance": {
    "arguments": [0.05, 0.075, 0.1, 0.12, 0.15, 0.2, 0.3,
      0.4, 0.5, 0.55, 0.75, 1.0],
    "values": [0.9784536100969946, 0.9784536127849371,
      1.0563012804778658, 1.0, 0.9784536218051285,
      0.9784536100969946, 0.9784536100969946, 0.9784536100969946,
      0.9784536100969946, 0.9784536100969946, 0.9784536100969946,
      0.9784536100969946]
  },
  "profile_stretch_by_distance": {
    "arguments": [0.05, 0.075, 0.1, 0.12, 0.15, 0.2,
      0.3, 0.4, 0.5, 0.55, 0.75, 1.0],
    "values": [2.4, 1.6, 1.2, 1.0, 0.8, 0.6,
      0.4, 0.3, 0.24, 0.21818181818181814, 0.16, 0.12]
  }
})");
        bool passed = false;
        try {
            spraycore::SprayTool tool;
            tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
            robot_qt_viewer::PublishedReproductionRuntimeInputs runtimeInputs;
            runtimeInputs.modelSourcePath = "generated-plate-stack.stl";
            const auto task = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Vanerio2021,
                stack.workpiece, trajectory, tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                parameters.trajectoryPointIntervalSeconds, calibration,
                runtimeInputs);
            const auto result = spraythickness::AlgorithmReproducer::run(task);
            const auto& native =
                std::get<spraythickness::published::VanerioResult>(
                    result.nativeResult);
            const auto display =
                robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                    result, stack.workpiece, {});
            std::cout << "Generated Vanerio contributions: "
                      << native.statistics.candidatePairCount
                      << ", displayed maximum (um): "
                      << display.scalarField.metrics.maxThickness * 1.0e6
                      << '\n';
            passed = native.statistics.candidatePairCount > 0
                && display.scalarField.metrics.maxThickness > 1.0e-9;
        } catch(const std::exception& error) {
            std::cerr << "Generated Vanerio reproduction: "
                      << error.what() << '\n';
        }
        std::filesystem::remove(calibration);
        return passed;
    }

    std::filesystem::path calibrationDirectory()
    {
        for(std::filesystem::path directory : {
                std::filesystem::current_path(),
                std::filesystem::path(__FILE__).parent_path() }) {
            while(!directory.empty()) {
                const auto candidate = directory / "data/ChenYu/CalibrationFile";
                if(std::filesystem::is_regular_file(candidate / "wu_2020.json")) {
                    return candidate;
                }
                if(directory == directory.parent_path()) {
                    break;
                }
                directory = directory.parent_path();
            }
        }
        return {};
    }

    bool testPublishedMethodsShareGeneratedGrid()
    {
        using Kind = spraythickness::ReproductionAlgorithmKind;
        const std::filesystem::path directory = calibrationDirectory();
        if(directory.empty()) {
            return false;
        }
        robot_qt_viewer::PlateStackParameters parameters;
        parameters.plateSideMillimeters = 10.0;
        parameters.cellSizeMillimeters = 5.0;
        parameters.plateCount = 2;
        parameters.plateSpacingMillimeters = 15.0;
        robot_qt_viewer::PlateStackData stack;
        if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                parameters, stack)) {
            return false;
        }
        const std::size_t vertices = stack.workpiece.samples.size();
        const std::size_t faces = stack.workpiece.triangleIndices.size() / 3;
        if(vertices != 18 || faces != 16) {
            return false;
        }
        spraycore::SprayTool tool;
        tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
        robot_qt_viewer::PublishedReproductionRuntimeInputs runtime;
        runtime.modelSourcePath = "algorithm-reproduction-plate-stack.stl";
        runtime.generatedPlateStack = true;
        const auto trajectory = makeTrajectory();
        for(const Kind kind : { Kind::Tzinava2020, Kind::Wu2020,
                Kind::Fuke2005, Kind::Vanerio2021,
                Kind::DynamicSurface2026 }) {
            const auto configuration = directory /
                (std::string(spraythickness::reproductionAlgorithmId(kind))
                    + ".json");
            const auto task = robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                kind, stack.workpiece, trajectory, tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                1.0, configuration, runtime);
            std::size_t taskVertices = 0;
            std::size_t taskFaces = 0;
            switch(kind) {
            case Kind::Tzinava2020: {
                const auto& mesh = std::get<spraythickness::published::TzinavaInputModel>(
                    task.input).initialMesh;
                taskVertices = mesh.vertices.size();
                taskFaces = mesh.faces.size();
                break;
            }
            case Kind::Wu2020: {
                const auto& mesh = std::get<spraythickness::published::WuInputModel>(
                    task.input).substrate;
                taskVertices = mesh.vertices.size();
                taskFaces = mesh.faces.size();
                break;
            }
            case Kind::Fuke2005: {
                const auto& input = std::get<spraythickness::published::FukeInputModel>(
                    task.input);
                taskVertices = input.vertices.size();
                taskFaces = input.polygons.size();
                break;
            }
            case Kind::Vanerio2021: {
                const auto& mesh = std::get<spraythickness::published::VanerioInputModel>(
                    task.input).initialStlSurface;
                taskVertices = mesh.vertices.size();
                taskFaces = mesh.faces.size();
                break;
            }
            case Kind::DynamicSurface2026: {
                const auto& mesh = std::get<
                    spraythickness::published::DynamicSurfaceInputModel>(
                    task.input).initialStlSurface;
                taskVertices = mesh.vertices.size();
                taskFaces = mesh.faces.size();
                break;
            }
            case Kind::CurrentMethod:
                return false;
            }
            if(taskVertices != vertices || taskFaces != faces) {
                return false;
            }
        }
        return true;
    }

    bool testRecommendedGeneratedScene(const std::string& selectedAlgorithm)
    {
        using Kind = spraythickness::ReproductionAlgorithmKind;
        const std::filesystem::path directory = calibrationDirectory();
        if(directory.empty()) {
            std::cerr << "Calibration directory not found.\n";
            return false;
        }
        const std::vector<Kind> algorithms = {
            Kind::Tzinava2020, Kind::Wu2020, Kind::Fuke2005,
            Kind::Vanerio2021, Kind::DynamicSurface2026
        };
        for(const Kind kind : algorithms) {
            const std::string id = spraythickness::reproductionAlgorithmId(kind);
            if(!selectedAlgorithm.empty() && id != selectedAlgorithm) {
                continue;
            }
            try {
                const auto calibration = directory / (id + ".json");
                const auto preset =
                    robot_qt_viewer::PublishedReproductionAdapter::trajectoryPreset(
                        kind, calibration);
                if(!preset) {
                    throw std::runtime_error("Missing verified line-scan preset.");
                }
                robot_qt_viewer::PlateStackParameters plate;
                plate.plateSideMillimeters = preset->plateSideMillimeters;
                plate.cellSizeMillimeters = preset->cellSizeMillimeters;
                plate.plateCount = preset->plateCount;
                plate.plateSpacingMillimeters = preset->plateSpacingMillimeters;
                robot_qt_viewer::PlateStackData stack;
                if(!robot_qt_viewer::SimulationExperiment::buildPlateStack(
                        plate, stack)) {
                    throw std::runtime_error("Failed to build the preset scene.");
                }
                robot_qt_viewer::SimulationExperimentParameters trajectoryParameters;
                trajectoryParameters.kind =
                    robot_qt_viewer::SimulationExperimentKind::LineScan;
                trajectoryParameters.plateSideMillimeters =
                    preset->plateSideMillimeters;
                trajectoryParameters.sprayDistanceMillimeters =
                    preset->sprayDistanceMillimeters;
                trajectoryParameters.incidenceAngleDegrees =
                    preset->incidenceAngleDegrees;
                trajectoryParameters.scanSpeedMillimetersPerSecond =
                    preset->scanSpeedMillimetersPerSecond;
                trajectoryParameters.trajectoryPointIntervalSeconds =
                    preset->pointIntervalSeconds;
                trajectoryParameters.trajectoryOverrunMillimeters =
                    preset->overrunMillimeters;
                trajectoryParameters.scanPassCount = preset->scanPassCount;
                trajectoryParameters.scanStartXMillimeters =
                    -0.5 * preset->plateSideMillimeters;
                trajectoryParameters.scanEndXMillimeters =
                    0.5 * preset->plateSideMillimeters;
                spraytrajectory::SprayTrajectory trajectory;
                if(!robot_qt_viewer::SimulationExperiment::buildTrajectory(
                        trajectoryParameters, trajectory)) {
                    throw std::runtime_error("Failed to build the preset trajectory.");
                }
                spraycore::SprayTool tool;
                tool.sprayDirectionLocal = Eigen::Vector3d::UnitZ();
                tool.powderFeedDirectionLocal = Eigen::Vector3d::UnitY();
                robot_qt_viewer::PublishedReproductionRuntimeInputs runtime;
                runtime.modelSourcePath = "algorithm-reproduction-plate-stack.stl";
                const bool squareSurface = kind == Kind::Wu2020
                    || kind == Kind::DynamicSurface2026;
                runtime.generatedPlateStack = squareSurface;
                const auto& substrate = squareSurface
                    ? stack.raycastWorkpiece : stack.workpiece;
                const auto task =
                    robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                        kind, substrate, trajectory, tool,
                        spraythickness::TrajectorySamplingMode::OriginalPoints,
                        preset->pointIntervalSeconds, calibration, runtime);
                const auto result = spraythickness::AlgorithmReproducer::run(task);
                if(kind == Kind::Wu2020) {
                    const auto& cylinders = std::get<
                        spraythickness::published::WuResult>(result.nativeResult)
                        .depositedCylinders;
                    double middleHeight = 0.0;
                    double edgeHeight = 0.0;
                    double middleSum = 0.0;
                    double edgeSum = 0.0;
                    std::size_t middleCount = 0;
                    std::size_t edgeCount = 0;
                    for(const auto& cylinder : cylinders) {
                        if(cylinder.substrateFaceIndex >= 2) {
                            continue;
                        }
                        const double top = (cylinder.baseCenter
                            + cylinder.heightMeters
                                * cylinder.growthDirection.normalized()).z();
                        if(std::abs(cylinder.baseCenter.x()) <= 0.005) {
                            middleHeight = std::max(middleHeight, top);
                            middleSum += top;
                            ++middleCount;
                        } else if(std::abs(cylinder.baseCenter.x()) >= 0.045
                            && std::abs(cylinder.baseCenter.x()) <= 0.05) {
                            edgeHeight = std::max(edgeHeight, top);
                            edgeSum += top;
                            ++edgeCount;
                        }
                    }
                    std::cout << "Wu cylinders=" << cylinders.size()
                              << ", native middle/edge maximum (um)="
                              << middleHeight * 1.0e6 << '/'
                              << edgeHeight * 1.0e6
                              << ", middle/edge average (um)="
                              << (middleCount > 0 ? middleSum / middleCount : 0.0)
                                  * 1.0e6 << '/'
                              << (edgeCount > 0 ? edgeSum / edgeCount : 0.0)
                                  * 1.0e6 << '\n';
                    if(middleHeight <= 0.0
                        || middleCount == 0 || edgeCount == 0
                        || edgeHeight > 1.5 * middleHeight
                        || edgeSum / edgeCount > 1.1 * middleSum / middleCount) {
                        return false;
                    }
                }
                const auto display =
                    robot_qt_viewer::PublishedReproductionDisplayAdapter::build(
                        result, stack.workpiece, {});
                const auto& values = display.scalarField.field.results;
                const std::size_t nonzero = static_cast<std::size_t>(
                    std::count_if(values.begin(), values.end(), [](const auto& value) {
                        return std::isfinite(value.thickness)
                            && value.thickness > 0.0;
                    }));
                std::cout << id << ": poses="
                          << trajectory.flattenedPoints().size()
                          << ", display points=" << values.size()
                          << ", nonzero=" << nonzero
                          << ", max um="
                          << display.scalarField.metrics.maxThickness * 1.0e6
                          << '\n';
                if(!display.displayModel || nonzero == 0
                    || !std::isfinite(display.scalarField.metrics.maxThickness)) {
                    return false;
                }
                if(kind == Kind::Wu2020) {
                    const auto& cylinders = std::get<
                        spraythickness::published::WuResult>(result.nativeResult)
                        .depositedCylinders;
                    const std::size_t expectedVertices =
                        stack.workpiece.samples.size() + 33 * cylinders.size();
                    const auto& geometry =
                        display.displayModel->subMeshes()[0].geometry;
                    if(values.size() != expectedVertices
                        || geometry.positions.size() != expectedVertices
                        || geometry.indices.size()
                            != stack.workpiece.triangleIndices.size()
                                + 144 * cylinders.size()) {
                        return false;
                    }
                }
            } catch(const std::exception& error) {
                std::cerr << id << ": " << error.what() << '\n';
                return false;
            }
        }
        return true;
    }
}

int main(int argc, char** argv)
{
    if(argc == 2 && std::string(argv[1]) == "--shared-grid-only") {
        return testPublishedMethodsShareGeneratedGrid() ? 0 : 18;
    }
    if(argc == 2 && std::string(argv[1]) == "--line-scan-only") {
        return testLineScanOverrun()
            && testLineScanUniformSamplingAcrossPlateEdges() ? 0 : 17;
    }
    if(argc == 2 && std::string(argv[1]) == "--wu-display-only") {
        return testWuCumulativeDisplaySurface()
            && testWuObliqueCylinderDisplay()
            && testWuCylinderPlateDisplay()
            && testWuEmptyDepositionDisplay() ? 0 : 15;
    }
    if(argc == 2 && std::string(argv[1]) == "--vanerio-only") {
        return testGeneratedVanerioDeposition() ? 0 : 13;
    }
    if(argc >= 2 && std::string(argv[1]) == "--recommended-only") {
        return testRecommendedGeneratedScene(argc >= 3 ? argv[2] : "") ? 0 : 14;
    }
    assetcore::ModelDesc model;
    assetcore::SubMeshDesc first;
    first.geometry.positions = {
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(1.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, 1.0f, 0.0f)
    };
    first.geometry.normals = {
        Eigen::Vector3f::UnitY(),
        Eigen::Vector3f::UnitY(),
        Eigen::Vector3f::UnitY()
    };
    first.geometry.indices = { 0, 1, 2 };
    model.addSubMesh(first);

    assetcore::SubMeshDesc second;
    second.geometry.positions = { Eigen::Vector3f(0.0f, 0.0f, 1.0f) };
    model.addSubMesh(second);

    Eigen::Isometry3d worldFromModel = Eigen::Isometry3d::Identity();
    worldFromModel.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
    const robot_qt_viewer::PaintingAnalysisMeshData mesh =
        robot_qt_viewer::PaintingAnalysisMeshAdapter::build(
            model, "mesh", "mesh.stl", worldFromModel);
    if(mesh.workpiece.sampleCount() != 4 ||
        mesh.workpiece.triangleIndices.size() != 3 ||
        !mesh.workpiece.samples[0].position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)) ||
        mesh.binding.sampleIndicesBySubMesh.size() != 2 ||
        mesh.binding.sampleIndicesBySubMesh[0].size() != 3 ||
        mesh.binding.sampleIndicesBySubMesh[1].size() != 1 ||
        mesh.warnings.empty()) {
        std::cerr << "Mesh sampling or binding contract failed.\n";
        return 1;
    }

    spraythickness::ThicknessPredictionResult prediction;
    prediction.field.resizeFromWorkpiece(mesh.workpiece);
    prediction.field.results[0].thickness = 1.0;
    prediction.field.results[1].thickness = 2.0;
    prediction.field.results[2].thickness = 3.0;
    prediction.field.results[3].thickness = 4.0;
    prediction.metrics.minThickness = 1.0;
    prediction.metrics.maxThickness = 3.0;
    const smrobot::visualization::SurfaceScalarOverlay overlay =
        robot_qt_viewer::PaintingAnalysisMeshAdapter::makeOverlay(
            "object",
            mesh.binding,
            prediction);
    if(overlay.objectId != "object" || overlay.subMeshes.size() != 2 ||
        overlay.subMeshes[0].values.size() != 3 ||
        overlay.subMeshes[1].values.size() != 1 ||
        overlay.subMeshes[1].values[0] != 4.0) {
        std::cerr << "Scalar overlay projection contract failed.\n";
        return 2;
    }
    if(!testPublishedReproductionConfiguration()) {
        std::cerr << "Published reproduction configuration contract failed.\n";
        return 3;
    }
    if(!testLineScanOverrun()) {
        std::cerr << "Line scan overrun symmetry contract failed.\n";
        return 4;
    }
    if(!testLineScanUniformSamplingAcrossPlateEdges()) {
        std::cerr << "Line scan plate-edge sampling contract failed.\n";
        return 17;
    }
    if(!testPlateStackSquareSurfaces()) {
        std::cerr << "Plate stack square-surface contract failed.\n";
        return 5;
    }
    if(!testPublishedMethodsShareGeneratedGrid()) {
        std::cerr << "Published methods shared-grid contract failed.\n";
        return 18;
    }
    if(!testWuCumulativeDisplaySurface()) {
        std::cerr << "Wu cumulative display-surface contract failed.\n";
        return 6;
    }
    if(!testWuCylinderPlateDisplay()) {
        std::cerr << "Wu cylinder-display contract failed.\n";
        return 15;
    }
    if(!testWuObliqueCylinderDisplay()) {
        std::cerr << "Wu oblique cylinder display failed.\n";
        return 16;
    }
    if(!testFukeDisabledTrajectory()) {
        std::cerr << "Fuke disabled-trajectory contract failed.\n";
        return 7;
    }
    if(!testFukeFaceDisplay()) {
        std::cerr << "Fuke per-face display contract failed.\n";
        return 8;
    }
    if(!testEvolvedSurfaceThickness()) {
        std::cerr << "Evolved-surface thickness contract failed.\n";
        return 9;
    }
    if(!testWuEmptyDepositionDisplay()) {
        std::cerr << "Wu empty-deposition display contract failed.\n";
        return 10;
    }
    if(!testTzinavaAndCurrentDisplay()) {
        std::cerr << "Tzinava/current display binding contract failed.\n";
        return 11;
    }
    if(!testGeneratedReproductionProducesThickness()) {
        std::cerr << "Generated reproduction thickness contract failed.\n";
        return 12;
    }
    if(!testGeneratedVanerioDeposition()) {
        std::cerr << "Generated Vanerio deposition contract failed.\n";
        return 13;
    }
    return 0;
}
