#include <PaintingAnalysisMeshAdapter.h>
#include <PublishedReproductionAdapter.h>

#include <chrono>
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

    bool buildFailsWith(const std::filesystem::path& path,
        const std::string& json, const std::string& expected)
    {
        writeText(path, json);
        try {
            spraycore::SprayTool tool;
            robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                spraythickness::ReproductionAlgorithmKind::Tanaka2024,
                makeWorkpiece(), makeTrajectory(), tool,
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
                ReproductionAlgorithmKind::Tanaka2024,
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
            temporaryJsonPath("tanaka_template");
        robot_qt_viewer::PublishedReproductionAdapter::writeConfigurationTemplate(
            ReproductionAlgorithmKind::Tanaka2024, templatePath);
        std::ifstream templateInput(templatePath);
        const std::string templateText((std::istreambuf_iterator<char>(templateInput)),
            std::istreambuf_iterator<char>());
        templateInput.close();
        std::filesystem::remove(templatePath);
        if(templateText.find("\"paint_discharge_m3_per_s\": null")
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
            || !tzinavaProfile.hasObjectRotationInput
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
        runtimeInputs.objectRotationOriginMeters = Eigen::Vector3d(1.0, 2.0, 3.0);
        runtimeInputs.objectRotationAxis = Eigen::Vector3d::UnitY();
        runtimeInputs.objectAngularSpeedRadiansPerSecond = 0.25;
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
            || !tzinavaInput->objectRotationOrigin.isApprox(
                runtimeInputs.objectRotationOriginMeters)
            || !tzinavaInput->objectRotationAxis.isApprox(
                runtimeInputs.objectRotationAxis)
            || tzinavaInput->objectAngularSpeedRadiansPerSecond != 0.25
            || tzinavaInput->gunTrajectory.size() != 2) {
            return false;
        }

        const std::filesystem::path missingPath = temporaryJsonPath("tanaka_missing");
        if(!buildFailsWith(missingPath, R"({"algorithm":"tanaka_2024"})",
                "paint_discharge_m3_per_s")) {
            return false;
        }
        const std::filesystem::path mismatchPath = temporaryJsonPath("tanaka_mismatch");
        if(!buildFailsWith(mismatchPath, R"({"algorithm":"wu_2020"})",
                "tanaka_2024")) {
            return false;
        }

        const std::filesystem::path completePath = temporaryJsonPath("tanaka_complete");
        writeText(completePath, R"({
  "algorithm": "tanaka_2024",
  "paint_discharge_m3_per_s": 1e-9,
  "reference_distance_m": 0.1,
  "reference_sigma_x_m": 0.01,
  "reference_sigma_y_m": 0.02,
  "distance_exponent": 1.0,
  "incidence_exponent": 1.0
})");
        spraycore::SprayTool tool;
        const auto task =
            robot_qt_viewer::PublishedReproductionAdapter::buildTask(
                ReproductionAlgorithmKind::Tanaka2024,
                makeWorkpiece(), makeTrajectory(), tool,
                spraythickness::TrajectorySamplingMode::OriginalPoints,
                0.1, completePath);
        std::filesystem::remove(completePath);
        const auto* input = std::get_if<
            spraythickness::published::TanakaInputModel>(&task.input);
        const auto* parameters = std::get_if<
            spraythickness::published::TanakaParameters>(&task.parameters);
        return input != nullptr && parameters != nullptr
            && input->targetPoints.size() == 1
            && input->sprayPoses.size() == 2
            && parameters->paintDischargeCubicMetersPerSecond == 1.0e-9;
    }
}

int main()
{
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
    return 0;
}
