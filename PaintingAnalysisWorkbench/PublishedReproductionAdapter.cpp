#include "PublishedReproductionAdapter.h"

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace robot_qt_viewer
{
    namespace
    {
        using Json = nlohmann::json;
        namespace published = spraythickness::published;

        const Json& required(const Json& parent, const char* key)
        {
            const auto found = parent.find(key);
            if(found == parent.end() || found->is_null()) {
                throw std::invalid_argument(
                    std::string("Calibration file is missing required key: ") + key);
            }
            return *found;
        }

        template<typename Value>
        Value requiredValue(const Json& parent, const char* key)
        {
            try {
                return required(parent, key).get<Value>();
            } catch(const nlohmann::json::exception&) {
                throw std::invalid_argument(
                    std::string("Calibration key has the wrong type: ") + key);
            }
        }

        published::TabulatedCurve curve(const Json& parent, const char* key)
        {
            const Json& value = required(parent, key);
            published::TabulatedCurve result;
            result.arguments = requiredValue<std::vector<double>>(
                value, "arguments");
            result.values = requiredValue<std::vector<double>>(value, "values");
            result.validate(key);
            return result;
        }

        published::Polynomial polynomial(
            const Json& parent, const char* key)
        {
            published::Polynomial result;
            result.coefficients = requiredValue<std::vector<double>>(parent, key);
            return result;
        }

        Json readConfiguration(const std::filesystem::path& path,
            spraythickness::ReproductionAlgorithmKind algorithm)
        {
            if(path.empty()) {
                throw std::invalid_argument(
                    "The selected published method requires a calibration JSON file.");
            }
            std::ifstream input(path);
            if(!input) {
                throw std::invalid_argument(
                    "The calibration JSON file cannot be opened: "
                    + path.generic_u8string());
            }
            Json root;
            try {
                input >> root;
            } catch(const nlohmann::json::exception& exception) {
                throw std::invalid_argument(
                    std::string("Invalid calibration JSON: ") + exception.what());
            }
            const std::string expected =
                spraythickness::reproductionAlgorithmId(algorithm);
            if(requiredValue<std::string>(root, "algorithm") != expected) {
                throw std::invalid_argument(
                    "Calibration file algorithm must be '" + expected + "'.");
            }
            return root;
        }

        Json curveTemplate()
        {
            return { { "arguments", { nullptr } },
                { "values", { nullptr } } };
        }

        Json configurationTemplate(
            spraythickness::ReproductionAlgorithmKind algorithm)
        {
            Json result{ { "algorithm",
                spraythickness::reproductionAlgorithmId(algorithm) } };
            switch(algorithm) {
            case spraythickness::ReproductionAlgorithmKind::Tanaka2024:
                result.update({
                    { "paint_discharge_m3_per_s", nullptr },
                    { "reference_distance_m", nullptr },
                    { "reference_sigma_x_m", nullptr },
                    { "reference_sigma_y_m", nullptr },
                    { "distance_exponent", nullptr },
                    { "incidence_exponent", nullptr }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::Tzinava2020:
                result.update({
                    { "beam_kind", nullptr },
                    { "beam_radius_m", nullptr },
                    { "cylindrical_length_m", nullptr },
                    { "cone_half_angle_rad", nullptr },
                    { "gaussian_sigma_m", nullptr },
                    { "gaussian_radial_profile", nullptr },
                    { "speed_coefficient_b", nullptr },
                    { "speed_coefficient_c", nullptr },
                    { "reference_spot_speed_mm_per_s", nullptr },
                    { "time_step_overlap_factor", nullptr },
                    { "stationary_time_step_s", nullptr },
                    { "lookup_reference_dwell_s", nullptr },
                    { "thickness_table", {
                        { "stand_off_distances_m", { nullptr } },
                        { "impact_angles_deg", { nullptr } },
                        { "thickness_m", { nullptr } }
                    } }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::Wu2020:
                result.update({
                    { "peak_cylinder_height_m", nullptr },
                    { "gaussian_sigma_m", nullptr },
                    { "maximum_deflection_rad", nullptr },
                    { "ray_angular_step_rad", nullptr },
                    { "cylinder_radius_m", nullptr },
                    { "spray_angle_rde_coefficients", { nullptr } },
                    { "spray_distance_rde_coefficients", { nullptr } },
                    { "traverse_speed_pcf_coefficients", { nullptr } }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::Fuke2005:
                result.update({
                    { "reference_thickness_rate_m_per_s", nullptr },
                    { "reference_distance_m", nullptr },
                    { "plume_exponent", nullptr }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::Vanerio2021:
                result.update({
                    { "growth_rate_coefficient_m_per_s", nullptr },
                    { "jet_radius_m", nullptr },
                    { "jet_shape_coefficient_k2", nullptr },
                    { "maximum_mesh_edge_m", nullptr },
                    { "shadow_grid_step_m", nullptr },
                    { "deposition_efficiency_by_tangent_angle", curveTemplate() },
                    { "deposition_efficiency_by_distance", curveTemplate() },
                    { "profile_stretch_by_distance", curveTemplate() }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::DynamicSurface2026:
                result.update({
                    { "equivalent_particle_diameter_m", nullptr },
                    { "deposited_cylinder_radius_m", nullptr },
                    { "equivalent_particle_rate_per_s", nullptr },
                    { "angular_mean_rad", nullptr },
                    { "angular_sigma_rad", nullptr },
                    { "maximum_ejection_angle_rad", nullptr },
                    { "polar_bin_count", nullptr },
                    { "deposition_batch_duration_s", nullptr },
                    { "random_seed", nullptr },
                    { "relative_build_up_by_inclination_rad", curveTemplate() },
                    { "voxel_leaf_m", nullptr },
                    { "uniform_sampling_radius_m", nullptr },
                    { "normal_search_radius_m", nullptr },
                    { "normal_maximum_neighbors", nullptr },
                    { "boundary_gradient_threshold_rad_per_m", nullptr },
                    { "dbscan_radius_m", nullptr },
                    { "dbscan_minimum_points", nullptr },
                    { "alpha_shape_radius_m", nullptr },
                    { "poisson_depth", nullptr },
                    { "maximum_hole_boundary_edges", nullptr },
                    { "smoothing_iterations", nullptr },
                    { "smoothing_relaxation", nullptr }
                });
                break;
            case spraythickness::ReproductionAlgorithmKind::CurrentMethod:
                throw std::invalid_argument(
                    "The current GPU method does not require a calibration template.");
            }
            return result;
        }

        published::TriangleMesh triangleMesh(
            const sprayworkpiece::WorkpieceModel& workpiece)
        {
            published::TriangleMesh result;
            result.vertices.reserve(workpiece.samples.size());
            for(const sprayworkpiece::SurfaceSample& sample : workpiece.samples) {
                result.vertices.push_back(sample.position);
            }
            for(std::size_t offset = 0;
                offset + 2 < workpiece.triangleIndices.size(); offset += 3) {
                result.faces.push_back({ workpiece.triangleIndices[offset],
                    workpiece.triangleIndices[offset + 1],
                    workpiece.triangleIndices[offset + 2] });
            }
            result.validate();
            return result;
        }

        std::vector<spraytrajectory::SprayTrajectorySample> samples(
            const spraytrajectory::SprayTrajectory& trajectory,
            spraythickness::TrajectorySamplingMode samplingMode,
            double timeStepSeconds)
        {
            if(samplingMode
                == spraythickness::TrajectorySamplingMode::ResampleByTimeStep) {
                return spraytrajectory::SprayTrajectorySampler::sample(
                    trajectory, timeStepSeconds);
            }
            return spraytrajectory::SprayTrajectorySampler::originalSamples(
                trajectory);
        }

        std::vector<published::SprayPose> sprayPoses(
            const std::vector<spraytrajectory::SprayTrajectorySample>& values,
            const spraycore::SprayTool& tool)
        {
            std::vector<published::SprayPose> result;
            result.reserve(values.size());
            for(std::size_t index = 0; index < values.size(); ++index) {
                const Eigen::Isometry3d toolPose =
                    values[index].tcpPose * tool.T_link_tool;
                published::SprayPose pose;
                pose.timeSeconds = values[index].time;
                pose.position = toolPose.translation();
                pose.axis = tool.worldSprayDirection(toolPose).normalized();
                pose.sprayEnabled = values[index].sprayEnabled;
                if(index + 1 < values.size()) {
                    const double duration =
                        values[index + 1].time - values[index].time;
                    if(duration > 0.0) {
                        pose.linearVelocity =
                            (values[index + 1].tcpPose.translation()
                                - values[index].tcpPose.translation()) / duration;
                    }
                } else if(!result.empty()) {
                    pose.linearVelocity = result.back().linearVelocity;
                }
                result.push_back(pose);
            }
            return result;
        }

        spraythickness::AlgorithmReproductionTask makeTanaka(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const std::vector<published::SprayPose>& poses,
            const Json& json)
        {
            published::TanakaInputModel input;
            input.sprayPoses = poses;
            for(const sprayworkpiece::SurfaceSample& sample : workpiece.samples) {
                input.targetPoints.push_back({ sample.position, sample.normal });
            }
            published::TanakaParameters parameters;
            parameters.paintDischargeCubicMetersPerSecond =
                requiredValue<double>(json, "paint_discharge_m3_per_s");
            parameters.referenceDistanceMeters =
                requiredValue<double>(json, "reference_distance_m");
            parameters.referenceSigmaXMeters =
                requiredValue<double>(json, "reference_sigma_x_m");
            parameters.referenceSigmaYMeters =
                requiredValue<double>(json, "reference_sigma_y_m");
            parameters.distanceExponent =
                requiredValue<double>(json, "distance_exponent");
            parameters.incidenceExponent =
                requiredValue<double>(json, "incidence_exponent");
            return { spraythickness::ReproductionAlgorithmKind::Tanaka2024,
                std::move(input), std::move(parameters) };
        }

        spraythickness::AlgorithmReproductionTask makeTzinava(
            published::TriangleMesh mesh,
            const std::vector<published::SprayPose>& poses,
            const Json& json,
            const PublishedReproductionRuntimeInputs& runtimeInputs)
        {
            published::TzinavaInputModel input;
            input.initialMesh = std::move(mesh);
            input.gunTrajectory = poses;
            input.objectRotationOrigin = runtimeInputs.objectRotationOriginMeters;
            input.objectRotationAxis = runtimeInputs.objectRotationAxis;
            input.objectAngularSpeedRadiansPerSecond =
                runtimeInputs.objectAngularSpeedRadiansPerSecond;
            published::TzinavaParameters parameters;
            const std::string beam = requiredValue<std::string>(json, "beam_kind");
            if(beam == "cylindrical") {
                parameters.beamKind = published::TzinavaBeamKind::Cylindrical;
            } else if(beam == "cylindrical_conical") {
                parameters.beamKind =
                    published::TzinavaBeamKind::CylindricalConical;
            } else {
                throw std::invalid_argument(
                    "beam_kind must be 'cylindrical' or 'cylindrical_conical'.");
            }
            parameters.beamRadiusMeters =
                requiredValue<double>(json, "beam_radius_m");
            parameters.cylindricalLengthMeters =
                requiredValue<double>(json, "cylindrical_length_m");
            parameters.coneHalfAngleRadians =
                requiredValue<double>(json, "cone_half_angle_rad");
            parameters.gaussianSigmaMeters =
                requiredValue<double>(json, "gaussian_sigma_m");
            parameters.gaussianRadialProfile =
                requiredValue<bool>(json, "gaussian_radial_profile");
            parameters.speedCoefficientB =
                requiredValue<double>(json, "speed_coefficient_b");
            parameters.speedCoefficientC =
                requiredValue<double>(json, "speed_coefficient_c");
            parameters.referenceSpotSpeedMillimetersPerSecond =
                requiredValue<double>(json, "reference_spot_speed_mm_per_s");
            parameters.timeStepOverlapFactor =
                requiredValue<double>(json, "time_step_overlap_factor");
            parameters.stationaryTimeStepSeconds =
                requiredValue<double>(json, "stationary_time_step_s");
            parameters.lookupReferenceDwellSeconds =
                requiredValue<double>(json, "lookup_reference_dwell_s");
            const Json& table = required(json, "thickness_table");
            parameters.thicknessTable.standOffDistancesMeters =
                requiredValue<std::vector<double>>(table, "stand_off_distances_m");
            parameters.thicknessTable.impactAnglesDegrees =
                requiredValue<std::vector<double>>(table, "impact_angles_deg");
            parameters.thicknessTable.thicknessMeters =
                requiredValue<std::vector<double>>(table, "thickness_m");
            return { spraythickness::ReproductionAlgorithmKind::Tzinava2020,
                std::move(input), std::move(parameters) };
        }

        spraythickness::AlgorithmReproductionTask makeWu(
            published::TriangleMesh mesh,
            const std::vector<published::SprayPose>& poses,
            const Json& json)
        {
            published::WuInputModel input{ std::move(mesh), poses };
            published::WuParameters parameters;
            parameters.peakCylinderHeightMeters =
                requiredValue<double>(json, "peak_cylinder_height_m");
            parameters.gaussianSigmaMeters =
                requiredValue<double>(json, "gaussian_sigma_m");
            parameters.maximumDeflectionRadians =
                requiredValue<double>(json, "maximum_deflection_rad");
            parameters.rayAngularStepRadians =
                requiredValue<double>(json, "ray_angular_step_rad");
            parameters.cylinderRadiusMeters =
                requiredValue<double>(json, "cylinder_radius_m");
            parameters.sprayAngleRelativeDepositionEfficiency =
                polynomial(json, "spray_angle_rde_coefficients");
            parameters.sprayDistanceRelativeDepositionEfficiency =
                polynomial(json, "spray_distance_rde_coefficients");
            parameters.traverseSpeedPeakCorrectionFactor =
                polynomial(json, "traverse_speed_pcf_coefficients");
            return { spraythickness::ReproductionAlgorithmKind::Wu2020,
                std::move(input), std::move(parameters) };
        }

        spraythickness::AlgorithmReproductionTask makeFuke(
            const published::TriangleMesh& mesh,
            const std::vector<spraytrajectory::SprayTrajectorySample>& values,
            const spraycore::SprayTool& tool,
            const Json& json)
        {
            published::FukeInputModel input;
            input.vertices = mesh.vertices;
            for(const auto& face : mesh.faces) {
                input.polygons.push_back({ { face[0], face[1], face[2] } });
            }
            input.vaporSourcePosition = Eigen::Vector3d::Zero();
            input.vaporSourceNormal = Eigen::Vector3d::UnitZ();
            for(const auto& value : values) {
                const Eigen::Isometry3d toolPose = value.tcpPose * tool.T_link_tool;
                Eigen::Isometry3d sourceFrame = Eigen::Isometry3d::Identity();
                const Eigen::Vector3d axis =
                    tool.worldSprayDirection(toolPose).normalized();
                Eigen::Vector3d frameX;
                Eigen::Vector3d frameY;
                const Eigen::Vector3d reference = std::abs(axis.z()) < 0.9
                    ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
                frameX = reference.cross(axis).normalized();
                frameY = axis.cross(frameX).normalized();
                sourceFrame.linear().col(0) = frameX;
                sourceFrame.linear().col(1) = frameY;
                sourceFrame.linear().col(2) = axis;
                sourceFrame.translation() = toolPose.translation();
                input.timesSeconds.push_back(value.time);
                input.workpiecePoses.push_back(sourceFrame.inverse());
            }
            published::FukeParameters parameters;
            parameters.referenceThicknessRateMetersPerSecond =
                requiredValue<double>(json, "reference_thickness_rate_m_per_s");
            parameters.referenceDistanceMeters =
                requiredValue<double>(json, "reference_distance_m");
            parameters.plumeExponent =
                requiredValue<int>(json, "plume_exponent");
            return { spraythickness::ReproductionAlgorithmKind::Fuke2005,
                std::move(input), std::move(parameters) };
        }

        spraythickness::AlgorithmReproductionTask makeVanerio(
            published::TriangleMesh mesh,
            const std::vector<published::SprayPose>& poses,
            const Json& json)
        {
            published::VanerioInputModel input{ std::move(mesh), poses };
            published::VanerioParameters parameters;
            parameters.growthRateCoefficientMetersPerSecond =
                requiredValue<double>(json, "growth_rate_coefficient_m_per_s");
            parameters.jetRadiusMeters =
                requiredValue<double>(json, "jet_radius_m");
            parameters.jetShapeCoefficientK2 =
                requiredValue<double>(json, "jet_shape_coefficient_k2");
            parameters.maximumMeshEdgeMeters =
                requiredValue<double>(json, "maximum_mesh_edge_m");
            parameters.shadowGridStepMeters =
                requiredValue<double>(json, "shadow_grid_step_m");
            parameters.depositionEfficiencyByTangentAngle =
                curve(json, "deposition_efficiency_by_tangent_angle");
            parameters.depositionEfficiencyByDistance =
                curve(json, "deposition_efficiency_by_distance");
            parameters.profileStretchByDistance =
                curve(json, "profile_stretch_by_distance");
            return { spraythickness::ReproductionAlgorithmKind::Vanerio2021,
                std::move(input), std::move(parameters) };
        }

        spraythickness::AlgorithmReproductionTask makeDynamicSurface(
            published::TriangleMesh mesh,
            const std::vector<published::SprayPose>& poses,
            const Json& json)
        {
            published::DynamicSurfaceInputModel input{ std::move(mesh), poses };
            published::DynamicSurfaceParameters parameters;
            parameters.equivalentParticleDiameterMeters =
                requiredValue<double>(json, "equivalent_particle_diameter_m");
            parameters.depositedCylinderRadiusMeters =
                requiredValue<double>(json, "deposited_cylinder_radius_m");
            parameters.equivalentParticleRatePerSecond =
                requiredValue<double>(json, "equivalent_particle_rate_per_s");
            parameters.angularMeanRadians =
                requiredValue<double>(json, "angular_mean_rad");
            parameters.angularSigmaRadians =
                requiredValue<double>(json, "angular_sigma_rad");
            parameters.maximumEjectionAngleRadians =
                requiredValue<double>(json, "maximum_ejection_angle_rad");
            parameters.polarBinCount =
                requiredValue<std::size_t>(json, "polar_bin_count");
            parameters.depositionBatchDurationSeconds =
                requiredValue<double>(json, "deposition_batch_duration_s");
            parameters.randomSeed =
                requiredValue<std::uint32_t>(json, "random_seed");
            parameters.relativeBuildUpByInclinationRadians =
                curve(json, "relative_build_up_by_inclination_rad");
            parameters.voxelLeafMeters =
                requiredValue<double>(json, "voxel_leaf_m");
            parameters.uniformSamplingRadiusMeters =
                requiredValue<double>(json, "uniform_sampling_radius_m");
            parameters.normalSearchRadiusMeters =
                requiredValue<double>(json, "normal_search_radius_m");
            parameters.normalMaximumNeighbors =
                requiredValue<std::size_t>(json, "normal_maximum_neighbors");
            parameters.boundaryGradientThresholdRadiansPerMeter =
                requiredValue<double>(json, "boundary_gradient_threshold_rad_per_m");
            parameters.dbscanRadiusMeters =
                requiredValue<double>(json, "dbscan_radius_m");
            parameters.dbscanMinimumPoints =
                requiredValue<std::size_t>(json, "dbscan_minimum_points");
            parameters.alphaShapeRadiusMeters =
                requiredValue<double>(json, "alpha_shape_radius_m");
            parameters.poissonDepth =
                requiredValue<int>(json, "poisson_depth");
            parameters.maximumHoleBoundaryEdges =
                requiredValue<std::size_t>(json, "maximum_hole_boundary_edges");
            parameters.smoothingIterations =
                requiredValue<std::size_t>(json, "smoothing_iterations");
            parameters.smoothingRelaxation =
                requiredValue<double>(json, "smoothing_relaxation");
            return {
                spraythickness::ReproductionAlgorithmKind::DynamicSurface2026,
                std::move(input), std::move(parameters) };
        }
    }

    PublishedReproductionInputProfile PublishedReproductionAdapter::inputProfile(
        spraythickness::ReproductionAlgorithmKind algorithm)
    {
        using Kind = spraythickness::ReproductionAlgorithmKind;
        switch(algorithm) {
        case Kind::CurrentMethod:
            return {
                "Triangulated workpiece surface",
                "Timed robot spray poses",
                "Open workpiece model",
                "Supported models (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY *.gltf *.GLTF *.glb *.GLB *.step *.STEP *.stp *.STP);;All files (*.*)",
                "Open spray trajectory",
                false,
                false,
                false
            };
        case Kind::Tanaka2024:
            return {
                "Discrete surface target points with normals",
                "Timed spray-gun poses",
                "Open target-point source model",
                "Supported models (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY *.gltf *.GLTF *.glb *.GLB *.step *.STEP *.stp *.STP);;All files (*.*)",
                "Open timed spray-gun trajectory",
                false,
                false,
                false
            };
        case Kind::Tzinava2020:
            return {
                "Triangulated STL surface with beam-based subdivision",
                "Gun trajectory with adaptive internal time stepping",
                "Open Tzinava STL surface",
                "STL surfaces (*.stl *.STL)",
                "Open Tzinava gun trajectory",
                true,
                true,
                true
            };
        case Kind::Wu2020:
            return {
                "CAD or mesh collision surface with deposited cylinders",
                "Nozzle trajectory with internal spatial resampling",
                "Open Wu substrate model",
                "Supported CAD and mesh models (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY *.gltf *.GLTF *.glb *.GLB *.step *.STEP *.stp *.STP);;All files (*.*)",
                "Open Wu nozzle trajectory",
                false,
                true,
                false
            };
        case Kind::Fuke2005:
            return {
                "Surface-meshed CAD polygons evaluated at centroids",
                "Relative vapor-source/workpiece pose sequence",
                "Open Fuke surface mesh",
                "Surface meshes (*.stl *.STL *.obj *.OBJ *.ply *.PLY);;All files (*.*)",
                "Open Fuke relative-pose trajectory",
                false,
                false,
                false
            };
        case Kind::Vanerio2021:
            return {
                "Remeshed evolving STL triangular surface",
                "Nozzle trajectory for interval-wise surface growth",
                "Open Vanerio STL surface",
                "STL surfaces (*.stl *.STL)",
                "Open Vanerio nozzle trajectory",
                true,
                false,
                false
            };
        case Kind::DynamicSurface2026:
            return {
                "STL surface with deposited cylinders and point-cloud reconstruction",
                "Nozzle trajectory processed in deposition batches",
                "Open dynamic-surface STL model",
                "STL surfaces (*.stl *.STL)",
                "Open dynamic-surface nozzle trajectory",
                true,
                false,
                false
            };
        }
        throw std::invalid_argument("Unknown published reproduction algorithm.");
    }

    void PublishedReproductionAdapter::writeConfigurationTemplate(
        spraythickness::ReproductionAlgorithmKind algorithm,
        const std::filesystem::path& path)
    {
        if(path.empty()) {
            throw std::invalid_argument(
                "A path is required for the calibration template.");
        }
        std::ofstream output(path);
        if(!output) {
            throw std::runtime_error(
                "The calibration template cannot be created: "
                + path.generic_u8string());
        }
        output << configurationTemplate(algorithm).dump(2) << '\n';
        if(!output) {
            throw std::runtime_error(
                "The calibration template cannot be written: "
                + path.generic_u8string());
        }
    }

    spraythickness::AlgorithmReproductionTask
    PublishedReproductionAdapter::buildTask(
        spraythickness::ReproductionAlgorithmKind algorithm,
        const sprayworkpiece::WorkpieceModel& workpiece,
        const spraytrajectory::SprayTrajectory& trajectory,
        const spraycore::SprayTool& tool,
        spraythickness::TrajectorySamplingMode samplingMode,
        double timeStepSeconds,
        const std::filesystem::path& configurationPath,
        const PublishedReproductionRuntimeInputs& runtimeInputs)
    {
        if(algorithm == spraythickness::ReproductionAlgorithmKind::CurrentMethod) {
            throw std::invalid_argument(
                "The current GPU method does not use the published-method adapter.");
        }
        const PublishedReproductionInputProfile profile = inputProfile(algorithm);
        if(profile.requiresStlSurface) {
            std::string extension = runtimeInputs.modelSourcePath.extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                [](unsigned char value) {
                    return static_cast<char>(std::tolower(value));
                });
            if(extension != ".stl") {
                throw std::invalid_argument(
                    std::string(spraythickness::reproductionAlgorithmName(algorithm))
                    + " requires an STL surface model.");
            }
        }
        if(algorithm == spraythickness::ReproductionAlgorithmKind::Tzinava2020
            && runtimeInputs.objectRotationAxis.squaredNorm() <= 1.0e-12) {
            throw std::invalid_argument(
                "Tzinava reproduction requires a non-zero object rotation axis.");
        }
        const Json json = readConfiguration(configurationPath, algorithm);
        const spraythickness::TrajectorySamplingMode effectiveSamplingMode =
            profile.forceOriginalTrajectoryPoints
            ? spraythickness::TrajectorySamplingMode::OriginalPoints
            : samplingMode;
        const auto trajectorySamples = samples(
            trajectory, effectiveSamplingMode, timeStepSeconds);
        if(trajectorySamples.size() < 2) {
            throw std::invalid_argument(
                "Published-method reproduction requires at least two trajectory samples.");
        }
        const auto poses = sprayPoses(trajectorySamples, tool);
        if(algorithm == spraythickness::ReproductionAlgorithmKind::Tanaka2024) {
            return makeTanaka(workpiece, poses, json);
        }
        const published::TriangleMesh mesh = triangleMesh(workpiece);
        switch(algorithm) {
        case spraythickness::ReproductionAlgorithmKind::Tzinava2020:
            return makeTzinava(mesh, poses, json, runtimeInputs);
        case spraythickness::ReproductionAlgorithmKind::Wu2020:
            return makeWu(mesh, poses, json);
        case spraythickness::ReproductionAlgorithmKind::Fuke2005:
            return makeFuke(mesh, trajectorySamples, tool, json);
        case spraythickness::ReproductionAlgorithmKind::Vanerio2021:
            return makeVanerio(mesh, poses, json);
        case spraythickness::ReproductionAlgorithmKind::DynamicSurface2026:
            return makeDynamicSurface(mesh, poses, json);
        case spraythickness::ReproductionAlgorithmKind::CurrentMethod:
        case spraythickness::ReproductionAlgorithmKind::Tanaka2024:
            break;
        }
        throw std::invalid_argument("Unknown published reproduction algorithm.");
    }
}
