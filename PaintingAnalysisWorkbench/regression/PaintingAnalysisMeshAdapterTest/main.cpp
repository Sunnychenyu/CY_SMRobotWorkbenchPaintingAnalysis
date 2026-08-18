#include <PaintingAnalysisMeshAdapter.h>

#include <iostream>

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
    return 0;
}
