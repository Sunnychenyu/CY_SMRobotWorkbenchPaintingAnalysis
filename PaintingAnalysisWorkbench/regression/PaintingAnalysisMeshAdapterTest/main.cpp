#include <PaintingAnalysisMeshAdapter.h>

#include <iostream>

int main()
{
    assetcore::ModelDesc model;
    assetcore::SubMeshDesc first;
    first.geometry.positions = {
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(1.0f, 0.0f, 0.0f)
    };
    first.geometry.normals = {
        Eigen::Vector3f::UnitY(),
        Eigen::Vector3f::UnitY()
    };
    model.addSubMesh(first);

    assetcore::SubMeshDesc second;
    second.geometry.positions = { Eigen::Vector3f(0.0f, 0.0f, 1.0f) };
    model.addSubMesh(second);

    const robot_qt_viewer::PaintingAnalysisMeshData mesh =
        robot_qt_viewer::PaintingAnalysisMeshAdapter::build(model, "mesh", "mesh.stl");
    if(mesh.workpiece.sampleCount() != 3 ||
        mesh.binding.sampleIndicesBySubMesh.size() != 2 ||
        mesh.binding.sampleIndicesBySubMesh[0].size() != 2 ||
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
    prediction.metrics.minThickness = 1.0;
    prediction.metrics.maxThickness = 3.0;
    const smrobot::visualization::SurfaceScalarOverlay overlay =
        robot_qt_viewer::PaintingAnalysisMeshAdapter::makeOverlay(
            "object",
            mesh.binding,
            prediction);
    if(overlay.objectId != "object" || overlay.subMeshes.size() != 2 ||
        overlay.subMeshes[0].values.size() != 2 ||
        overlay.subMeshes[1].values.size() != 1 ||
        overlay.subMeshes[1].values[0] != 3.0) {
        std::cerr << "Scalar overlay projection contract failed.\n";
        return 2;
    }
    return 0;
}
