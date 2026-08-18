#include "PaintingAnalysisMeshAdapter.h"

#include <stdexcept>

namespace robot_qt_viewer
{
    PaintingAnalysisMeshData PaintingAnalysisMeshAdapter::build(
        const assetcore::ModelDesc& model,
        const std::string& name,
        const std::string& sourcePath)
    {
        PaintingAnalysisMeshData data;
        data.workpiece.name = name;
        data.workpiece.sourceMeshPath = sourcePath;
        data.binding.sampleIndicesBySubMesh.resize(model.subMeshCount());

        bool usedFallbackNormal = false;
        for(std::size_t subMeshIndex = 0; subMeshIndex < model.subMeshCount(); ++subMeshIndex) {
            const assetcore::GeometryDesc& geometry = model.subMesh(subMeshIndex).geometry;
            std::vector<std::size_t>& bindings = data.binding.sampleIndicesBySubMesh[subMeshIndex];
            bindings.reserve(geometry.positions.size());
            for(std::size_t vertexIndex = 0; vertexIndex < geometry.positions.size(); ++vertexIndex) {
                sprayworkpiece::SurfaceSample sample;
                sample.position = geometry.positions[vertexIndex].cast<double>();
                if(vertexIndex < geometry.normals.size() &&
                    geometry.normals[vertexIndex].allFinite() &&
                    geometry.normals[vertexIndex].norm() > 1.0e-12f) {
                    sample.normal = geometry.normals[vertexIndex].cast<double>().normalized();
                } else {
                    sample.normal = Eigen::Vector3d::UnitZ();
                    usedFallbackNormal = true;
                }
                bindings.push_back(data.workpiece.samples.size());
                data.workpiece.addSample(sample);
            }
        }
        if(usedFallbackNormal) {
            data.warnings.push_back("Some mesh vertices had no valid normal; +Z was used.");
        }
        return data;
    }

    smrobot::visualization::SurfaceScalarOverlay PaintingAnalysisMeshAdapter::makeOverlay(
        const std::string& objectId,
        const PaintingAnalysisMeshBinding& binding,
        const spraythickness::ThicknessPredictionResult& prediction)
    {
        smrobot::visualization::SurfaceScalarOverlay overlay;
        overlay.objectId = objectId;
        overlay.quantityName = "Thickness";
        overlay.unit = "m";
        overlay.range.minimum = prediction.metrics.minThickness;
        overlay.range.maximum = prediction.metrics.maxThickness;
        overlay.subMeshes.resize(binding.sampleIndicesBySubMesh.size());

        for(std::size_t subMeshIndex = 0;
            subMeshIndex < binding.sampleIndicesBySubMesh.size();
            ++subMeshIndex) {
            const std::vector<std::size_t>& sampleIndices =
                binding.sampleIndicesBySubMesh[subMeshIndex];
            std::vector<double>& values = overlay.subMeshes[subMeshIndex].values;
            values.reserve(sampleIndices.size());
            for(const std::size_t sampleIndex : sampleIndices) {
                if(sampleIndex >= prediction.field.results.size()) {
                    throw std::runtime_error("Thickness sample binding is out of range.");
                }
                values.push_back(prediction.field.results[sampleIndex].thickness);
            }
        }
        return overlay;
    }
}
