#include "PaintingAnalysisMeshAdapter.h"

#include <stdexcept>

namespace robot_qt_viewer
{
    PaintingAnalysisMeshData PaintingAnalysisMeshAdapter::build(
        const assetcore::ModelDesc& model,
        const std::string& name,
        const std::string& sourcePath,
        const Eigen::Isometry3d& worldFromModel)
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
            const std::size_t sampleOffset = data.workpiece.samples.size();
            for(std::size_t vertexIndex = 0; vertexIndex < geometry.positions.size(); ++vertexIndex) {
                sprayworkpiece::SurfaceSample sample;
                const glm::vec4 localPosition = model.get_local() * glm::vec4(
                    geometry.positions[vertexIndex].x(),
                    geometry.positions[vertexIndex].y(),
                    geometry.positions[vertexIndex].z(),
                    1.0f);
                const Eigen::Vector3d modelPosition(
                    static_cast<double>(localPosition.x),
                    static_cast<double>(localPosition.y),
                    static_cast<double>(localPosition.z));
                sample.position = worldFromModel * modelPosition;
                if(vertexIndex < geometry.normals.size() &&
                    geometry.normals[vertexIndex].allFinite() &&
                    geometry.normals[vertexIndex].norm() > 1.0e-12f) {
                    const glm::vec4 localNormal = model.get_local() * glm::vec4(
                        geometry.normals[vertexIndex].x(),
                        geometry.normals[vertexIndex].y(),
                        geometry.normals[vertexIndex].z(),
                        0.0f);
                    const Eigen::Vector3d modelNormal(
                        static_cast<double>(localNormal.x),
                        static_cast<double>(localNormal.y),
                        static_cast<double>(localNormal.z));
                    sample.normal = (worldFromModel.linear() * modelNormal).normalized();
                } else {
                    sample.normal = Eigen::Vector3d::UnitZ();
                    usedFallbackNormal = true;
                }
                bindings.push_back(data.workpiece.samples.size());
                data.workpiece.addSample(sample);
            }

            if(!geometry.indices.empty()) {
                const std::size_t triangleIndexCount = geometry.indices.size() -
                    geometry.indices.size() % 3;
                for(std::size_t indexOffset = 0;
                    indexOffset < triangleIndexCount;
                    indexOffset += 3) {
                    const std::uint32_t i0 = geometry.indices[indexOffset];
                    const std::uint32_t i1 = geometry.indices[indexOffset + 1];
                    const std::uint32_t i2 = geometry.indices[indexOffset + 2];
                    if(i0 >= geometry.positions.size() ||
                        i1 >= geometry.positions.size() ||
                        i2 >= geometry.positions.size()) {
                        data.warnings.push_back("Ignored an out-of-range mesh triangle index.");
                        continue;
                    }
                    data.workpiece.triangleIndices.push_back(
                        static_cast<std::uint32_t>(sampleOffset + i0));
                    data.workpiece.triangleIndices.push_back(
                        static_cast<std::uint32_t>(sampleOffset + i1));
                    data.workpiece.triangleIndices.push_back(
                        static_cast<std::uint32_t>(sampleOffset + i2));
                }
                if(triangleIndexCount != geometry.indices.size()) {
                    data.warnings.push_back("Ignored incomplete mesh triangle indices.");
                }
            } else if(geometry.positions.size() % 3 == 0) {
                for(std::size_t index = 0; index < geometry.positions.size(); ++index) {
                    data.workpiece.triangleIndices.push_back(
                        static_cast<std::uint32_t>(sampleOffset + index));
                }
            } else {
                data.warnings.push_back("A submesh had no triangle index buffer.");
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
