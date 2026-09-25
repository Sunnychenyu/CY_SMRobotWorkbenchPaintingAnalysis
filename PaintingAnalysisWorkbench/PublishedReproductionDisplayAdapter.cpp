#include "PublishedReproductionDisplayAdapter.h"

#include <AssetCore/GeometryDesc.h>
#include <AssetCore/SubMeshDesc.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <type_traits>

namespace robot_qt_viewer
{
    namespace
    {
        namespace published = spraythickness::published;

        spraythickness::ThicknessPredictionResult prediction(
            const std::vector<double>& values)
        {
            spraythickness::ThicknessPredictionResult result;
            result.field.results.reserve(values.size());
            for(std::size_t index = 0; index < values.size(); ++index) {
                result.field.results.push_back({ index, values[index], 0.0, 0.0 });
            }
            result.metrics = spraythickness::ThicknessMetricsCalculator::calculate(
                result.field);
            return result;
        }

        void calculateNormals(assetcore::GeometryDesc& geometry)
        {
            geometry.normals.assign(
                geometry.positions.size(), Eigen::Vector3f::Zero());
            for(std::size_t offset = 0;
                offset + 2 < geometry.indices.size(); offset += 3) {
                const std::uint32_t first = geometry.indices[offset];
                const std::uint32_t second = geometry.indices[offset + 1];
                const std::uint32_t third = geometry.indices[offset + 2];
                const Eigen::Vector3f normal =
                    (geometry.positions[second] - geometry.positions[first]).cross(
                        geometry.positions[third] - geometry.positions[first]);
                geometry.normals[first] += normal;
                geometry.normals[second] += normal;
                geometry.normals[third] += normal;
            }
            for(Eigen::Vector3f& normal : geometry.normals) {
                normal = normal.squaredNorm() > 1.0e-16f
                    ? normal.normalized() : Eigen::Vector3f::UnitZ();
            }
        }

        std::shared_ptr<assetcore::ModelDesc> meshModel(
            const published::TriangleMesh& mesh)
        {
            auto model = std::make_shared<assetcore::ModelDesc>();
            assetcore::SubMeshDesc subMesh;
            subMesh.name = "Published reproduction";
            subMesh.geometry.positions.reserve(mesh.vertices.size());
            for(const Eigen::Vector3d& vertex : mesh.vertices) {
                subMesh.geometry.positions.push_back(vertex.cast<float>());
            }
            subMesh.geometry.indices.reserve(mesh.faces.size() * 3);
            for(const auto& face : mesh.faces) {
                subMesh.geometry.indices.insert(subMesh.geometry.indices.end(),
                    { face[0], face[1], face[2] });
            }
            calculateNormals(subMesh.geometry);
            model->addSubMesh(subMesh);
            return model;
        }

        PaintingAnalysisMeshBinding sequentialBinding(std::size_t count)
        {
            PaintingAnalysisMeshBinding binding;
            binding.sampleIndicesBySubMesh.resize(1);
            auto& indices = binding.sampleIndicesBySubMesh.front();
            indices.resize(count);
            std::iota(indices.begin(), indices.end(), std::size_t{ 0 });
            return binding;
        }

        PublishedReproductionDisplayData tanaka(
            const published::TanakaResult& result,
            const PaintingAnalysisMeshBinding& originalBinding)
        {
            return { nullptr, originalBinding,
                prediction(result.pointThicknessMeters), "target points" };
        }

        PublishedReproductionDisplayData tzinava(
            const published::TzinavaResult& result)
        {
            published::TriangleMesh faceMesh;
            std::vector<double> values;
            for(std::size_t faceIndex = 0;
                faceIndex < result.subdividedMesh.faces.size(); ++faceIndex) {
                const auto& face = result.subdividedMesh.faces[faceIndex];
                const std::uint32_t offset =
                    static_cast<std::uint32_t>(faceMesh.vertices.size());
                faceMesh.vertices.push_back(result.subdividedMesh.vertices[face[0]]);
                faceMesh.vertices.push_back(result.subdividedMesh.vertices[face[1]]);
                faceMesh.vertices.push_back(result.subdividedMesh.vertices[face[2]]);
                faceMesh.faces.push_back({ offset, offset + 1, offset + 2 });
                for(int corner = 0; corner < 3; ++corner) {
                    values.push_back(result.faceThicknessMeters[faceIndex]);
                }
            }
            return { meshModel(faceMesh), sequentialBinding(values.size()),
                prediction(values), "subdivided triangle faces" };
        }

        PublishedReproductionDisplayData fuke(
            const published::FukeResult& result,
            const sprayworkpiece::WorkpieceModel& workpiece,
            const PaintingAnalysisMeshBinding& originalBinding)
        {
            std::vector<double> sums(workpiece.samples.size(), 0.0);
            std::vector<std::size_t> counts(workpiece.samples.size(), 0);
            for(std::size_t face = 0;
                face < result.polygonThicknessMeters.size(); ++face) {
                const std::size_t offset = face * 3;
                if(offset + 2 >= workpiece.triangleIndices.size()) {
                    break;
                }
                for(int corner = 0; corner < 3; ++corner) {
                    const std::uint32_t vertex =
                        workpiece.triangleIndices[offset + corner];
                    sums[vertex] += result.polygonThicknessMeters[face];
                    ++counts[vertex];
                }
            }
            for(std::size_t index = 0; index < sums.size(); ++index) {
                if(counts[index] > 0) {
                    sums[index] /= static_cast<double>(counts[index]);
                }
            }
            return { nullptr, originalBinding, prediction(sums),
                "polygon centroids" };
        }

        void basis(const Eigen::Vector3d& axis,
            Eigen::Vector3d& x, Eigen::Vector3d& y)
        {
            const Eigen::Vector3d reference = std::abs(axis.z()) < 0.9
                ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
            x = reference.cross(axis).normalized();
            y = axis.cross(x).normalized();
        }

        PublishedReproductionDisplayData wu(const published::WuResult& result)
        {
            constexpr std::size_t kSegments = 12;
            published::TriangleMesh mesh;
            std::vector<double> values;
            for(const published::WuDepositedCylinder& cylinder :
                result.depositedCylinders) {
                const Eigen::Vector3d axis =
                    cylinder.growthDirection.normalized();
                Eigen::Vector3d x;
                Eigen::Vector3d y;
                basis(axis, x, y);
                const std::uint32_t offset =
                    static_cast<std::uint32_t>(mesh.vertices.size());
                for(std::size_t ring = 0; ring < 2; ++ring) {
                    for(std::size_t segment = 0; segment < kSegments; ++segment) {
                        const double angle = 2.0 * 3.14159265358979323846
                            * static_cast<double>(segment)
                            / static_cast<double>(kSegments);
                        mesh.vertices.push_back(cylinder.baseCenter
                            + static_cast<double>(ring) * cylinder.heightMeters * axis
                            + cylinder.radiusMeters
                                * (std::cos(angle) * x + std::sin(angle) * y));
                        values.push_back(cylinder.heightMeters);
                    }
                }
                for(std::size_t segment = 0; segment < kSegments; ++segment) {
                    const std::uint32_t next = static_cast<std::uint32_t>(
                        (segment + 1) % kSegments);
                    const std::uint32_t bottom =
                        offset + static_cast<std::uint32_t>(segment);
                    const std::uint32_t top = bottom + kSegments;
                    mesh.faces.push_back({ bottom, offset + next, top });
                    mesh.faces.push_back({ offset + next,
                        offset + kSegments + next, top });
                }
            }
            if(mesh.empty()) {
                throw std::runtime_error("Wu reproduction deposited no cylinders.");
            }
            return { meshModel(mesh), sequentialBinding(values.size()),
                prediction(values), "deposited cylinders" };
        }

        std::vector<double> surfaceDisplacement(
            const published::TriangleMesh& mesh,
            const sprayworkpiece::WorkpieceModel& original)
        {
            std::vector<double> values;
            values.reserve(mesh.vertices.size());
            for(const Eigen::Vector3d& vertex : mesh.vertices) {
                double nearest = std::numeric_limits<double>::infinity();
                for(const sprayworkpiece::SurfaceSample& sample : original.samples) {
                    nearest = std::min(nearest,
                        (vertex - sample.position).norm());
                }
                values.push_back(std::isfinite(nearest) ? nearest : 0.0);
            }
            return values;
        }

        PublishedReproductionDisplayData evolvedSurface(
            const published::TriangleMesh& mesh,
            const sprayworkpiece::WorkpieceModel& original,
            const std::string& label)
        {
            const std::vector<double> values =
                surfaceDisplacement(mesh, original);
            return { meshModel(mesh), sequentialBinding(values.size()),
                prediction(values), label };
        }
    }

    PublishedReproductionDisplayData PublishedReproductionDisplayAdapter::build(
        const spraythickness::AlgorithmReproductionResult& result,
        const sprayworkpiece::WorkpieceModel& originalWorkpiece,
        const PaintingAnalysisMeshBinding& originalBinding)
    {
        return std::visit([&](const auto& native) {
            using Result = std::decay_t<decltype(native)>;
            if constexpr(std::is_same_v<Result,
                spraythickness::CurrentMethodReproductionResult>) {
                return PublishedReproductionDisplayData{ nullptr,
                    originalBinding, native.prediction, "mesh vertices" };
            } else if constexpr(std::is_same_v<Result, published::TanakaResult>) {
                return tanaka(native, originalBinding);
            } else if constexpr(std::is_same_v<Result, published::TzinavaResult>) {
                return tzinava(native);
            } else if constexpr(std::is_same_v<Result, published::WuResult>) {
                return wu(native);
            } else if constexpr(std::is_same_v<Result, published::FukeResult>) {
                return fuke(native, originalWorkpiece, originalBinding);
            } else if constexpr(std::is_same_v<Result, published::VanerioResult>) {
                return evolvedSurface(native.evolvedStlSurface,
                    originalWorkpiece, "dynamic STL surface");
            } else {
                return evolvedSurface(native.evolvedStlSurface,
                    originalWorkpiece, "reconstructed dynamic coating surface");
            }
        }, result.nativeResult);
    }
}
