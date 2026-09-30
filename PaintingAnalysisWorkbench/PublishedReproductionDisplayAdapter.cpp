#include "PublishedReproductionDisplayAdapter.h"

#include <AssetCore/GeometryDesc.h>
#include <AssetCore/SubMeshDesc.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
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
            const sprayworkpiece::WorkpieceModel& workpiece)
        {
            published::TriangleMesh faceMesh;
            std::vector<double> values;
            faceMesh.vertices.reserve(result.polygonThicknessMeters.size() * 3);
            faceMesh.faces.reserve(result.polygonThicknessMeters.size());
            values.reserve(result.polygonThicknessMeters.size() * 3);
            for(std::size_t face = 0;
                face < result.polygonThicknessMeters.size(); ++face) {
                const std::size_t offset = face * 3;
                if(offset + 2 >= workpiece.triangleIndices.size()) {
                    throw std::runtime_error(
                        "Fuke result does not match the substrate faces.");
                }
                const std::uint32_t first =
                    static_cast<std::uint32_t>(faceMesh.vertices.size());
                for(int corner = 0; corner < 3; ++corner) {
                    const std::uint32_t vertex =
                        workpiece.triangleIndices[offset + corner];
                    faceMesh.vertices.push_back(workpiece.samples.at(vertex).position);
                    values.push_back(result.polygonThicknessMeters[face]);
                }
                faceMesh.faces.push_back({ first, first + 1, first + 2 });
            }
            return { meshModel(faceMesh), sequentialBinding(values.size()),
                prediction(values),
                "polygon centroids" };
        }

        double distanceToTriangle(const Eigen::Vector3d& point,
            const Eigen::Vector3d& a, const Eigen::Vector3d& b,
            const Eigen::Vector3d& c)
        {
            const Eigen::Vector3d ab = b - a;
            const Eigen::Vector3d ac = c - a;
            if(ab.cross(ac).squaredNorm() <= 1.0e-30) {
                const auto segmentDistance = [&point](
                    const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second) {
                    const Eigen::Vector3d edge = second - first;
                    const double lengthSquared = edge.squaredNorm();
                    const double ratio = lengthSquared > 0.0
                        ? std::clamp((point - first).dot(edge) / lengthSquared,
                            0.0, 1.0)
                        : 0.0;
                    return (point - first - ratio * edge).norm();
                };
                return std::min({ segmentDistance(a, b),
                    segmentDistance(b, c), segmentDistance(c, a) });
            }
            const Eigen::Vector3d ap = point - a;
            const double d1 = ab.dot(ap);
            const double d2 = ac.dot(ap);
            if(d1 <= 0.0 && d2 <= 0.0) {
                return ap.norm();
            }
            const Eigen::Vector3d bp = point - b;
            const double d3 = ab.dot(bp);
            const double d4 = ac.dot(bp);
            if(d3 >= 0.0 && d4 <= d3) {
                return bp.norm();
            }
            const double vc = d1 * d4 - d3 * d2;
            if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                return (point - (a + d1 / (d1 - d3) * ab)).norm();
            }
            const Eigen::Vector3d cp = point - c;
            const double d5 = ab.dot(cp);
            const double d6 = ac.dot(cp);
            if(d6 >= 0.0 && d5 <= d6) {
                return cp.norm();
            }
            const double vb = d5 * d2 - d1 * d6;
            if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                return (point - (a + d2 / (d2 - d6) * ac)).norm();
            }
            const double va = d3 * d6 - d5 * d4;
            if(va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
                const Eigen::Vector3d edge = c - b;
                return (point - (b + (d4 - d3)
                    / ((d4 - d3) + (d5 - d6)) * edge)).norm();
            }
            const double denominator = 1.0 / (va + vb + vc);
            return (point - (a + vb * denominator * ab
                + vc * denominator * ac)).norm();
        }

        class OriginalSurfaceIndex
        {
        public:
            explicit OriginalSurfaceIndex(const sprayworkpiece::WorkpieceModel& mesh)
                : m_mesh(mesh)
            {
                const std::size_t count = mesh.triangleIndices.size() / 3;
                if(count == 0) {
                    throw std::runtime_error(
                        "Evolved surface has no original substrate faces.");
                }
                m_faces.resize(count);
                std::iota(m_faces.begin(), m_faces.end(), std::size_t{ 0 });
                m_nodes.reserve(count / 4 + 1);
                build(0, count);
            }

            double nearestDistance(const Eigen::Vector3d& point) const
            {
                double nearestSquared = std::numeric_limits<double>::infinity();
                search(0, point, nearestSquared);
                return std::sqrt(nearestSquared);
            }

        private:
            struct Node
            {
                Eigen::Vector3d lower = Eigen::Vector3d::Zero();
                Eigen::Vector3d upper = Eigen::Vector3d::Zero();
                std::size_t begin{ 0 };
                std::size_t end{ 0 };
                std::size_t left{ 0 };
                std::size_t right{ 0 };
            };

            const Eigen::Vector3d& vertex(std::size_t face, int corner) const
            {
                return m_mesh.samples[m_mesh.triangleIndices[face * 3 + corner]]
                    .position;
            }

            std::size_t build(std::size_t begin, std::size_t end)
            {
                Node node;
                node.begin = begin;
                node.end = end;
                node.lower.setConstant(std::numeric_limits<double>::infinity());
                node.upper.setConstant(-std::numeric_limits<double>::infinity());
                for(std::size_t offset = begin; offset < end; ++offset) {
                    for(int corner = 0; corner < 3; ++corner) {
                        node.lower = node.lower.cwiseMin(vertex(m_faces[offset], corner));
                        node.upper = node.upper.cwiseMax(vertex(m_faces[offset], corner));
                    }
                }
                const std::size_t index = m_nodes.size();
                m_nodes.push_back(node);
                if(end - begin <= 8) {
                    return index;
                }
                Eigen::Index axis = 0;
                (node.upper - node.lower).maxCoeff(&axis);
                const std::size_t middle = begin + (end - begin) / 2;
                std::nth_element(m_faces.begin() + begin,
                    m_faces.begin() + middle, m_faces.begin() + end,
                    [this, axis](std::size_t first, std::size_t second) {
                        const double firstCenter = (vertex(first, 0)[axis]
                            + vertex(first, 1)[axis] + vertex(first, 2)[axis]) / 3.0;
                        const double secondCenter = (vertex(second, 0)[axis]
                            + vertex(second, 1)[axis] + vertex(second, 2)[axis]) / 3.0;
                        return firstCenter < secondCenter;
                    });
                const std::size_t left = build(begin, middle);
                const std::size_t right = build(middle, end);
                m_nodes[index].left = left;
                m_nodes[index].right = right;
                return index;
            }

            double boundsDistanceSquared(const Node& node,
                const Eigen::Vector3d& point) const
            {
                return (point.cwiseMax(node.lower).cwiseMin(node.upper)
                    - point).squaredNorm();
            }

            void search(std::size_t index, const Eigen::Vector3d& point,
                double& nearestSquared) const
            {
                const Node& node = m_nodes[index];
                if(boundsDistanceSquared(node, point) >= nearestSquared) {
                    return;
                }
                if(node.left == 0) {
                    for(std::size_t offset = node.begin; offset < node.end; ++offset) {
                        const std::size_t face = m_faces[offset];
                        const double distance = distanceToTriangle(point,
                            vertex(face, 0), vertex(face, 1), vertex(face, 2));
                        nearestSquared = std::min(nearestSquared,
                            distance * distance);
                    }
                    return;
                }
                const double leftDistance = boundsDistanceSquared(
                    m_nodes[node.left], point);
                const double rightDistance = boundsDistanceSquared(
                    m_nodes[node.right], point);
                const std::size_t first = leftDistance <= rightDistance
                    ? node.left : node.right;
                const std::size_t second = leftDistance <= rightDistance
                    ? node.right : node.left;
                search(first, point, nearestSquared);
                search(second, point, nearestSquared);
            }

            const sprayworkpiece::WorkpieceModel& m_mesh;
            std::vector<std::size_t> m_faces;
            std::vector<Node> m_nodes;
        };

        PublishedReproductionDisplayData wu(const published::WuResult& result,
            const sprayworkpiece::WorkpieceModel& original,
            double normalDisplayScale)
        {
            constexpr std::uint32_t kSegments = 16;
            constexpr std::size_t kVerticesPerCylinder = 2 * kSegments + 1;
            constexpr std::size_t kIndicesPerCylinder = 9 * kSegments;
            constexpr double kTwoPi = 6.28318530717958647692;
            auto model = std::make_shared<assetcore::ModelDesc>();
            model->subMeshes().emplace_back();
            assetcore::SubMeshDesc& subMesh = model->subMeshes().back();
            subMesh.name = "Wu deposited cylinders";
            assetcore::GeometryDesc& geometry = subMesh.geometry;
            geometry.positions.reserve(original.samples.size()
                + kVerticesPerCylinder * result.depositedCylinders.size());
            geometry.indices.reserve(original.triangleIndices.size()
                + kIndicesPerCylinder * result.depositedCylinders.size());
            std::vector<double> values;
            values.reserve(geometry.positions.capacity());
            for(const sprayworkpiece::SurfaceSample& sample : original.samples) {
                geometry.positions.push_back(sample.position.cast<float>());
                values.push_back(0.0);
            }
            geometry.indices = original.triangleIndices;

            for(const published::WuDepositedCylinder& cylinder :
                result.depositedCylinders) {
                if(cylinder.substrateFaceIndex >= result.substrate.faces.size()) {
                    throw std::runtime_error(
                        "Wu cylinder has no source substrate face.");
                }
                const auto& face = result.substrate.faces[
                    cylinder.substrateFaceIndex];
                const Eigen::Vector3d& a = result.substrate.vertices[face[0]];
                const Eigen::Vector3d normal =
                    published::faceNormal(result.substrate,
                        cylinder.substrateFaceIndex);
                const Eigen::Vector3d axis =
                    cylinder.growthDirection.normalized();
                const Eigen::Vector3d reference = std::abs(normal.z()) < 0.9
                    ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
                const Eigen::Vector3d tangentX =
                    reference.cross(normal).normalized();
                const Eigen::Vector3d tangentY = normal.cross(tangentX);
                const std::uint32_t first =
                    static_cast<std::uint32_t>(geometry.positions.size());
                for(std::uint32_t ring = 0; ring < 2; ++ring) {
                    const Eigen::Vector3d center = cylinder.baseCenter
                        + static_cast<double>(ring) * cylinder.heightMeters * axis;
                    const double signedHeight = (center - a).dot(normal);
                    const Eigen::Vector3d displayCenter = center
                        + (normalDisplayScale - 1.0) * signedHeight * normal;
                    for(std::uint32_t segment = 0; segment < kSegments; ++segment) {
                        const double angle = kTwoPi * segment / kSegments;
                        const Eigen::Vector3d vertex = displayCenter
                            + cylinder.radiusMeters * (std::cos(angle) * tangentX
                                + std::sin(angle) * tangentY);
                        geometry.positions.push_back(vertex.cast<float>());
                        values.push_back(std::max(0.0, signedHeight));
                    }
                }
                const Eigen::Vector3d top = cylinder.baseCenter
                    + cylinder.heightMeters * axis;
                const double topSignedHeight = (top - a).dot(normal);
                geometry.positions.push_back((top
                    + (normalDisplayScale - 1.0) * topSignedHeight * normal)
                        .cast<float>());
                values.push_back(std::max(0.0, topSignedHeight));
                for(std::uint32_t segment = 0; segment < kSegments; ++segment) {
                    const std::uint32_t next = (segment + 1) % kSegments;
                    const std::uint32_t bottom = first + segment;
                    const std::uint32_t topRim = bottom + kSegments;
                    geometry.indices.insert(geometry.indices.end(), {
                        bottom, first + next, topRim,
                        first + next, first + kSegments + next, topRim,
                        first + 2 * kSegments, topRim,
                        first + kSegments + next
                    });
                }
            }
            calculateNormals(geometry);
            return { std::move(model), sequentialBinding(values.size()),
                prediction(values), "deposited cylinders" };
        }

        std::vector<double> surfaceDisplacement(
            const published::TriangleMesh& mesh,
            const sprayworkpiece::WorkpieceModel& original)
        {
            const OriginalSurfaceIndex surface(original);
            std::vector<double> values;
            values.reserve(mesh.vertices.size());
            for(const Eigen::Vector3d& vertex : mesh.vertices) {
                values.push_back(surface.nearestDistance(vertex));
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
        const PaintingAnalysisMeshBinding& originalBinding,
        double wuNormalDisplayScale)
    {
        return std::visit([&](const auto& native) {
            using Result = std::decay_t<decltype(native)>;
            if constexpr(std::is_same_v<Result,
                spraythickness::CurrentMethodReproductionResult>) {
                return PublishedReproductionDisplayData{ nullptr,
                    originalBinding, native.prediction, "mesh vertices" };
            } else if constexpr(std::is_same_v<Result, published::TzinavaResult>) {
                return tzinava(native);
            } else if constexpr(std::is_same_v<Result, published::WuResult>) {
                return wu(native, originalWorkpiece, wuNormalDisplayScale);
            } else if constexpr(std::is_same_v<Result, published::FukeResult>) {
                return fuke(native, originalWorkpiece);
            } else if constexpr(std::is_same_v<Result, published::VanerioResult>) {
                if(native.vertexThicknessMeters.size()
                    != native.evolvedStlSurface.vertices.size()) {
                    throw std::runtime_error(
                        "Vanerio thickness does not match the evolved surface.");
                }
                return PublishedReproductionDisplayData{
                    meshModel(native.evolvedStlSurface),
                    sequentialBinding(native.vertexThicknessMeters.size()),
                    prediction(native.vertexThicknessMeters), "dynamic STL surface" };
            } else {
                return evolvedSurface(native.evolvedStlSurface,
                    originalWorkpiece, "reconstructed dynamic coating surface");
            }
        }, result.nativeResult);
    }
}
