#include "PaintingAnalysisMeshAdapter.h"

#include <CustomLog/CustomLog.h>
#include <SimulationProject/RuntimePaths.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>
#include <cstdint>
#include <limits>
#include <numeric>
#include <array>
#include <mutex>
#include <queue>
#include <unordered_set>
#include <optional>

namespace robot_qt_viewer
{
    namespace
    {
        bool inAdaptiveRegion(
            const Eigen::Vector3d& position,
            const AdaptiveMeshOptions& options)
        {
            const Eigen::Vector3d axis = options.axisDirection.squaredNorm() > 1.0e-16
                ? options.axisDirection.normalized() : Eigen::Vector3d::UnitZ();
            const Eigen::Vector3d relative = position - options.axisOrigin;
            const double z = relative.dot(axis);
            const double r = (relative - z * axis).norm();
            const Eigen::Vector2d minimum = options.selectionMinimum.cwiseMin(options.selectionMaximum);
            const Eigen::Vector2d maximum = options.selectionMinimum.cwiseMax(options.selectionMaximum);
            const Eigen::Vector2d sectionPoint(r, z);
            if(sectionPoint.x() < minimum.x() || sectionPoint.x() > maximum.x()
                || sectionPoint.y() < minimum.y() || sectionPoint.y() > maximum.y()) {
                return false;
            }
            if(options.selectionPolygon.size() < 3) {
                return true;
            }
            bool inside = false;
            for(std::size_t index = 0; index < options.selectionPolygon.size(); ++index) {
                const Eigen::Vector2d first = options.selectionPolygon[index];
                const Eigen::Vector2d second = options.selectionPolygon[
                    (index + 1) % options.selectionPolygon.size()];
                const Eigen::Vector2d edge = second - first;
                const Eigen::Vector2d relative = sectionPoint - first;
                if(std::abs(edge.x() * relative.y() - edge.y() * relative.x()) <= 1.0e-10
                    && relative.dot(sectionPoint - second) <= 1.0e-10) {
                    return true;
                }
                if((first.y() > sectionPoint.y()) != (second.y() > sectionPoint.y())) {
                    const double xAtPoint = first.x()
                        + (second.x() - first.x()) * (sectionPoint.y() - first.y())
                            / (second.y() - first.y());
                    if(sectionPoint.x() < xAtPoint) {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

        using Quadric = Eigen::Matrix4d;

        struct SimplifyVertex
        {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
            Quadric quadric = Quadric::Zero();
            std::unordered_set<std::size_t> neighbours;
            std::unordered_set<std::size_t> incidentFaces;
            bool protectedVertex{ false };
            bool valid{ true };
        };

        struct SimplifyFace
        {
            std::array<std::size_t, 3> vertices{};
            bool valid{ true };
        };

        struct CollapseCandidate
        {
            double cost{ 0.0 };
            std::size_t first{ 0 };
            std::size_t second{ 0 };
            std::uint64_t version{ 0 };

            bool operator<(const CollapseCandidate& other) const
            {
                return cost > other.cost;
            }
        };

        struct SimplificationStats
        {
            std::size_t initialCandidateCount{ 0 };
            std::size_t collapseCount{ 0 };
            std::size_t candidateEvaluations{ 0 };
            bool reachedTimeLimit{ false };
        };

        // Bump this whenever the simplifier or its topology guarantees change.
        // Old cache entries may contain a mesh produced before the topology
        // signature validation was added.
        constexpr std::uint64_t kAdaptiveMeshCacheVersion = 4;
        constexpr std::int64_t kMaxSimplificationMilliseconds = 60000;
        constexpr std::size_t kMaxCandidateEvaluations = 10000000;
        constexpr std::uint64_t kAdaptiveMeshDiskMagic = 0x4359414441505448ULL;
        constexpr std::uint32_t kAdaptiveMeshDiskVersion = 1;
        constexpr std::size_t kMaxInMemoryAdaptiveMeshCacheEntries = 2;

        struct AdaptiveMeshDiskHeader
        {
            std::uint64_t magic{ kAdaptiveMeshDiskMagic };
            std::uint32_t version{ kAdaptiveMeshDiskVersion };
            std::uint32_t subMeshCount{ 0 };
            std::uint64_t key{ 0 };
            std::uint64_t sampleCount{ 0 };
            std::uint64_t triangleIndexCount{ 0 };
            std::uint64_t bindingSubMeshCount{ 0 };
        };

        struct AdaptiveMeshDiskGeometryHeader
        {
            std::uint64_t positionCount{ 0 };
            std::uint64_t normalCount{ 0 };
            std::uint64_t indexCount{ 0 };
        };

        std::unordered_map<std::uint64_t, std::shared_ptr<const PaintingAnalysisMeshData>>
            g_adaptiveMeshCache;
        std::mutex g_adaptiveMeshCacheMutex;

        std::filesystem::path adaptiveMeshCachePath(std::uint64_t key)
        {
            const std::filesystem::path root = simulation_project::RuntimePaths::applicationRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_adaptive_mesh" /
                (std::to_string(key) + ".bin");
        }

        template<typename Value>
        bool readRaw(std::ifstream& input, Value& value)
        {
            return static_cast<bool>(input.read(
                reinterpret_cast<char*>(&value), sizeof(Value)));
        }

        template<typename Value>
        void writeRaw(std::ofstream& output, const Value& value)
        {
            output.write(reinterpret_cast<const char*>(&value), sizeof(Value));
        }

        void writeVector3f(std::ofstream& output, const Eigen::Vector3f& value)
        {
            writeRaw(output, value.x());
            writeRaw(output, value.y());
            writeRaw(output, value.z());
        }

        bool readVector3f(std::ifstream& input, Eigen::Vector3f& value)
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if(!readRaw(input, x) || !readRaw(input, y) || !readRaw(input, z)) {
                return false;
            }
            value = Eigen::Vector3f(x, y, z);
            return true;
        }

        void writeVector3d(std::ofstream& output, const Eigen::Vector3d& value)
        {
            writeRaw(output, value.x());
            writeRaw(output, value.y());
            writeRaw(output, value.z());
        }

        bool readVector3d(std::ifstream& input, Eigen::Vector3d& value)
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            if(!readRaw(input, x) || !readRaw(input, y) || !readRaw(input, z)) {
                return false;
            }
            value = Eigen::Vector3d(x, y, z);
            return true;
        }

        void hashBytes(std::uint64_t& hash, const void* data, std::size_t size)
        {
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            for(std::size_t index = 0; index < size; ++index) {
                hash ^= bytes[index];
                hash *= 1099511628211ULL;
            }
        }

        template<typename Value>
        void hashValue(std::uint64_t& hash, const Value& value)
        {
            hashBytes(hash, &value, sizeof(Value));
        }

        void hashVector3(std::uint64_t& hash, const Eigen::Vector3d& value)
        {
            hashValue(hash, value.x());
            hashValue(hash, value.y());
            hashValue(hash, value.z());
        }

        void hashVector2(std::uint64_t& hash, const Eigen::Vector2d& value)
        {
            hashValue(hash, value.x());
            hashValue(hash, value.y());
        }

        std::uint64_t adaptiveMeshCacheKey(
            const assetcore::ModelDesc& model,
            const std::string& sourcePath,
            const Eigen::Isometry3d& worldFromModel,
            const AdaptiveMeshOptions& options)
        {
            std::uint64_t hash = 1469598103934665603ULL;
            hashValue(hash, kAdaptiveMeshCacheVersion);
            hashValue(hash, sourcePath.size());
            hashBytes(hash, sourcePath.data(), sourcePath.size());
            for(int row = 0; row < 4; ++row) {
                for(int column = 0; column < 4; ++column) {
                    hashValue(hash, worldFromModel.matrix()(row, column));
                }
            }
            hashVector3(hash, options.axisOrigin);
            hashVector3(hash, options.axisDirection);
            hashVector2(hash, options.selectionMinimum);
            hashVector2(hash, options.selectionMaximum);
            hashValue(hash, options.simplificationPercent);
            for(const Eigen::Vector2d& point : options.selectionPolygon) {
                hashVector2(hash, point);
            }
            hashValue(hash, options.selectionPolygon.size());
            hashValue(hash, model.subMeshCount());
            const glm::mat4 local = model.get_local();
            for(int column = 0; column < 4; ++column) {
                for(int row = 0; row < 4; ++row) {
                    hashValue(hash, local[column][row]);
                }
            }
            for(const auto& subMesh : model.subMeshes()) {
                const auto& geometry = subMesh.geometry;
                hashValue(hash, geometry.positions.size());
                hashValue(hash, geometry.normals.size());
                hashValue(hash, geometry.indices.size());
                if(!geometry.positions.empty()) {
                    hashBytes(hash, geometry.positions.data(),
                        geometry.positions.size() * sizeof(geometry.positions.front()));
                }
                if(!geometry.normals.empty()) {
                    hashBytes(hash, geometry.normals.data(),
                        geometry.normals.size() * sizeof(geometry.normals.front()));
                }
                if(!geometry.indices.empty()) {
                    hashBytes(hash, geometry.indices.data(),
                        geometry.indices.size() * sizeof(geometry.indices.front()));
                }
            }
            return hash;
        }

        std::optional<PaintingAnalysisMeshData> loadAdaptiveMeshDiskCache(
            std::uint64_t key,
            const assetcore::ModelDesc& sourceModel,
            const std::string& name,
            const std::string& sourcePath)
        {
            const std::filesystem::path path = adaptiveMeshCachePath(key);
            if(path.empty() || !std::filesystem::exists(path)) {
                return std::nullopt;
            }
            std::ifstream input(path, std::ios::binary);
            AdaptiveMeshDiskHeader header;
            if(!readRaw(input, header)
                || header.magic != kAdaptiveMeshDiskMagic
                || header.version != kAdaptiveMeshDiskVersion
                || header.key != key
                || header.subMeshCount != sourceModel.subMeshCount()
                || header.bindingSubMeshCount != sourceModel.subMeshCount()) {
                return std::nullopt;
            }
            constexpr std::uint64_t kMaxCachedElements = 200000000ULL;
            if(header.sampleCount > kMaxCachedElements
                || header.triangleIndexCount > kMaxCachedElements) {
                return std::nullopt;
            }

            PaintingAnalysisMeshData data;
            data.workpiece.name = name;
            data.workpiece.sourceMeshPath = sourcePath;
            data.displayModel = std::make_shared<assetcore::ModelDesc>(sourceModel);
            data.binding.sampleIndicesBySubMesh.resize(header.bindingSubMeshCount);
            data.workpiece.samples.resize(static_cast<std::size_t>(header.sampleCount));
            for(sprayworkpiece::SurfaceSample& sample : data.workpiece.samples) {
                std::uint8_t valid = 0;
                std::int32_t regionId = -1;
                if(!readVector3d(input, sample.position)
                    || !readVector3d(input, sample.normal)
                    || !readRaw(input, sample.areaWeight)
                    || !readRaw(input, regionId)
                    || !readRaw(input, sample.targetThickness)
                    || !readRaw(input, valid)) {
                    return std::nullopt;
                }
                sample.regionId = regionId;
                sample.valid = valid != 0;
            }
            data.workpiece.triangleIndices.resize(
                static_cast<std::size_t>(header.triangleIndexCount));
            if(!data.workpiece.triangleIndices.empty()
                && !input.read(reinterpret_cast<char*>(data.workpiece.triangleIndices.data()),
                    static_cast<std::streamsize>(data.workpiece.triangleIndices.size()
                        * sizeof(std::uint32_t)))) {
                return std::nullopt;
            }

            for(std::size_t subMeshIndex = 0; subMeshIndex < sourceModel.subMeshCount();
                ++subMeshIndex) {
                AdaptiveMeshDiskGeometryHeader geometryHeader;
                if(!readRaw(input, geometryHeader)
                    || geometryHeader.positionCount > kMaxCachedElements
                    || geometryHeader.normalCount > kMaxCachedElements
                    || geometryHeader.indexCount > kMaxCachedElements) {
                    return std::nullopt;
                }
                assetcore::GeometryDesc& geometry =
                    data.displayModel->subMeshes()[subMeshIndex].geometry;
                geometry.positions.resize(static_cast<std::size_t>(geometryHeader.positionCount));
                geometry.normals.resize(static_cast<std::size_t>(geometryHeader.normalCount));
                geometry.indices.resize(static_cast<std::size_t>(geometryHeader.indexCount));
                geometry.texcoords.clear();
                geometry.colors.clear();
                for(Eigen::Vector3f& value : geometry.positions) {
                    if(!readVector3f(input, value)) {
                        return std::nullopt;
                    }
                }
                for(Eigen::Vector3f& value : geometry.normals) {
                    if(!readVector3f(input, value)) {
                        return std::nullopt;
                    }
                }
                if(!geometry.indices.empty()
                    && !input.read(reinterpret_cast<char*>(geometry.indices.data()),
                        static_cast<std::streamsize>(geometry.indices.size()
                            * sizeof(std::uint32_t)))) {
                    return std::nullopt;
                }
            }

            for(std::vector<std::size_t>& binding : data.binding.sampleIndicesBySubMesh) {
                std::uint64_t count = 0;
                if(!readRaw(input, count) || count > kMaxCachedElements) {
                    return std::nullopt;
                }
                binding.resize(static_cast<std::size_t>(count));
                for(std::size_t& index : binding) {
                    std::uint64_t value = 0;
                    if(!readRaw(input, value) || value >= header.sampleCount) {
                        return std::nullopt;
                    }
                    index = static_cast<std::size_t>(value);
                }
            }
            data.warnings.push_back("Adaptive mesh disk cache hit; simplification was skipped.");
            return data;
        }

        void saveAdaptiveMeshDiskCache(
            std::uint64_t key,
            const PaintingAnalysisMeshData& data)
        {
            if(data.displayModel == nullptr) {
                return;
            }
            const std::filesystem::path path = adaptiveMeshCachePath(key);
            if(path.empty()) {
                return;
            }
            std::error_code error;
            std::filesystem::create_directories(path.parent_path(), error);
            if(error) {
                return;
            }
            std::ofstream output(path, std::ios::binary | std::ios::trunc);
            if(!output) {
                return;
            }
            AdaptiveMeshDiskHeader header;
            header.subMeshCount = static_cast<std::uint32_t>(data.displayModel->subMeshCount());
            header.key = key;
            header.sampleCount = data.workpiece.samples.size();
            header.triangleIndexCount = data.workpiece.triangleIndices.size();
            header.bindingSubMeshCount = data.binding.sampleIndicesBySubMesh.size();
            writeRaw(output, header);
            for(const sprayworkpiece::SurfaceSample& sample : data.workpiece.samples) {
                writeVector3d(output, sample.position);
                writeVector3d(output, sample.normal);
                writeRaw(output, sample.areaWeight);
                const std::int32_t regionId = sample.regionId;
                writeRaw(output, regionId);
                writeRaw(output, sample.targetThickness);
                const std::uint8_t valid = sample.valid ? 1U : 0U;
                writeRaw(output, valid);
            }
            if(!data.workpiece.triangleIndices.empty()) {
                output.write(reinterpret_cast<const char*>(data.workpiece.triangleIndices.data()),
                    static_cast<std::streamsize>(data.workpiece.triangleIndices.size()
                        * sizeof(std::uint32_t)));
            }
            for(const auto& subMesh : data.displayModel->subMeshes()) {
                const auto& geometry = subMesh.geometry;
                AdaptiveMeshDiskGeometryHeader geometryHeader;
                geometryHeader.positionCount = geometry.positions.size();
                geometryHeader.normalCount = geometry.normals.size();
                geometryHeader.indexCount = geometry.indices.size();
                writeRaw(output, geometryHeader);
                for(const Eigen::Vector3f& value : geometry.positions) {
                    writeVector3f(output, value);
                }
                for(const Eigen::Vector3f& value : geometry.normals) {
                    writeVector3f(output, value);
                }
                if(!geometry.indices.empty()) {
                    output.write(reinterpret_cast<const char*>(geometry.indices.data()),
                        static_cast<std::streamsize>(geometry.indices.size()
                            * sizeof(std::uint32_t)));
                }
            }
            for(const auto& binding : data.binding.sampleIndicesBySubMesh) {
                const std::uint64_t count = binding.size();
                writeRaw(output, count);
                for(const std::size_t index : binding) {
                    const std::uint64_t value = index;
                    writeRaw(output, value);
                }
            }
        }

        std::uint64_t edgeKey(std::size_t first, std::size_t second)
        {
            if(first > second) {
                std::swap(first, second);
            }
            return (static_cast<std::uint64_t>(first) << 32U)
                ^ static_cast<std::uint64_t>(second);
        }

        Quadric planeQuadric(
            const Eigen::Vector3d& first,
            const Eigen::Vector3d& second,
            const Eigen::Vector3d& third)
        {
            const Eigen::Vector3d cross = (second - first).cross(third - first);
            if(cross.squaredNorm() <= 1.0e-24) {
                return Quadric::Zero();
            }
            const Eigen::Vector3d normal = cross.normalized();
            Eigen::Vector4d plane;
            plane << normal, -normal.dot(first);
            return plane * plane.transpose();
        }

        Eigen::Vector3d optimalCollapsePosition(
            const SimplifyVertex& first,
            const SimplifyVertex& second,
            double& cost)
        {
            const Quadric quadric = first.quadric + second.quadric;
            Eigen::Matrix3d system = quadric.block<3, 3>(0, 0);
            Eigen::Vector3d position = (first.position + second.position) * 0.5;
            if(std::abs(system.determinant()) > 1.0e-12) {
                position = -system.inverse() * quadric.block<3, 1>(0, 3);
            }
            Eigen::Vector4d homogeneous;
            homogeneous << position, 1.0;
            cost = std::max(0.0, homogeneous.dot(quadric * homogeneous));
            return position;
        }

        void rebuildNeighbourPair(
            std::vector<SimplifyVertex>& vertices,
            std::size_t first,
            std::size_t second)
        {
            if(first == second || !vertices[first].valid || !vertices[second].valid) {
                return;
            }
            vertices[first].neighbours.insert(second);
            vertices[second].neighbours.insert(first);
        }

        bool collapseKeepsNormals(
            const std::vector<SimplifyVertex>& vertices,
            const std::vector<SimplifyFace>& faces,
            std::size_t removed,
            std::size_t kept,
            const Eigen::Vector3d& newPosition)
        {
            for(const std::size_t faceIndex : vertices[removed].incidentFaces) {
                if(faceIndex >= faces.size() || !faces[faceIndex].valid) {
                    continue;
                }
                const SimplifyFace& face = faces[faceIndex];
                std::array<std::size_t, 3> current = face.vertices;
                for(std::size_t& index : current) {
                    if(index == removed) {
                        index = kept;
                    }
                }
                if(current[0] == current[1] || current[1] == current[2]
                    || current[0] == current[2]) {
                    continue;
                }
                Eigen::Vector3d oldPosition[3];
                Eigen::Vector3d newPositions[3];
                for(int corner = 0; corner < 3; ++corner) {
                    oldPosition[corner] = vertices[face.vertices[corner]].position;
                    newPositions[corner] = vertices[current[corner]].position;
                }
                newPositions[0] = current[0] == kept ? newPosition : newPositions[0];
                newPositions[1] = current[1] == kept ? newPosition : newPositions[1];
                newPositions[2] = current[2] == kept ? newPosition : newPositions[2];
                const Eigen::Vector3d oldNormal =
                    (oldPosition[1] - oldPosition[0]).cross(oldPosition[2] - oldPosition[0]);
                const Eigen::Vector3d newNormal =
                    (newPositions[1] - newPositions[0]).cross(newPositions[2] - newPositions[0]);
                if(oldNormal.squaredNorm() <= 1.0e-24 || newNormal.squaredNorm() <= 1.0e-24
                    || oldNormal.dot(newNormal) <= 0.15 * oldNormal.norm() * newNormal.norm()) {
                    return false;
                }
            }
            return true;
        }

        bool collapsePreservesManifoldLink(
            const std::vector<SimplifyVertex>& vertices,
            std::size_t first,
            std::size_t second)
        {
            if(first >= vertices.size() || second >= vertices.size()) {
                return false;
            }
            std::size_t commonNeighbourCount = 0;
            const auto& firstNeighbours = vertices[first].neighbours;
            const auto& secondNeighbours = vertices[second].neighbours;
            for(const std::size_t neighbour : firstNeighbours) {
                if(neighbour != second
                    && secondNeighbours.find(neighbour) != secondNeighbours.end()) {
                    ++commonNeighbourCount;
                }
            }
            // A collapsible edge in a closed two-manifold has exactly the two
            // adjacent triangle vertices in its link. Boundary and non-manifold
            // edges are protected or rejected so they cannot create a bridge
            // across a hole.
            return commonNeighbourCount == 2;
        }

        struct TopologySignature
        {
            std::size_t connectedComponents{ 0 };
            std::size_t boundaryEdgeCount{ 0 };
            std::size_t boundaryLoopCount{ 0 };
            std::int64_t eulerCharacteristic{ 0 };
            double maximumEdgeLength{ 0.0 };
            bool valid{ true };
        };

        TopologySignature makeTopologySignature(
            const std::vector<SimplifyVertex>& vertices,
            const std::vector<SimplifyFace>& faces)
        {
            TopologySignature result;
            std::vector<std::size_t> parent(vertices.size());
            std::vector<bool> used(vertices.size(), false);
            std::iota(parent.begin(), parent.end(), 0U);
            const auto findRoot = [&parent](std::size_t value) {
                while(parent[value] != value) {
                    parent[value] = parent[parent[value]];
                    value = parent[value];
                }
                return value;
            };
            const auto unite = [&parent, &findRoot](std::size_t first, std::size_t second) {
                first = findRoot(first);
                second = findRoot(second);
                if(first != second) {
                    parent[second] = first;
                }
            };
            std::unordered_map<std::uint64_t, std::size_t> edgeCounts;
            std::unordered_map<std::size_t, std::unordered_set<std::size_t>> boundaryNeighbours;
            for(const SimplifyFace& face : faces) {
                if(!face.valid) {
                    continue;
                }
                if(face.vertices[0] >= vertices.size()
                    || face.vertices[1] >= vertices.size()
                    || face.vertices[2] >= vertices.size()
                    || !vertices[face.vertices[0]].valid
                    || !vertices[face.vertices[1]].valid
                    || !vertices[face.vertices[2]].valid
                    || face.vertices[0] == face.vertices[1]
                    || face.vertices[1] == face.vertices[2]
                    || face.vertices[0] == face.vertices[2]) {
                    result.valid = false;
                    continue;
                }
                const Eigen::Vector3d& first = vertices[face.vertices[0]].position;
                const Eigen::Vector3d& second = vertices[face.vertices[1]].position;
                const Eigen::Vector3d& third = vertices[face.vertices[2]].position;
                if(!first.allFinite() || !second.allFinite() || !third.allFinite()
                    || (second - first).cross(third - first).squaredNorm() <= 1.0e-24) {
                    result.valid = false;
                    continue;
                }
                const std::array<std::size_t, 3>& faceVertices = face.vertices;
                for(int corner = 0; corner < 3; ++corner) {
                    const std::size_t current = faceVertices[corner];
                    const std::size_t next = faceVertices[(corner + 1) % 3];
                    used[current] = true;
                    used[next] = true;
                    unite(current, next);
                    ++edgeCounts[edgeKey(current, next)];
                    result.maximumEdgeLength = std::max(
                        result.maximumEdgeLength,
                        (vertices[current].position - vertices[next].position).norm());
                }
            }

            std::unordered_set<std::size_t> componentRoots;
            for(std::size_t vertex = 0; vertex < used.size(); ++vertex) {
                if(used[vertex]) {
                    componentRoots.insert(findRoot(vertex));
                }
            }
            result.connectedComponents = componentRoots.size();
            const std::size_t usedVertexCount = componentRoots.empty()
                ? 0U
                : static_cast<std::size_t>(std::count(used.begin(), used.end(), true));
            const std::size_t faceCount = std::count_if(
                faces.begin(), faces.end(), [](const SimplifyFace& face) {
                    return face.valid;
                });
            for(const auto& entry : edgeCounts) {
                if(entry.second > 2) {
                    result.valid = false;
                    continue;
                }
                if(entry.second != 1) {
                    continue;
                }
                ++result.boundaryEdgeCount;
                const std::size_t first = static_cast<std::size_t>(entry.first >> 32U);
                const std::size_t second = static_cast<std::size_t>(entry.first & 0xffffffffU);
                boundaryNeighbours[first].insert(second);
                boundaryNeighbours[second].insert(first);
            }

            std::unordered_set<std::size_t> visitedBoundaryVertices;
            for(const auto& entry : boundaryNeighbours) {
                if(!visitedBoundaryVertices.insert(entry.first).second) {
                    continue;
                }
                ++result.boundaryLoopCount;
                std::vector<std::size_t> pending{ entry.first };
                while(!pending.empty()) {
                    const std::size_t current = pending.back();
                    pending.pop_back();
                    const auto neighbours = boundaryNeighbours.find(current);
                    if(neighbours == boundaryNeighbours.end()) {
                        continue;
                    }
                    for(const std::size_t next : neighbours->second) {
                        if(visitedBoundaryVertices.insert(next).second) {
                            pending.push_back(next);
                        }
                    }
                }
            }
            result.eulerCharacteristic = static_cast<std::int64_t>(usedVertexCount)
                - static_cast<std::int64_t>(edgeCounts.size())
                + static_cast<std::int64_t>(faceCount);
            return result;
        }

        bool topologySignaturesMatch(
            const TopologySignature& source,
            const TopologySignature& candidate)
        {
            // A topologically valid but geometrically stretched triangle can
            // still bridge a ring opening. Keep a generous bound for normal
            // QEM coarsening while rejecting this failure mode.
            constexpr double kMaximumAllowedEdgeStretch = 8.0;
            return source.valid && candidate.valid
                && source.connectedComponents == candidate.connectedComponents
                && source.boundaryEdgeCount == candidate.boundaryEdgeCount
                && source.boundaryLoopCount == candidate.boundaryLoopCount
                && source.eulerCharacteristic == candidate.eulerCharacteristic
                && (source.maximumEdgeLength <= 1.0e-12
                    || candidate.maximumEdgeLength
                        <= source.maximumEdgeLength * kMaximumAllowedEdgeStretch);
        }

        SimplificationStats simplifyWithQem(
            std::vector<SimplifyVertex>& vertices,
            std::vector<SimplifyFace>& faces,
            std::size_t targetVertexCount)
        {
            SimplificationStats stats;
            if(targetVertexCount >= vertices.size()) {
                return stats;
            }
            std::priority_queue<CollapseCandidate> queue;
            std::unordered_map<std::uint64_t, std::uint64_t> edgeVersions;
            edgeVersions.reserve(vertices.size());
            const auto startedAt = std::chrono::steady_clock::now();
            bool initializationStopped = false;
            const auto enqueue = [&queue, &edgeVersions, &vertices](
                std::size_t first, std::size_t second) {
                if(first >= vertices.size() || second >= vertices.size() || first == second
                    || !vertices[first].valid || !vertices[second].valid
                    || vertices[first].protectedVertex || vertices[second].protectedVertex) {
                    return;
                }
                if(vertices[first].neighbours.find(second) == vertices[first].neighbours.end()) {
                    return;
                }
                double cost = 0.0;
                optimalCollapsePosition(vertices[first], vertices[second], cost);
                const std::uint64_t key = edgeKey(first, second);
                const std::uint64_t version = ++edgeVersions[key];
                queue.push({ cost, first, second, version });
            };
            for(std::size_t first = 0; first < vertices.size(); ++first) {
                for(const std::size_t second : vertices[first].neighbours) {
                    if(first < second) {
                        enqueue(first, second);
                    }
                    if((queue.size() & 0x3fffU) == 0U
                        && std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - startedAt).count()
                            >= kMaxSimplificationMilliseconds) {
                        initializationStopped = true;
                        break;
                    }
                }
                if(initializationStopped) {
                    break;
                }
            }
            stats.initialCandidateCount = queue.size();

            std::size_t validVertexCount = vertices.size();
            while(validVertexCount > targetVertexCount && !queue.empty()) {
                ++stats.candidateEvaluations;
                if(stats.candidateEvaluations >= kMaxCandidateEvaluations
                    || std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - startedAt).count()
                        >= kMaxSimplificationMilliseconds) {
                    stats.reachedTimeLimit = true;
                    break;
                }
                const CollapseCandidate candidate = queue.top();
                queue.pop();
                const std::size_t first = candidate.first;
                const std::size_t second = candidate.second;
                const auto version = edgeVersions.find(edgeKey(first, second));
                if(first >= vertices.size() || second >= vertices.size()
                    || !vertices[first].valid || !vertices[second].valid
                    || vertices[first].protectedVertex || vertices[second].protectedVertex
                    || version == edgeVersions.end() || version->second != candidate.version
                    || vertices[first].neighbours.find(second) == vertices[first].neighbours.end()) {
                    continue;
                }
                double currentCost = 0.0;
                const Eigen::Vector3d newPosition = optimalCollapsePosition(
                    vertices[first], vertices[second], currentCost);
                if(currentCost > candidate.cost * 1.0001 + 1.0e-12
                    || !collapsePreservesManifoldLink(vertices, first, second)
                    || !collapseKeepsNormals(vertices, faces, second, first, newPosition)) {
                    continue;
                }

                const std::vector<std::size_t> affectedNeighbours(
                    vertices[second].neighbours.begin(), vertices[second].neighbours.end());
                vertices[first].position = newPosition;
                vertices[first].normal = (vertices[first].normal + vertices[second].normal).normalized();
                vertices[first].quadric += vertices[second].quadric;
                std::unordered_set<std::size_t> affectedFaces = vertices[first].incidentFaces;
                affectedFaces.insert(vertices[second].incidentFaces.begin(),
                    vertices[second].incidentFaces.end());
                for(const std::size_t faceIndex : affectedFaces) {
                    if(faceIndex >= faces.size() || !faces[faceIndex].valid) {
                        continue;
                    }
                    const auto oldFace = faces[faceIndex].vertices;
                    for(int corner = 0; corner < 3; ++corner) {
                        vertices[oldFace[corner]].neighbours.erase(oldFace[(corner + 1) % 3]);
                        vertices[oldFace[corner]].neighbours.erase(oldFace[(corner + 2) % 3]);
                    }
                    for(std::size_t& index : faces[faceIndex].vertices) {
                        if(index == second) {
                            index = first;
                        }
                    }
                    if(faces[faceIndex].vertices[0] == faces[faceIndex].vertices[1]
                        || faces[faceIndex].vertices[1] == faces[faceIndex].vertices[2]
                        || faces[faceIndex].vertices[0] == faces[faceIndex].vertices[2]) {
                        faces[faceIndex].valid = false;
                        continue;
                    }
                    const auto& updatedFace = faces[faceIndex].vertices;
                    rebuildNeighbourPair(vertices, updatedFace[0], updatedFace[1]);
                    rebuildNeighbourPair(vertices, updatedFace[1], updatedFace[2]);
                    rebuildNeighbourPair(vertices, updatedFace[2], updatedFace[0]);
                    vertices[updatedFace[0]].incidentFaces.insert(faceIndex);
                    vertices[updatedFace[1]].incidentFaces.insert(faceIndex);
                    vertices[updatedFace[2]].incidentFaces.insert(faceIndex);
                }
                for(const std::size_t neighbour : affectedNeighbours) {
                    vertices[neighbour].neighbours.erase(second);
                    if(neighbour != first) {
                        rebuildNeighbourPair(vertices, neighbour, first);
                    }
                }
                vertices[first].neighbours.erase(second);
                vertices[second].neighbours.clear();
                vertices[second].valid = false;
                --validVertexCount;
                ++stats.collapseCount;
                for(const std::size_t neighbour : vertices[first].neighbours) {
                    enqueue(std::min(first, neighbour), std::max(first, neighbour));
                }
            }
            stats.reachedTimeLimit = stats.reachedTimeLimit || initializationStopped;
            return stats;
        }
    }

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

    PaintingAnalysisMeshData buildAdaptiveUncached(
        const assetcore::ModelDesc& model,
        const std::string& name,
        const std::string& sourcePath,
        const Eigen::Isometry3d& worldFromModel,
        const AdaptiveMeshOptions& options)
    {
        PaintingAnalysisMeshData data;
        data.displayModel = std::make_shared<assetcore::ModelDesc>(model);
        data.workpiece.name = name;
        data.workpiece.sourceMeshPath = sourcePath;
        data.binding.sampleIndicesBySubMesh.resize(model.subMeshCount());
        const double ratio = std::clamp(options.simplificationPercent, 1.0, 100.0) / 100.0;
        bool usedFallbackNormal = false;

        for(std::size_t subMeshIndex = 0; subMeshIndex < model.subMeshCount(); ++subMeshIndex) {
            const assetcore::GeometryDesc& source = model.subMesh(subMeshIndex).geometry;
            assetcore::GeometryDesc& target = data.displayModel->subMeshes()[subMeshIndex].geometry;
            if(source.positions.empty()) {
                continue;
            }
            std::vector<Eigen::Vector3d> worldPositions(source.positions.size());
            std::vector<SimplifyVertex> vertices(source.positions.size());
            for(std::size_t i = 0; i < source.positions.size(); ++i) {
                const glm::vec4 local = model.get_local() * glm::vec4(
                    source.positions[i].x(), source.positions[i].y(), source.positions[i].z(), 1.0f);
                worldPositions[i] = worldFromModel * Eigen::Vector3d(local.x, local.y, local.z);
                vertices[i].position = source.positions[i].cast<double>();
                if(i < source.normals.size()) {
                    vertices[i].normal = source.normals[i].cast<double>();
                }
                if(vertices[i].normal.squaredNorm() <= 1.0e-16) {
                    vertices[i].normal = Eigen::Vector3d::UnitZ();
                    usedFallbackNormal = true;
                }
                vertices[i].protectedVertex = inAdaptiveRegion(worldPositions[i], options);
            }
            std::vector<std::uint32_t> sourceIndices = source.indices;
            if(sourceIndices.empty() && source.positions.size() % 3 == 0) {
                sourceIndices.resize(source.positions.size());
                std::iota(sourceIndices.begin(), sourceIndices.end(), 0U);
            }
            std::vector<SimplifyFace> faces;
            for(std::size_t offset = 0; offset + 2 < sourceIndices.size(); offset += 3) {
                const std::uint32_t a = sourceIndices[offset];
                const std::uint32_t b = sourceIndices[offset + 1];
                const std::uint32_t c = sourceIndices[offset + 2];
                if(a >= vertices.size() || b >= vertices.size() || c >= vertices.size()
                    || a == b || b == c || a == c) {
                    continue;
                }
                faces.push_back({ { a, b, c }, true });
            }

            struct EdgeInfo { std::size_t firstFace{ 0 }; std::size_t secondFace{ 0 }; int count{ 0 }; };
            std::unordered_map<std::uint64_t, EdgeInfo> edgeFaces;
            const auto addEdge = [&edgeFaces](std::size_t first, std::size_t second,
                std::size_t faceIndex) {
                const std::uint64_t key = edgeKey(first, second);
                EdgeInfo& info = edgeFaces[key];
                if(info.count == 0) {
                    info.firstFace = faceIndex;
                } else if(info.count == 1) {
                    info.secondFace = faceIndex;
                }
                ++info.count;
            };
            for(std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
                const auto& face = faces[faceIndex].vertices;
                const Quadric quadric = planeQuadric(
                    vertices[face[0]].position,
                    vertices[face[1]].position,
                    vertices[face[2]].position);
                vertices[face[0]].quadric += quadric;
                vertices[face[1]].quadric += quadric;
                vertices[face[2]].quadric += quadric;
                rebuildNeighbourPair(vertices, face[0], face[1]);
                rebuildNeighbourPair(vertices, face[1], face[2]);
                rebuildNeighbourPair(vertices, face[2], face[0]);
                vertices[face[0]].incidentFaces.insert(faceIndex);
                vertices[face[1]].incidentFaces.insert(faceIndex);
                vertices[face[2]].incidentFaces.insert(faceIndex);
                addEdge(face[0], face[1], faceIndex);
                addEdge(face[1], face[2], faceIndex);
                addEdge(face[2], face[0], faceIndex);
            }

            // Keep one complete triangle ring around the dense prediction area
            // so the simplified and dense regions meet without a visible seam.
            // Use a snapshot here: testing the mutable protectedVertex flags would
            // propagate protection through the whole connected mesh.
            std::vector<bool> denseRegionVertices(vertices.size(), false);
            for(std::size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex) {
                denseRegionVertices[vertexIndex] = vertices[vertexIndex].protectedVertex;
            }
            for(const SimplifyFace& face : faces) {
                if(denseRegionVertices[face.vertices[0]]
                    || denseRegionVertices[face.vertices[1]]
                    || denseRegionVertices[face.vertices[2]]) {
                    vertices[face.vertices[0]].protectedVertex = true;
                    vertices[face.vertices[1]].protectedVertex = true;
                    vertices[face.vertices[2]].protectedVertex = true;
                }
            }

            // Preserve the dense-region one-ring and sharp/boundary edges.
            for(const auto& entry : edgeFaces) {
                const std::size_t first = static_cast<std::size_t>(entry.first >> 32U);
                const std::size_t second = static_cast<std::size_t>(entry.first & 0xffffffffU);
                const EdgeInfo& info = entry.second;
                bool feature = info.count != 2;
                if(!feature) {
                    const auto& firstFace = faces[info.firstFace].vertices;
                    const auto& secondFace = faces[info.secondFace].vertices;
                    const auto normalFor = [&vertices, &faces](const std::array<std::size_t, 3>& face) {
                        return (vertices[face[1]].position - vertices[face[0]].position).cross(
                            vertices[face[2]].position - vertices[face[0]].position).normalized();
                    };
                    const Eigen::Vector3d firstNormal = normalFor(firstFace);
                    const Eigen::Vector3d secondNormal = normalFor(secondFace);
                    constexpr double kPi = 3.14159265358979323846;
                    feature = firstNormal.dot(secondNormal) < std::cos(35.0 * kPi / 180.0);
                }
                if(feature) {
                    vertices[first].protectedVertex = true;
                    vertices[second].protectedVertex = true;
                }
            }
            std::size_t protectedCount = 0;
            for(const SimplifyVertex& vertex : vertices) {
                protectedCount += vertex.protectedVertex ? 1U : 0U;
            }
            const std::size_t unprotectedCount = vertices.size() - protectedCount;
            const std::size_t targetVertexCount = std::min(vertices.size(),
                protectedCount + std::max<std::size_t>(1,
                    static_cast<std::size_t>(std::ceil(unprotectedCount * ratio))));
            const TopologySignature sourceTopology = makeTopologySignature(vertices, faces);
            const SimplificationStats simplification = simplifyWithQem(
                vertices, faces, targetVertexCount);
            std::size_t finalVertexCount = 0;
            for(const SimplifyVertex& vertex : vertices) {
                finalVertexCount += vertex.valid ? 1U : 0U;
            }
            const std::string simplificationSummary =
                "Adaptive mesh QEM: sourceVertices=" + std::to_string(source.positions.size())
                + ", sourceTriangles=" + std::to_string(faces.size())
                + ", protectedVertices=" + std::to_string(protectedCount)
                + ", initialCandidates=" + std::to_string(simplification.initialCandidateCount)
                + ", candidateEvaluations=" + std::to_string(simplification.candidateEvaluations)
                + ", collapses=" + std::to_string(simplification.collapseCount)
                + ", finalVertices=" + std::to_string(finalVertexCount)
                + (simplification.reachedTimeLimit ? ", timeLimitReached=true" : ", timeLimitReached=false");
            data.warnings.push_back(simplificationSummary);
            LOG_DEBUG("rs2026") << simplificationSummary;

            const TopologySignature simplifiedTopology = makeTopologySignature(vertices, faces);
            const bool topologyValid = topologySignaturesMatch(
                sourceTopology, simplifiedTopology);
            if(!topologyValid) {
                target = source;
                data.warnings.push_back(
                    "Adaptive mesh QEM output failed topology validation; original submesh was retained.");
                LOG_DEBUG("rs2026")
                    << "Adaptive mesh QEM topology validation failed; retaining source submesh."
                    << " source(components=" << sourceTopology.connectedComponents
                    << ", boundaryEdges=" << sourceTopology.boundaryEdgeCount
                    << ", boundaryLoops=" << sourceTopology.boundaryLoopCount
                    << ", euler=" << sourceTopology.eulerCharacteristic << ")"
                    << " candidate(components=" << simplifiedTopology.connectedComponents
                    << ", boundaryEdges=" << simplifiedTopology.boundaryEdgeCount
                    << ", boundaryLoops=" << simplifiedTopology.boundaryLoopCount
                    << ", euler=" << simplifiedTopology.eulerCharacteristic << ")";
            } else {
                std::vector<std::uint32_t> remap(source.positions.size(), 0);
                target.positions.clear();
                target.normals.clear();
                target.texcoords.clear();
                target.colors.clear();
                target.indices.clear();
                for(std::size_t index = 0; index < vertices.size(); ++index) {
                    if(!vertices[index].valid) {
                        continue;
                    }
                    const std::uint32_t output = static_cast<std::uint32_t>(target.positions.size());
                    target.positions.push_back(vertices[index].position.cast<float>());
                    target.normals.push_back(vertices[index].normal.normalized().cast<float>());
                    remap[index] = output;
                }
                for(const SimplifyFace& face : faces) {
                    if(!face.valid || !vertices[face.vertices[0]].valid
                        || !vertices[face.vertices[1]].valid || !vertices[face.vertices[2]].valid) {
                        continue;
                    }
                    const std::uint32_t a = remap[face.vertices[0]];
                    const std::uint32_t b = remap[face.vertices[1]];
                    const std::uint32_t c = remap[face.vertices[2]];
                    if(a == b || b == c || a == c) {
                        continue;
                    }
                    target.indices.push_back(a);
                    target.indices.push_back(b);
                    target.indices.push_back(c);
                }
            }
            std::vector<std::size_t>& binding = data.binding.sampleIndicesBySubMesh[subMeshIndex];
            binding.resize(target.positions.size());
            const std::size_t sampleOffset = data.workpiece.samples.size();
            for(std::size_t i = 0; i < target.positions.size(); ++i) {
                sprayworkpiece::SurfaceSample sample;
                const glm::vec4 localPosition = model.get_local() * glm::vec4(
                    target.positions[i].x(), target.positions[i].y(), target.positions[i].z(), 1.0f);
                sample.position = worldFromModel * Eigen::Vector3d(
                    static_cast<double>(localPosition.x), static_cast<double>(localPosition.y), static_cast<double>(localPosition.z));
                sample.normal = target.normals[i].cast<double>().normalized();
                sample.valid = inAdaptiveRegion(sample.position, options);
                binding[i] = data.workpiece.samples.size();
                data.workpiece.addSample(sample);
            }
            for(const std::uint32_t index : target.indices) {
                data.workpiece.triangleIndices.push_back(static_cast<std::uint32_t>(sampleOffset + index));
            }
        }
        if(usedFallbackNormal) {
            data.warnings.push_back("Some adaptive mesh vertices had no valid normal; +Z was used.");
        }
        return data;
    }

    PaintingAnalysisMeshData PaintingAnalysisMeshAdapter::buildAdaptive(
        const assetcore::ModelDesc& model,
        const std::string& name,
        const std::string& sourcePath,
        const Eigen::Isometry3d& worldFromModel,
        const AdaptiveMeshOptions& options)
    {
        const auto cacheKey = adaptiveMeshCacheKey(model, sourcePath, worldFromModel, options);
        {
            std::lock_guard<std::mutex> lock(g_adaptiveMeshCacheMutex);
            const auto found = g_adaptiveMeshCache.find(cacheKey);
            if(found != g_adaptiveMeshCache.end() && found->second != nullptr) {
                LOG_DEBUG("rs2026") << "Adaptive mesh cache hit: key=" << cacheKey;
                PaintingAnalysisMeshData cached = *found->second;
                cached.warnings.insert(cached.warnings.begin(),
                    "Adaptive mesh cache hit; simplification was skipped.");
                return cached;
            }
        }

        if(auto diskCached = loadAdaptiveMeshDiskCache(
            cacheKey, model, name, sourcePath)) {
            LOG_DEBUG("rs2026") << "Adaptive mesh disk cache hit: key=" << cacheKey;
            PaintingAnalysisMeshData cached = std::move(*diskCached);
            std::lock_guard<std::mutex> lock(g_adaptiveMeshCacheMutex);
            if(g_adaptiveMeshCache.size() >= kMaxInMemoryAdaptiveMeshCacheEntries) {
                g_adaptiveMeshCache.erase(g_adaptiveMeshCache.begin());
            }
            g_adaptiveMeshCache[cacheKey] =
                std::make_shared<const PaintingAnalysisMeshData>(cached);
            return cached;
        }

        const auto startedAt = std::chrono::steady_clock::now();
        PaintingAnalysisMeshData result = buildAdaptiveUncached(
            model, name, sourcePath, worldFromModel, options);
        const double elapsedMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - startedAt).count();
        const std::string timing = "Adaptive mesh total: "
            + std::to_string(elapsedMilliseconds) + " ms";
        result.warnings.push_back(timing);
        LOG_DEBUG("rs2026") << timing << ", cacheKey=" << cacheKey;

        {
            std::lock_guard<std::mutex> lock(g_adaptiveMeshCacheMutex);
            if(g_adaptiveMeshCache.size() >= kMaxInMemoryAdaptiveMeshCacheEntries) {
                g_adaptiveMeshCache.erase(g_adaptiveMeshCache.begin());
            }
            g_adaptiveMeshCache[cacheKey] =
                std::make_shared<const PaintingAnalysisMeshData>(result);
        }
        saveAdaptiveMeshDiskCache(cacheKey, result);
        return result;
    }

    std::vector<std::uint32_t> PaintingAnalysisMeshAdapter::selectVerticesInRegion(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const AdaptiveMeshOptions& options)
    {
        std::vector<std::uint32_t> indices;
        indices.reserve(workpiece.samples.size());
        for(std::size_t index = 0; index < workpiece.samples.size(); ++index) {
            if(inAdaptiveRegion(workpiece.samples[index].position, options)) {
                indices.push_back(static_cast<std::uint32_t>(index));
            }
        }
        return indices;
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

    smrobot::visualization::SurfaceScalarOverlay
    PaintingAnalysisMeshAdapter::makeRelativeErrorOverlay(
        const std::string& objectId,
        const PaintingAnalysisMeshBinding& binding,
        const spraythickness::ThicknessField& reference,
        const spraythickness::ThicknessField& candidate,
        const std::vector<std::uint8_t>* comparisonMask)
    {
        constexpr double kReferenceFloorMeters = 1.0e-12;
        constexpr double kRelativeErrorColorLimitPercent = 5.0;
        smrobot::visualization::SurfaceScalarOverlay overlay;
        overlay.objectId = objectId;
        overlay.quantityName = "Relative error";
        overlay.unit = "%";
        overlay.range.minimum = 0.0;
        overlay.range.maximum = kRelativeErrorColorLimitPercent;
        overlay.colorMap = smrobot::visualization::ScalarColorMap({
            { 0.000, Eigen::Vector3f(0.0f, 0.0f, 0.5f) },
            { 0.125, Eigen::Vector3f(0.0f, 0.0f, 1.0f) },
            { 0.250, Eigen::Vector3f(0.0f, 0.5f, 1.0f) },
            { 0.375, Eigen::Vector3f(0.0f, 1.0f, 1.0f) },
            { 0.500, Eigen::Vector3f(0.5f, 1.0f, 0.5f) },
            { 0.625, Eigen::Vector3f(1.0f, 1.0f, 0.0f) },
            { 0.750, Eigen::Vector3f(1.0f, 0.5f, 0.0f) },
            { 0.875, Eigen::Vector3f(1.0f, 0.0f, 0.0f) },
            { 1.000, Eigen::Vector3f(0.5f, 0.0f, 0.0f) }
        });
        overlay.subMeshes.resize(binding.sampleIndicesBySubMesh.size());

        for(std::size_t subMeshIndex = 0;
            subMeshIndex < binding.sampleIndicesBySubMesh.size();
            ++subMeshIndex) {
            const std::vector<std::size_t>& sampleIndices =
                binding.sampleIndicesBySubMesh[subMeshIndex];
            std::vector<double>& values = overlay.subMeshes[subMeshIndex].values;
            values.reserve(sampleIndices.size());
            for(const std::size_t sampleIndex : sampleIndices) {
                if(sampleIndex >= reference.results.size()
                    || sampleIndex >= candidate.results.size()) {
                    throw std::runtime_error(
                        "Relative error sample binding is out of range.");
                }
                if(comparisonMask != nullptr
                    && (sampleIndex >= comparisonMask->size()
                        || (*comparisonMask)[sampleIndex] == 0U)) {
                    // The renderer requires one value per mesh vertex. Keep
                    // excluded vertices at the lower color-map bound; they
                    // are excluded from all numerical validation statistics.
                    values.push_back(overlay.range.minimum);
                    continue;
                }
                const double referenceValue = reference.results[sampleIndex].thickness;
                const double candidateValue = candidate.results[sampleIndex].thickness;
                const double referenceMagnitude = std::abs(referenceValue);
                const double absoluteError = std::abs(candidateValue - referenceValue);
                const double relativeError = referenceMagnitude > kReferenceFloorMeters
                    ? absoluteError / referenceMagnitude
                    : (std::abs(candidateValue) > kReferenceFloorMeters ? 1.0 : 0.0);
                const double relativeErrorPercent = relativeError * 100.0;
                values.push_back(relativeErrorPercent);
            }
        }
        return overlay;
    }
}
