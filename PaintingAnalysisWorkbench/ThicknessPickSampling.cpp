#include "ThicknessPickSampling.h"

#include <AssetCore/ModelDesc.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double kMaximumSpacingMeters = 0.0002;
        constexpr std::size_t kMaximumSampleCount = 200000;

        struct IntersectionPoint
        {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            double thickness = 0.0;
            Eigen::Vector3d normal = Eigen::Vector3d::Zero();
        };

        class SampleCollector
        {
        public:
            SampleCollector(double tolerance, const Eigen::Vector3d& minimum)
                : m_tolerance(tolerance), m_minimum(minimum)
            {
            }

            void add(std::size_t triangleIndex, IntersectionPoint point)
            {
                point.position.x() = 0.0;
                if(!point.position.allFinite() || !std::isfinite(point.thickness)
                    || point.thickness < 0.0) {
                    throw std::runtime_error(
                        "YZ thickness sampling produced a non-finite or negative value.");
                }
                const auto key = pointKey(point.position);
                for(std::int64_t dy = -1; dy <= 1; ++dy) {
                    for(std::int64_t dz = -1; dz <= 1; ++dz) {
                        const auto found = m_buckets.find(
                            { key.first + dy, key.second + dz });
                        if(found == m_buckets.end()) {
                            continue;
                        }
                        for(const std::size_t index : found->second) {
                            const auto& existing = m_samples[index];
                            const double thicknessTolerance = 32.0
                                * std::numeric_limits<double>::epsilon()
                                * std::max(std::abs(existing.thicknessMeters),
                                    std::abs(point.thickness));
                            if((existing.positionMeters - point.position).stableNorm()
                                    <= m_tolerance
                                && std::abs(existing.thicknessMeters - point.thickness)
                                    <= thicknessTolerance) {
                                return;
                            }
                        }
                    }
                }
                if(m_samples.size() >= kMaximumSampleCount) {
                    throw std::runtime_error(
                        "The YZ thickness report exceeds the limit of 200000 points.");
                }
                ThicknessPickReportSample sample;
                sample.triangleIndex = triangleIndex;
                sample.positionMeters = point.position;
                sample.thicknessMeters = point.thickness;
                m_buckets[key].push_back(m_samples.size());
                m_samples.push_back(std::move(sample));
            }

            void addSegment(std::size_t triangleIndex,
                const IntersectionPoint& first, const IntersectionPoint& second,
                bool = false)
            {
                const double length = (second.position - first.position).stableNorm();
                if(!std::isfinite(length)) {
                    throw std::runtime_error("The YZ intersection length is non-finite.");
                }
                const double intervalCount = std::ceil(length / kMaximumSpacingMeters);
                if(intervalCount >= static_cast<double>(kMaximumSampleCount)) {
                    throw std::runtime_error(
                        "A YZ intersection exceeds the limit of 200000 points.");
                }
                const std::size_t intervals = std::max<std::size_t>(1,
                    static_cast<std::size_t>(intervalCount));
                for(std::size_t index = 0; index <= intervals; ++index) {
                    const double ratio = static_cast<double>(index)
                        / static_cast<double>(intervals);
                    add(triangleIndex, {
                        (1.0 - ratio) * first.position + ratio * second.position,
                        (1.0 - ratio) * first.thickness + ratio * second.thickness });
                }
            }

            std::vector<ThicknessPickReportSample> finish()
            {
                return std::move(m_samples);
            }

        private:
            using PointKey = std::pair<std::int64_t, std::int64_t>;

            PointKey pointKey(const Eigen::Vector3d& position) const
            {
                // Subtract the mesh minimum so large world translations do not
                // overflow the integer key of a small part.
                const double y = std::floor(
                    (position.y() - m_minimum.y()) / m_tolerance);
                const double z = std::floor(
                    (position.z() - m_minimum.z()) / m_tolerance);
                const double maximum = static_cast<double>(
                    std::numeric_limits<std::int64_t>::max() / 2);
                if(!std::isfinite(y) || !std::isfinite(z)
                    || std::abs(y) > maximum || std::abs(z) > maximum) {
                    throw std::runtime_error("YZ thickness point coordinates are out of range.");
                }
                return { static_cast<std::int64_t>(y), static_cast<std::int64_t>(z) };
            }

            double m_tolerance = 0.0;
            Eigen::Vector3d m_minimum = Eigen::Vector3d::Zero();
            std::map<PointKey, std::vector<std::size_t>> m_buckets;
            std::vector<ThicknessPickReportSample> m_samples;
        };

        int planeSide(double x, double tolerance)
        {
            return x > tolerance ? 1 : x < -tolerance ? -1 : 0;
        }

        template<class Collector>
        void sampleTriangle(std::size_t triangleIndex,
            std::array<IntersectionPoint, 3> corners,
            double planeTolerance, double positionTolerance,
            Collector& collector)
        {
            const Eigen::Vector3d cross = (corners[1].position - corners[0].position)
                .cross(corners[2].position - corners[0].position);
            const double crossLength = cross.stableNorm();
            const Eigen::Vector3d faceNormal = crossLength > 0.0
                ? Eigen::Vector3d(cross / crossLength) : Eigen::Vector3d::Zero();
            for(auto& corner : corners) corner.normal = faceNormal;
            const std::array<int, 3> sides{
                planeSide(corners[0].position.x(), planeTolerance),
                planeSide(corners[1].position.x(), planeTolerance),
                planeSide(corners[2].position.x(), planeTolerance) };
            if(sides[0] == 0 && sides[1] == 0 && sides[2] == 0) {
                for(std::size_t edge = 0; edge < 3; ++edge) {
                    collector.addSegment(triangleIndex,
                        corners[edge], corners[(edge + 1) % 3], true);
                }
                return;
            }
            std::vector<IntersectionPoint> intersections;
            intersections.reserve(3);
            const auto append = [&](IntersectionPoint point) {
                point.position.x() = 0.0;
                const auto existing = std::find_if(
                    intersections.begin(), intersections.end(),
                    [&](const IntersectionPoint& value) {
                        return (value.position - point.position).stableNorm()
                            <= positionTolerance;
                    });
                if(existing == intersections.end()) {
                    intersections.push_back(std::move(point));
                }
            };
            for(std::size_t corner = 0; corner < 3; ++corner) {
                if(sides[corner] == 0) {
                    append(corners[corner]);
                }
                const std::size_t next = (corner + 1) % 3;
                if(sides[corner] * sides[next] < 0) {
                    const double x = corners[corner].position.x();
                    const double nextX = corners[next].position.x();
                    const double ratio = x / (x - nextX);
                    append({
                        (1.0 - ratio) * corners[corner].position
                            + ratio * corners[next].position,
                        (1.0 - ratio) * corners[corner].thickness
                            + ratio * corners[next].thickness, faceNormal });
                }
            }
            if(intersections.size() == 1) {
                collector.add(triangleIndex, intersections.front());
            } else if(intersections.size() == 2) {
                collector.addSegment(triangleIndex,
                    intersections[0], intersections[1]);
            } else if(intersections.size() > 2) {
                throw std::runtime_error("The YZ triangle intersection is inconsistent.");
            }
        }
    }

    namespace
    {
        using SectionSegment = ThicknessPickSectionSegment;
        using ReportSample = ThicknessPickReportSample;

        double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
        {
            return a.x() * b.y() - a.y() * b.x();
        }

        ReportSample interpolateSample(const ReportSample& a, const ReportSample& b, double t)
        {
            ReportSample result = a;
            result.positionMeters = (1.0 - t) * a.positionMeters + t * b.positionMeters;
            result.positionMeters.x() = 0.0;
            result.thicknessMeters = (1.0 - t) * a.thicknessMeters + t * b.thicknessMeters;
            result.normalWorld = (1.0 - t) * a.normalWorld + t * b.normalWorld;
            result.normalWorld.x() = 0.0;
            const double normalLength = result.normalWorld.stableNorm();
            if(normalLength > 0.0) result.normalWorld /= normalLength;
            result.arcLengthMeters = (1.0 - t) * a.arcLengthMeters + t * b.arcLengthMeters;
            return result;
        }

        bool sameThickness(double a, double b)
        {
            return std::abs(a - b) <= 64.0 * std::numeric_limits<double>::epsilon()
                * std::max({ 1.0e-12, std::abs(a), std::abs(b) });
        }

        class PositionNodes
        {
        public:
            PositionNodes(double tolerance, const Eigen::Vector3d& minimum)
                : m_tolerance(tolerance), m_minimum(minimum) {}

            std::size_t add(const Eigen::Vector3d& position)
            {
                const Eigen::Vector3d shifted = (position - m_minimum) / m_tolerance;
                const double limit = static_cast<double>(std::numeric_limits<std::int64_t>::max() / 4);
                if(!shifted.allFinite() || std::abs(shifted.y()) > limit || std::abs(shifted.z()) > limit) {
                    throw std::runtime_error("YZ section coordinates exceed the supported precision.");
                }
                const Key key{ static_cast<std::int64_t>(std::floor(shifted.y())),
                    static_cast<std::int64_t>(std::floor(shifted.z())) };
                for(std::int64_t y = -1; y <= 1; ++y) {
                    for(std::int64_t z = -1; z <= 1; ++z) {
                        const auto found = m_buckets.find({ key.first + y, key.second + z });
                        if(found == m_buckets.end()) continue;
                        for(const auto index : found->second) {
                            if((positions[index] - position).stableNorm() <= m_tolerance) return index;
                        }
                    }
                }
                if(positions.size() >= 2 * kMaximumSampleCount) {
                    throw std::runtime_error("The YZ section exceeds the limit of 200000 segments.");
                }
                const auto index = positions.size();
                positions.push_back(position);
                m_buckets[key].push_back(index);
                return index;
            }

            std::vector<Eigen::Vector3d> positions;

        private:
            using Key = std::pair<std::int64_t, std::int64_t>;
            double m_tolerance;
            Eigen::Vector3d m_minimum;
            std::map<Key, std::vector<std::size_t>> m_buckets;
        };

        class SegmentCollector
        {
        public:
            SegmentCollector(double tolerance, const Eigen::Vector3d& minimum)
                : m_tolerance(tolerance), m_nodes(tolerance, minimum) {}

            void add(std::size_t, const IntersectionPoint&) {}

            void addSegment(std::size_t triangle, const IntersectionPoint& a,
                const IntersectionPoint& b, bool coplanar = false)
            {
                if((b.position - a.position).stableNorm() <= m_tolerance) return;
                SectionSegment segment;
                segment.first = { triangle, a.position, a.thickness };
                segment.second = { triangle, b.position, b.thickness };
                segment.first.positionMeters.x() = segment.second.positionMeters.x() = 0.0;
                Eigen::Vector3d normal = a.normal;
                if(coplanar) normal = (b.position - a.position).cross(a.normal);
                normal.x() = 0.0;
                const double length = normal.stableNorm();
                if(length > 0.0) normal /= length;
                segment.first.normalWorld = segment.second.normalWorld = normal;
                if(m_segments.size() + m_coplanar.size() >= kMaximumSampleCount) {
                    throw std::runtime_error("The YZ section exceeds the limit of 200000 segments.");
                }
                if(!coplanar) {
                    m_segments.push_back(std::move(segment));
                    return;
                }
                const auto first = m_nodes.add(segment.first.positionMeters);
                const auto second = m_nodes.add(segment.second.positionMeters);
                const auto key = std::minmax(first, second);
                auto& boundary = m_coplanar[{ key.first, key.second }];
                boundary.first = std::move(segment);
                if(++boundary.second > 2) {
                    throw std::runtime_error("The YZ coplanar mesh contains a non-manifold edge.");
                }
            }

            std::vector<SectionSegment> finish()
            {
                // A mesh cut face lying on X=0 can share its boundary with the
                // actual coated side. The side intersection owns that boundary;
                // its scalar must not be mixed with the coplanar cut-face value.
                std::set<std::pair<std::size_t, std::size_t>> sideEdges;
                for(const auto& segment : m_segments) {
                    const auto a = m_nodes.add(segment.first.positionMeters);
                    const auto b = m_nodes.add(segment.second.positionMeters);
                    const auto ordered = std::minmax(a, b);
                    sideEdges.insert({ ordered.first, ordered.second });
                }
                for(const auto& entry : m_coplanar) {
                    if(entry.second.second == 1 && sideEdges.count(entry.first) == 0) {
                        m_segments.push_back(entry.second.first);
                    }
                }
                return std::move(m_segments);
            }

        private:
            double m_tolerance;
            PositionNodes m_nodes;
            std::vector<SectionSegment> m_segments;
            std::map<std::pair<std::size_t, std::size_t>,
                std::pair<SectionSegment, std::size_t>> m_coplanar;
        };

        struct SectionBounds
        {
            Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
            Eigen::Vector3d maximum = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
            double tolerance = 0.0;
        };

        SectionBounds validateSegments(const std::vector<SectionSegment>& segments)
        {
            if(segments.size() > kMaximumSampleCount) {
                throw std::runtime_error("The YZ section exceeds the limit of 200000 segments.");
            }
            SectionBounds result;
            for(const auto& segment : segments) {
                for(const auto* sample : { &segment.first, &segment.second }) {
                    if(!sample->positionMeters.allFinite() || sample->positionMeters.x() != 0.0
                        || !std::isfinite(sample->thicknessMeters) || sample->thicknessMeters < 0.0
                        || !sample->normalWorld.allFinite()) {
                        throw std::runtime_error("The YZ section contains invalid coordinates or thickness.");
                    }
                    result.minimum = result.minimum.cwiseMin(sample->positionMeters);
                    result.maximum = result.maximum.cwiseMax(sample->positionMeters);
                }
            }
            if(!segments.empty()) {
                const double scale = (result.maximum - result.minimum).stableNorm();
                if(!std::isfinite(scale) || scale <= 0.0) {
                    throw std::runtime_error("The YZ section has zero or invalid length.");
                }
                result.tolerance = scale * 1.0e-9;
                if(result.tolerance <= 0.0) throw std::runtime_error("The YZ section is below coordinate precision.");
            }
            return result;
        }

        bool insidePolygon(const Eigen::Vector2d& point,
            const std::vector<Eigen::Vector2d>& polygon, double tolerance)
        {
            bool inside = false;
            for(std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
                const Eigen::Vector2d delta = polygon[i] - polygon[j];
                const Eigen::Vector2d offset = point - polygon[j];
                const double length = delta.stableNorm();
                if(length > 0.0 && std::abs(cross2(delta, offset)) <= tolerance * length
                    && offset.dot(delta) >= -tolerance * length
                    && offset.dot(delta) <= delta.squaredNorm() + tolerance * length) return true;
                if((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())
                    && point.x() < polygon[i].x() + (point.y() - polygon[i].y())
                        * (polygon[j].x() - polygon[i].x()) / (polygon[j].y() - polygon[i].y())) inside = !inside;
            }
            return inside;
        }
    }

    template<class Collector>
    auto collectYzThickness(
        const assetcore::ModelDesc& model,
        const smrobot::visualization::SurfaceScalarOverlay& thicknessOverlay,
        const Eigen::Isometry3d& worldFromModel)
    {
        if(!worldFromModel.matrix().allFinite()
            || (worldFromModel.matrix().row(3)
                    - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1.0e-6) {
            throw std::runtime_error("The workpiece world transform is invalid.");
        }
        if(thicknessOverlay.unit != "m") {
            throw std::runtime_error("The report requires a thickness field in meters.");
        }
        if(model.subMeshCount() == 0
            || thicknessOverlay.subMeshes.size() != model.subMeshCount()) {
            throw std::runtime_error("The thickness field does not match the display mesh.");
        }

        // The packaged AssetCore SDK exposes only a mutable transform accessor.
        const glm::mat4 local = const_cast<assetcore::ModelDesc&>(model).get_local();
        Eigen::Matrix4d modelLocal;
        for(int row = 0; row < 4; ++row) {
            for(int column = 0; column < 4; ++column) {
                modelLocal(row, column) = static_cast<double>(local[column][row]);
            }
        }
        if(!modelLocal.allFinite()
            || (modelLocal.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm()
                > 1.0e-6) {
            throw std::runtime_error("The display mesh has an invalid local transform.");
        }
        const Eigen::Matrix4d world = worldFromModel.matrix() * modelLocal;
        Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::infinity());
        Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
            -std::numeric_limits<double>::infinity());
        std::vector<std::vector<Eigen::Vector3d>> positions(model.subMeshCount());
        std::size_t totalTriangles = 0;
        double maximumXRoundoff = 0.0;
        for(std::size_t subMesh = 0; subMesh < model.subMeshCount(); ++subMesh) {
            const auto& geometry = model.subMesh(subMesh).geometry;
            const auto& values = thicknessOverlay.subMeshes[subMesh].values;
            if(values.size() != geometry.positions.size()
                || (!geometry.indices.empty() && geometry.indices.size() % 3 != 0)
                || (geometry.indices.empty() && geometry.positions.size() % 3 != 0)) {
                throw std::runtime_error("The display mesh or thickness array is incomplete.");
            }
            positions[subMesh].reserve(geometry.positions.size());
            for(std::size_t vertex = 0; vertex < geometry.positions.size(); ++vertex) {
                if(!geometry.positions[vertex].allFinite()
                    || !std::isfinite(values[vertex]) || values[vertex] < 0.0) {
                    throw std::runtime_error(
                        "The display mesh contains non-finite coordinates or invalid thickness.");
                }
                const Eigen::Vector4d localPosition(
                    geometry.positions[vertex].x(), geometry.positions[vertex].y(),
                    geometry.positions[vertex].z(), 1.0);
                const Eigen::Vector3d position = (world * localPosition).head<3>();
                if(!position.allFinite()) {
                    throw std::runtime_error("The workpiece world coordinates are non-finite.");
                }
                // Account for cancellation in both affine matrix products.
                // A fixed tolerance in meters would absorb very small parts.
                const Eigen::Vector4d localMagnitude =
                    modelLocal.cwiseAbs() * localPosition.cwiseAbs();
                const double xMagnitude = worldFromModel.matrix().row(0)
                    .cwiseAbs().dot(localMagnitude);
                const double xRoundoff = 32.0
                    * std::numeric_limits<double>::epsilon() * xMagnitude;
                if(!std::isfinite(xRoundoff)) {
                    throw std::runtime_error("The workpiece transform precision is out of range.");
                }
                maximumXRoundoff = std::max(maximumXRoundoff, xRoundoff);
                positions[subMesh].push_back(position);
                minimum = minimum.cwiseMin(position);
                maximum = maximum.cwiseMax(position);
            }
            for(const std::uint32_t index : geometry.indices) {
                if(index >= geometry.positions.size()) {
                    throw std::runtime_error("The display mesh has an out-of-range triangle index.");
                }
            }
            totalTriangles += (geometry.indices.empty()
                ? geometry.positions.size() : geometry.indices.size()) / 3;
        }
        const double meshScale = (maximum - minimum).stableNorm();
        if(totalTriangles == 0 || !std::isfinite(meshScale) || meshScale <= 0.0) {
            throw std::runtime_error("The workpiece has no valid triangle mesh.");
        }
        const double positionTolerance = meshScale * 1.0e-9;
        if(positionTolerance <= 0.0) {
            throw std::runtime_error("The workpiece is below the supported coordinate precision.");
        }
        const double planeTolerance = std::max(positionTolerance, maximumXRoundoff);
        Collector collector(positionTolerance, minimum);
        std::size_t triangleIndex = 0;
        for(std::size_t subMesh = 0; subMesh < model.subMeshCount(); ++subMesh) {
            const auto& geometry = model.subMesh(subMesh).geometry;
            const auto& values = thicknessOverlay.subMeshes[subMesh].values;
            const std::size_t count = geometry.indices.empty()
                ? geometry.positions.size() : geometry.indices.size();
            for(std::size_t offset = 0; offset < count; offset += 3, ++triangleIndex) {
                std::array<IntersectionPoint, 3> corners;
                for(std::size_t corner = 0; corner < 3; ++corner) {
                    const std::size_t vertex = geometry.indices.empty()
                        ? offset + corner : geometry.indices[offset + corner];
                    corners[corner] = { positions[subMesh][vertex], values[vertex] };
                }
                sampleTriangle(triangleIndex, corners,
                    planeTolerance, positionTolerance, collector);
            }
        }
        auto samples = collector.finish();
        if(samples.empty()) {
            throw std::runtime_error("The workpiece does not intersect the world YZ plane (X = 0).");
        }
        return samples;
    }

    std::vector<ThicknessPickReportSample> sampleYzThickness(
        const assetcore::ModelDesc& model,
        const smrobot::visualization::SurfaceScalarOverlay& thicknessOverlay,
        const Eigen::Isometry3d& worldFromModel)
    {
        return collectYzThickness<SampleCollector>(model, thicknessOverlay, worldFromModel);
    }

    std::vector<ThicknessPickSectionSegment> extractYzThicknessSegments(
        const assetcore::ModelDesc& model,
        const smrobot::visualization::SurfaceScalarOverlay& thicknessOverlay,
        const Eigen::Isometry3d& worldFromModel)
    {
        return collectYzThickness<SegmentCollector>(model, thicknessOverlay, worldFromModel);
    }

    std::vector<ThicknessPickSectionSegment> clipYzThicknessSegments(
        const std::vector<ThicknessPickSectionSegment>& segments,
        const std::vector<Eigen::Vector2d>& polygon)
    {
        if(polygon.size() < 3 || polygon.size() > 10000) {
            throw std::runtime_error("Select a closed polygon around the desired YZ surface section.");
        }
        Eigen::Vector2d minimum = polygon.front(), maximum = polygon.front();
        for(const auto& vertex : polygon) {
            if(!vertex.allFinite()) throw std::runtime_error("The section selection polygon is invalid.");
            minimum = minimum.cwiseMin(vertex);
            maximum = maximum.cwiseMax(vertex);
        }
        const double polygonScale = (maximum - minimum).stableNorm();
        if(!std::isfinite(polygonScale) || polygonScale <= 0.0) {
            throw std::runtime_error("The section selection polygon is invalid.");
        }
        double twiceArea = 0.0;
        for(std::size_t i = 0; i < polygon.size(); ++i) {
            twiceArea += cross2(polygon[i] - minimum, polygon[(i + 1) % polygon.size()] - minimum);
        }
        if(!std::isfinite(twiceArea) || std::abs(twiceArea) <= polygonScale * polygonScale * 1.0e-12) {
            throw std::runtime_error("The section selection polygon has no usable area.");
        }
        const auto bounds = validateSegments(segments);
        const double tolerance = std::max(bounds.tolerance, polygonScale * 1.0e-12);
        std::vector<SectionSegment> result;
        for(const auto& segment : segments) {
            const Eigen::Vector2d a = segment.first.positionMeters.tail<2>();
            const Eigen::Vector2d delta = segment.second.positionMeters.tail<2>() - a;
            const double length = delta.stableNorm();
            if(length <= tolerance) continue;
            if(a.cwiseMax(a + delta).x() < minimum.x() - tolerance
                || a.cwiseMax(a + delta).y() < minimum.y() - tolerance
                || a.cwiseMin(a + delta).x() > maximum.x() + tolerance
                || a.cwiseMin(a + delta).y() > maximum.y() + tolerance) continue;
            std::vector<double> parameters{ 0.0, 1.0 };
            for(std::size_t i = 0; i < polygon.size(); ++i) {
                const Eigen::Vector2d edge = polygon[(i + 1) % polygon.size()] - polygon[i];
                const Eigen::Vector2d offset = polygon[i] - a;
                const double denominator = cross2(delta, edge);
                const double edgeLength = edge.stableNorm();
                if(std::abs(denominator) <= 32.0 * std::numeric_limits<double>::epsilon() * length * edgeLength) {
                    if(std::abs(cross2(offset, delta)) <= tolerance * length) {
                        for(const auto& vertex : { polygon[i], polygon[(i + 1) % polygon.size()] }) {
                            parameters.push_back(std::clamp((vertex - a).dot(delta) / delta.squaredNorm(), 0.0, 1.0));
                        }
                    }
                    continue;
                }
                const double t = cross2(offset, edge) / denominator;
                const double u = cross2(offset, delta) / denominator;
                if(t >= -1.0e-12 && t <= 1.0 + 1.0e-12 && u >= -1.0e-12 && u <= 1.0 + 1.0e-12) {
                    parameters.push_back(std::clamp(t, 0.0, 1.0));
                }
            }
            std::sort(parameters.begin(), parameters.end());
            parameters.erase(std::unique(parameters.begin(), parameters.end(),
                [](double a, double b) { return std::abs(a - b) < 1.0e-12; }), parameters.end());
            for(std::size_t i = 1; i < parameters.size(); ++i) {
                const double first = parameters[i - 1], second = parameters[i];
                if((second - first) * length <= tolerance
                    || !insidePolygon(a + ((first + second) * 0.5) * delta, polygon, tolerance)) continue;
                if(result.size() >= kMaximumSampleCount) {
                    throw std::runtime_error("The selected YZ section exceeds the limit of 200000 segments.");
                }
                result.push_back({ interpolateSample(segment.first, segment.second, first),
                    interpolateSample(segment.first, segment.second, second) });
            }
        }
        return result;
    }

    ThicknessPickReportSection buildSelectedYzThicknessSection(
        const std::vector<ThicknessPickSectionSegment>& segments,
        const std::vector<Eigen::Vector2d>& polygon,
        double annotationSpacingMeters)
    {
        if(!std::isfinite(annotationSpacingMeters) || annotationSpacingMeters <= 0.0) {
            throw std::runtime_error("The annotation spacing must be finite and greater than zero.");
        }
        const auto clipped = clipYzThicknessSegments(segments, polygon);
        if(clipped.empty()) throw std::runtime_error("The selected region contains no YZ surface section.");
        const auto bounds = validateSegments(clipped);
        PositionNodes nodes(bounds.tolerance, bounds.minimum);
        struct Edge { SectionSegment segment; std::size_t a; std::size_t b; bool used = false; };
        std::vector<Edge> edges;
        std::map<std::pair<std::size_t, std::size_t>, std::size_t> uniqueEdges;
        for(const auto& segment : clipped) {
            const auto a = nodes.add(segment.first.positionMeters);
            const auto b = nodes.add(segment.second.positionMeters);
            if(a == b) continue;
            const auto ordered = std::minmax(a, b);
            const std::pair<std::size_t, std::size_t> key{ ordered.first, ordered.second };
            const auto found = uniqueEdges.find(key);
            if(found != uniqueEdges.end()) {
                const auto& previous = edges[found->second];
                const auto& first = previous.a == a ? previous.segment.first : previous.segment.second;
                const auto& second = previous.a == a ? previous.segment.second : previous.segment.first;
                if(!sameThickness(first.thicknessMeters, segment.first.thicknessMeters)
                    || !sameThickness(second.thicknessMeters, segment.second.thicknessMeters)) {
                    throw std::runtime_error("Coincident YZ faces have conflicting thickness; narrow the selected surface.");
                }
                continue;
            }
            uniqueEdges[key] = edges.size();
            edges.push_back({ segment, a, b });
        }
        if(edges.empty()) throw std::runtime_error("The selected YZ section has zero length.");
        std::vector<std::vector<std::size_t>> adjacency(nodes.positions.size());
        for(std::size_t i = 0; i < edges.size(); ++i) {
            adjacency[edges[i].a].push_back(i);
            adjacency[edges[i].b].push_back(i);
        }
        for(const auto& adjacent : adjacency) {
            if(adjacent.size() > 2) {
                throw std::runtime_error("The selected YZ section branches; select only the desired surface boundary.");
            }
        }
        std::vector<std::size_t> starts(nodes.positions.size());
        for(std::size_t i = 0; i < starts.size(); ++i) starts[i] = i;
        std::sort(starts.begin(), starts.end(), [&](std::size_t a, std::size_t b) {
            if(adjacency[a].size() != adjacency[b].size()) return adjacency[a].size() < adjacency[b].size();
            const auto& p = nodes.positions[a]; const auto& q = nodes.positions[b];
            return p.z() != q.z() ? p.z() < q.z() : p.y() < q.y();
        });
        ThicknessPickReportSection result;
        result.annotationSpacingMeters = annotationSpacingMeters;
        std::size_t detailCount = 0;
        for(const auto start : starts) {
            if(adjacency[start].empty()
                || std::all_of(adjacency[start].begin(), adjacency[start].end(),
                    [&](std::size_t edge) { return edges[edge].used; })) continue;
            std::vector<SectionSegment> path;
            std::size_t current = start;
            double arc = 0.0;
            while(true) {
                auto candidate = std::find_if(adjacency[current].begin(), adjacency[current].end(),
                    [&](std::size_t edge) { return !edges[edge].used; });
                if(candidate == adjacency[current].end()) break;
                auto& edge = edges[*candidate]; edge.used = true;
                SectionSegment segment = edge.segment;
                if(edge.b == current) std::swap(segment.first, segment.second);
                const std::size_t next = edge.a == current ? edge.b : edge.a;
                segment.first.positionMeters = nodes.positions[current];
                segment.second.positionMeters = nodes.positions[next];
                const double length = (segment.second.positionMeters - segment.first.positionMeters).stableNorm();
                segment.first.arcLengthMeters = arc;
                arc += length;
                segment.second.arcLengthMeters = arc;
                segment.first.profileIndex = segment.second.profileIndex = result.profiles.size();
                path.push_back(std::move(segment));
                current = next;
                if(current == start) break;
            }
            if(path.empty()) continue;
            if(!std::isfinite(arc) || arc / annotationSpacingMeters >= static_cast<double>(kMaximumSampleCount)) {
                throw std::runtime_error("The section annotation count exceeds the limit of 200000 points.");
            }
            std::vector<ReportSample> detail;
            for(const auto& segment : path) {
                const double length = segment.second.arcLengthMeters - segment.first.arcLengthMeters;
                const double desired = std::ceil(length / kMaximumSpacingMeters);
                if(!std::isfinite(desired) || desired >= static_cast<double>(kMaximumSampleCount)) {
                    throw std::runtime_error("The YZ thickness report exceeds the limit of 200000 points.");
                }
                const auto intervals = std::max<std::size_t>(1, static_cast<std::size_t>(desired));
                for(std::size_t i = 0; i <= intervals; ++i) {
                    if(i == 0 && !detail.empty()
                        && sameThickness(detail.back().thicknessMeters, segment.first.thicknessMeters)
                        && (detail.back().normalWorld - segment.first.normalWorld).stableNorm() <= 1.0e-9) continue;
                    if(++detailCount > kMaximumSampleCount) {
                        throw std::runtime_error("The YZ thickness report exceeds the limit of 200000 points.");
                    }
                    detail.push_back(interpolateSample(segment.first, segment.second,
                        static_cast<double>(i) / static_cast<double>(intervals)));
                }
            }
            const auto appendAnnotation = [&](double target, std::size_t& edgeIndex) {
                while(edgeIndex + 1 < path.size() && target >= path[edgeIndex].second.arcLengthMeters) ++edgeIndex;
                const auto& edge = path[edgeIndex];
                const double length = edge.second.arcLengthMeters - edge.first.arcLengthMeters;
                auto sample = interpolateSample(edge.first, edge.second,
                    std::clamp((target - edge.first.arcLengthMeters) / length, 0.0, 1.0));
                sample.arcLengthMeters = target;
                if(result.annotations.size() >= kMaximumSampleCount) {
                    throw std::runtime_error("The section annotation count exceeds the limit of 200000 points.");
                }
                result.annotations.push_back(std::move(sample));
            };
            std::size_t edgeIndex = 0;
            const auto intervalCount = static_cast<std::size_t>(std::floor(arc / annotationSpacingMeters));
            for(std::size_t i = 0; i <= intervalCount; ++i) {
                appendAnnotation(static_cast<double>(i) * annotationSpacingMeters, edgeIndex);
            }
            // Do not shift the equal-arc-length lattice to fit the last endpoint.
            if(arc - static_cast<double>(intervalCount) * annotationSpacingMeters > bounds.tolerance) {
                appendAnnotation(arc, edgeIndex);
            }
            result.profiles.push_back(std::move(detail));
        }
        return result;
    }
}
