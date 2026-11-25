#include "ThicknessSectionSchematic.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr std::size_t kMaximumPoints = 200000;
        constexpr double kNormalHalfWindow = 0.0001;
        constexpr double kDrawingSpacing = 0.00002;
        using Point = Eigen::Vector2d;
        using DrawingPoint = ThicknessSectionSchematicPoint;

        double cross(const Point& a, const Point& b)
        {
            return a.x() * b.y() - a.y() * b.x();
        }

        struct CurveNode
        {
            Point position = Point::Zero();
            double arc = 0.0;
            double before = 0.0;
            double after = 0.0;
        };

        struct Segment
        {
            Point a = Point::Zero();
            Point b = Point::Zero();
        };

        struct Box
        {
            Point minimum = Point::Constant(std::numeric_limits<double>::infinity());
            Point maximum = Point::Constant(-std::numeric_limits<double>::infinity());

            void include(const Point& p)
            {
                minimum = minimum.cwiseMin(p);
                maximum = maximum.cwiseMax(p);
            }

            bool overlaps(const Box& other, double tolerance) const
            {
                return (maximum.array() + tolerance >= other.minimum.array()).all()
                    && (other.maximum.array() + tolerance >= minimum.array()).all();
            }
        };

        Box boxOf(const Segment& segment)
        {
            Box box; box.include(segment.a); box.include(segment.b); return box;
        }

        // A balanced segment AABB tree keeps all intersection checks bounded by
        // geometry locality instead of an unbounded quadratic all-pairs scan.
        class SegmentTree
        {
        public:
            explicit SegmentTree(const std::vector<Segment>& segments) : m_segments(segments)
            {
                m_order.resize(segments.size());
                std::iota(m_order.begin(), m_order.end(), std::size_t{ 0 });
                m_nodes.reserve(2 * segments.size());
                if(!segments.empty()) build(0, segments.size());
            }

            template<class Predicate>
            bool any(const Segment& segment, double tolerance, Predicate predicate) const
            {
                if(m_nodes.empty()) return false;
                return visit(0, boxOf(segment), tolerance, predicate);
            }

        private:
            struct Node
            {
                Box box;
                std::size_t begin = 0, end = 0;
                std::size_t left = 0, right = 0;
            };

            std::size_t build(std::size_t begin, std::size_t end)
            {
                const auto id = m_nodes.size();
                m_nodes.emplace_back();
                Box box;
                for(std::size_t i = begin; i < end; ++i) {
                    box.include(m_segments[m_order[i]].a);
                    box.include(m_segments[m_order[i]].b);
                }
                m_nodes[id].box = box;
                m_nodes[id].begin = begin; m_nodes[id].end = end;
                if(end - begin <= 8) return id;
                const int axis = box.maximum.x() - box.minimum.x()
                    > box.maximum.y() - box.minimum.y() ? 0 : 1;
                const auto middle = begin + (end - begin) / 2;
                std::nth_element(m_order.begin() + begin, m_order.begin() + middle,
                    m_order.begin() + end, [&](std::size_t a, std::size_t b) {
                        return m_segments[a].a[axis] + m_segments[a].b[axis]
                            < m_segments[b].a[axis] + m_segments[b].b[axis];
                    });
                const auto left = build(begin, middle);
                const auto right = build(middle, end);
                m_nodes[id].left = left; m_nodes[id].right = right;
                return id;
            }

            template<class Predicate>
            bool visit(std::size_t id, const Box& box, double tolerance, Predicate& predicate) const
            {
                const auto& node = m_nodes[id];
                if(!node.box.overlaps(box, tolerance)) return false;
                if(node.left == 0) {
                    for(std::size_t i = node.begin; i < node.end; ++i) {
                        if(predicate(m_order[i], m_segments[m_order[i]])) return true;
                    }
                    return false;
                }
                return visit(node.left, box, tolerance, predicate)
                    || visit(node.right, box, tolerance, predicate);
            }

            const std::vector<Segment>& m_segments;
            std::vector<std::size_t> m_order;
            std::vector<Node> m_nodes;
        };

        // Includes interior touching/overlap for detecting a folded boundary.
        // The optional endpoint permissions are used only for true zero anchors
        // where the coating is intentionally coincident with the base.
        bool intersects(const Segment& a, const Segment& b, double tolerance,
            bool permitFirstEndpoint = false, bool permitLastEndpoint = false,
            bool permitCoincident = false)
        {
            if(!boxOf(a).overlaps(boxOf(b), tolerance)) return false;
            const Point u = a.b - a.a, v = b.b - b.a;
            const double lengthU = u.stableNorm(), lengthV = v.stableNorm();
            if(lengthU <= tolerance || lengthV <= tolerance) return false;
            const Point delta = b.a - a.a;
            const double determinant = cross(u, v);
            const double crossTolerance = tolerance * (lengthU + lengthV);
            if(std::abs(determinant) <= crossTolerance) {
                if(std::abs(cross(delta, u)) > tolerance * lengthU) return false;
                const double first = delta.dot(u) / u.squaredNorm();
                const double second = (b.b - a.a).dot(u) / u.squaredNorm();
                const double lo = std::max(0.0, std::min(first, second));
                const double hi = std::min(1.0, std::max(first, second));
                const double eps = tolerance / lengthU;
                if(hi < lo - eps) return false;
                if(permitCoincident) return false;
                if(hi - lo <= eps && ((permitFirstEndpoint && hi <= eps)
                    || (permitLastEndpoint && lo >= 1.0 - eps))) return false;
                return true;
            }
            const double t = cross(delta, v) / determinant;
            const double q = cross(delta, u) / determinant;
            const double epsU = tolerance / lengthU, epsV = tolerance / lengthV;
            if(t < -epsU || t > 1.0 + epsU || q < -epsV || q > 1.0 + epsV) return false;
            if((permitFirstEndpoint && t <= epsU) || (permitLastEndpoint && t >= 1.0 - epsU)) return false;
            return true;
        }

        bool selfIntersects(const std::vector<Segment>& segments, double tolerance, bool closed)
        {
            const SegmentTree tree(segments);
            for(std::size_t i = 0; i < segments.size(); ++i) {
                if(tree.any(segments[i], tolerance, [&](std::size_t j, const Segment& other) {
                    if(j <= i + 1 || (closed && i == 0 && j + 1 == segments.size())) return false;
                    return intersects(segments[i], other, tolerance);
                })) return true;
            }
            return false;
        }

        class SourceCurve
        {
        public:
            explicit SourceCurve(const std::vector<ThicknessPickReportSample>& profile)
            {
                if(profile.size() < 2 || profile.size() > kMaximumPoints) {
                    throw std::runtime_error("The schematic requires 2 to 200000 ordered profile points.");
                }
                Box bounds;
                for(const auto& sample : profile) {
                    if(!sample.positionMeters.allFinite() || !sample.normalWorld.allFinite()
                        || !std::isfinite(sample.arcLengthMeters) || sample.arcLengthMeters < 0.0
                        || !std::isfinite(sample.thicknessMeters) || sample.thicknessMeters < 0.0) {
                        throw std::runtime_error("The schematic profile contains invalid coordinates or thickness.");
                    }
                    bounds.include(sample.positionMeters.tail<2>());
                    maximumThickness = std::max(maximumThickness, sample.thicknessMeters);
                }
                span = (bounds.maximum - bounds.minimum).maxCoeff();
                tolerance = std::max(span * 1.0e-11, 1.0e-15);
                if(!std::isfinite(span) || span <= tolerance) {
                    throw std::runtime_error("The schematic base contour has zero length.");
                }
                double orientationSum = 0.0;
                for(std::size_t i = 0; i < profile.size(); ++i) {
                    const auto& sample = profile[i];
                    const Point position = sample.positionMeters.tail<2>();
                    if(!nodes.empty()) {
                        const double step = sample.arcLengthMeters - nodes.back().arc;
                        const double distance = (position - nodes.back().position).stableNorm();
                        if(step < -tolerance || std::abs(step - distance) > std::max(tolerance * 10.0, distance * 1.0e-6)) {
                            throw std::runtime_error("The schematic base contour has inconsistent arc length.");
                        }
                        if(step <= tolerance && distance <= tolerance) {
                            nodes.back().after = sample.thicknessMeters;
                            continue;
                        }
                        Point normal = sample.normalWorld.tail<2>();
                        const double normalLength = normal.stableNorm();
                        if(normalLength > 0.0) {
                            const Point edge = position - nodes.back().position;
                            const Point right(edge.y(), -edge.x());
                            orientationSum += right.dot(normal / normalLength);
                        }
                    }
                    nodes.push_back({ position, sample.arcLengthMeters,
                        sample.thicknessMeters, sample.thicknessMeters });
                }
                if(nodes.size() < 2) throw std::runtime_error("The schematic base contour has zero length.");
                first = nodes.front().arc; last = nodes.back().arc; length = last - first;
                if(!std::isfinite(length) || length <= tolerance) {
                    throw std::runtime_error("The schematic base contour has invalid arc length.");
                }
                // Only the global outward side comes from mesh normals. Their
                // per-triangle fluctuations never drive the drawing tangent.
                orientation = orientationSum < 0.0 ? -1.0 : 1.0;
                halfWindow = std::min(kNormalHalfWindow, length * 0.1);
                closed = (nodes.front().position - nodes.back().position).stableNorm() <= tolerance;
                integrals.reserve(nodes.size()); integrals.push_back(0.0);
                for(std::size_t i = 1; i < nodes.size(); ++i) {
                    baseSegments.push_back({ nodes[i - 1].position, nodes[i].position });
                    integrals.push_back(integrals.back() + (nodes[i].arc - nodes[i - 1].arc)
                        * (nodes[i - 1].after + nodes[i].before) * 0.5);
                }
                if(selfIntersects(baseSegments, tolerance, closed)) {
                    throw std::runtime_error("The schematic base contour crosses itself; select one unambiguous surface.");
                }
            }

            std::size_t interval(double arc) const
            {
                const auto next = std::upper_bound(nodes.begin(), nodes.end(), arc,
                    [](double value, const CurveNode& node) { return value < node.arc; });
                if(next == nodes.begin()) return 0;
                return std::min(static_cast<std::size_t>(next - nodes.begin() - 1), nodes.size() - 2);
            }

            Point position(double arc) const
            {
                if(arc <= first) return nodes.front().position;
                if(arc >= last) return nodes.back().position;
                const auto i = interval(arc);
                const double t = (arc - nodes[i].arc) / (nodes[i + 1].arc - nodes[i].arc);
                return (1.0 - t) * nodes[i].position + t * nodes[i + 1].position;
            }

            Point normal(double arc) const
            {
                double lo = arc - halfWindow, hi = arc + halfWindow;
                Point a, b;
                if(closed) {
                    while(lo < first) lo += length;
                    while(hi > last) hi -= length;
                    a = position(lo); b = position(hi);
                } else {
                    a = position(std::max(first, lo)); b = position(std::min(last, hi));
                }
                Point tangent = b - a;
                const double norm = tangent.stableNorm();
                if(norm <= tolerance) throw std::runtime_error("The schematic base has an unresolved cusp.");
                tangent /= norm;
                return orientation * Point(tangent.y(), -tangent.x());
            }

            double rawThickness(double arc) const
            {
                if(arc <= first) return nodes.front().after;
                if(arc >= last) return nodes.back().after;
                const auto i = interval(arc);
                const double t = (arc - nodes[i].arc) / (nodes[i + 1].arc - nodes[i].arc);
                return (1.0 - t) * nodes[i].after + t * nodes[i + 1].before;
            }

            double integral(double arc) const
            {
                if(arc <= first) return 0.0;
                if(arc >= last) return integrals.back();
                const auto i = interval(arc);
                const double delta = arc - nodes[i].arc;
                return integrals[i] + delta * (nodes[i].after + rawThickness(arc)) * 0.5;
            }

            double drawingThickness(double arc) const
            {
                const double raw = rawThickness(arc);
                if(raw == 0.0) return 0.0;
                const double lo = std::max(first, arc - halfWindow);
                const double hi = std::min(last, arc + halfWindow);
                const double average = (integral(hi) - integral(lo)) / (hi - lo);
                // This average affects the illustrative boundary only. Smooth
                // cell-value steps over a physical 0.2 mm neighborhood; preserve
                // raw zero spans here and exact original annotations below.
                return std::max(0.0, average);
            }

            std::vector<CurveNode> nodes;
            std::vector<Segment> baseSegments;
            std::vector<double> integrals;
            double span = 0.0, tolerance = 0.0, first = 0.0, last = 0.0, length = 0.0;
            double maximumThickness = 0.0, halfWindow = 0.0, orientation = 1.0;
            bool closed = false;
        };
    }

    ThicknessSectionSchematicGeometry buildThicknessSectionSchematic(
        const std::vector<ThicknessPickReportSample>& profile,
        const std::vector<ThicknessPickReportSample>& annotations,
        const ThicknessSectionSchematicOptions& options)
    {
        if(!options.automaticThicknessMultiplier
            && (!std::isfinite(options.fixedThicknessMultiplier)
                || options.fixedThicknessMultiplier < 1.0 || options.fixedThicknessMultiplier > 1000.0)) {
            throw std::runtime_error("The fixed thickness display multiplier must be between 1 and 1000.");
        }
        if(annotations.size() > kMaximumPoints) {
            throw std::runtime_error("The schematic annotation count exceeds 200000.");
        }
        const SourceCurve source(profile);
        struct Anchor
        {
            std::size_t index = 0;
            double arc = 0.0;
            double raw = 0.0;
            double correction = 0.0;
        };
        std::vector<Anchor> anchors;
        anchors.reserve(annotations.size());
        for(std::size_t i = 0; i < annotations.size(); ++i) {
            const auto& annotation = annotations[i];
            if(!annotation.positionMeters.allFinite() || !std::isfinite(annotation.arcLengthMeters)
                || !std::isfinite(annotation.thicknessMeters) || annotation.thicknessMeters < 0.0
                || annotation.arcLengthMeters < source.first - source.tolerance
                || annotation.arcLengthMeters > source.last + source.tolerance
                || (source.position(annotation.arcLengthMeters)
                    - annotation.positionMeters.tail<2>()).stableNorm() > source.tolerance * 20.0) {
                throw std::runtime_error("A schematic annotation does not lie on its original base contour.");
            }
            anchors.push_back({ i, annotation.arcLengthMeters, annotation.thicknessMeters,
                annotation.thicknessMeters - source.drawingThickness(annotation.arcLengthMeters) });
        }
        std::stable_sort(anchors.begin(), anchors.end(), [](const Anchor& a, const Anchor& b) { return a.arc < b.arc; });
        const auto correctedThickness = [&](double arc) {
            const double raw = source.rawThickness(arc);
            if(raw == 0.0) return 0.0;
            if(anchors.empty()) return source.drawingThickness(arc);
            const auto next = std::upper_bound(anchors.begin(), anchors.end(), arc,
                [](double value, const Anchor& anchor) { return value < anchor.arc; });
            double correction = 0.0;
            if(next == anchors.begin()) correction = next->correction;
            else if(next == anchors.end()) correction = anchors.back().correction;
            else {
                const auto& previous = *(next - 1);
                const double t = (arc - previous.arc) / (next->arc - previous.arc);
                correction = previous.correction * (1.0 - t) + next->correction * t;
            }
            return std::max(0.0, source.drawingThickness(arc) + correction);
        };

        // Uniform physical spacing controls smoothing independently of mesh
        // subdivision. Original geometric knots and exact annotations are added
        // so the unmodified base corners and measurement endpoints remain exact.
        const double spacing = std::max(kDrawingSpacing, source.length / 50000.0);
        const auto intervals = std::max<std::size_t>(1,
            static_cast<std::size_t>(std::ceil(source.length / spacing)));
        std::vector<double> parameters;
        parameters.reserve(intervals + source.nodes.size() + anchors.size() + 1);
        for(std::size_t i = 0; i <= intervals; ++i) {
            parameters.push_back(source.first + source.length * static_cast<double>(i) / static_cast<double>(intervals));
        }
        for(const auto& node : source.nodes) parameters.push_back(node.arc);
        for(const auto& anchor : anchors) parameters.push_back(anchor.arc);
        std::sort(parameters.begin(), parameters.end());
        parameters.erase(std::unique(parameters.begin(), parameters.end(), [&](double a, double b) {
            return std::abs(a - b) <= source.tolerance;
        }), parameters.end());
        if(parameters.size() + anchors.size() > kMaximumPoints) {
            throw std::runtime_error("The schematic display geometry exceeds 200000 points.");
        }
        ThicknessSectionSchematicGeometry result;
        result.smoothingWindowMeters = 2.0 * source.halfWindow;
        std::vector<double> thicknesses;
        std::vector<std::size_t> annotationVertices(annotations.size());
        std::size_t anchorIndex = 0;
        for(const double arc : parameters) {
            while(anchorIndex < anchors.size() && anchors[anchorIndex].arc < arc - source.tolerance) ++anchorIndex;
            if(anchorIndex < anchors.size() && std::abs(anchors[anchorIndex].arc - arc) <= source.tolerance) {
                do {
                    const auto& anchor = anchors[anchorIndex];
                    const auto& raw = annotations[anchor.index];
                    annotationVertices[anchor.index] = result.points.size();
                    DrawingPoint point;
                    point.baseYZ = raw.positionMeters.tail<2>();
                    point.normalYZ = source.normal(raw.arcLengthMeters);
                    point.arcLengthMeters = raw.arcLengthMeters;
                    result.points.push_back(point);
                    thicknesses.push_back(raw.thicknessMeters);
                    ++anchorIndex;
                } while(anchorIndex < anchors.size() && std::abs(anchors[anchorIndex].arc - arc) <= source.tolerance);
            } else {
                DrawingPoint point;
                point.baseYZ = source.position(arc);
                point.normalYZ = source.normal(arc);
                point.arcLengthMeters = arc;
                result.points.push_back(point);
                thicknesses.push_back(correctedThickness(arc));
            }
        }
        const SegmentTree baseTree(source.baseSegments);
        const auto makeCoating = [&](double multiplier) {
            for(std::size_t i = 0; i < result.points.size(); ++i) {
                auto& point = result.points[i];
                point.coatingYZ = point.baseYZ + point.normalYZ * (thicknesses[i] * multiplier);
            }
        };
        const auto safeCoating = [&](double multiplierForCheck) {
            std::vector<Segment> coating;
            std::vector<std::size_t> originalIndices;
            coating.reserve(result.points.size()); originalIndices.reserve(result.points.size());
            for(std::size_t i = 1; i < result.points.size(); ++i) {
                const auto& a = result.points[i - 1]; const auto& b = result.points[i];
                const Point baseDelta = b.baseYZ - a.baseYZ;
                const Point coatDelta = b.coatingYZ - a.coatingYZ;
                if(!b.coatingYZ.allFinite()) return false;
                // Detect curvature-induced folds without confusing a legitimate
                // thickness jump with reversed geometry. The full variable-
                // thickness boundary still passes the intersection checks below.
                const Point curvatureOffset = baseDelta + (b.normalYZ - a.normalYZ)
                    * (0.5 * (thicknesses[i - 1] + thicknesses[i]) * multiplierForCheck);
                if(baseDelta.squaredNorm() > source.tolerance * source.tolerance
                    && baseDelta.dot(curvatureOffset) <= baseDelta.squaredNorm() * 0.05) return false;
                if(coatDelta.stableNorm() <= source.tolerance) continue;
                coating.push_back({ a.coatingYZ, b.coatingYZ });
                originalIndices.push_back(i);
            }
            if(selfIntersects(coating, source.tolerance, source.closed)) return false;
            for(std::size_t i = 0; i < coating.size(); ++i) {
                const auto end = originalIndices[i];
                const bool firstZero = thicknesses[end - 1] == 0.0;
                const bool lastZero = thicknesses[end] == 0.0;
                if(firstZero && lastZero) continue;
                if(baseTree.any(coating[i], source.tolerance, [&](std::size_t, const Segment& base) {
                    return intersects(coating[i], base, source.tolerance, firstZero, lastZero,
                        firstZero && lastZero);
                })) return false;
            }
            return true;
        };
        double multiplier = options.automaticThicknessMultiplier
            ? (source.maximumThickness > 0.0
                ? std::clamp(source.span * 0.07 / source.maximumThickness, 1.0, 1000.0) : 1.0)
            : options.fixedThicknessMultiplier;
        bool accepted = false;
        const int maximumAttempts = options.automaticThicknessMultiplier ? 80 : 1;
        for(int attempt = 0; attempt < maximumAttempts; ++attempt) {
            makeCoating(multiplier);
            if(safeCoating(multiplier)) { accepted = true; break; }
            // A single whole-profile multiplier protects all views and preserves
            // every raw annotation value, including narrow concave recesses.
            if(options.automaticThicknessMultiplier) multiplier *= 0.8;
        }
        if(!accepted) {
            if(!options.automaticThicknessMultiplier) throw ThicknessDisplayMultiplierError();
            throw std::runtime_error("A non-crossing coating schematic cannot be generated for this base contour.");
        }
        result.thicknessMultiplier = multiplier;
        result.annotations.reserve(annotations.size());
        for(const auto index : annotationVertices) result.annotations.push_back(result.points[index]);
        return result;
    }
}
