#include <ThicknessSectionSchematic.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    using namespace robot_qt_viewer;

    constexpr double kPi = 3.14159265358979323846;

    void requireSchematic(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    void nearSchematic(double actual, double expected, double tolerance, const char* message)
    {
        requireSchematic(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
    }

    ThicknessPickReportSample point(double y, double z, double thickness,
        double arc, double normalY, double normalZ, std::size_t triangle = 0)
    {
        ThicknessPickReportSample result;
        result.positionMeters = Eigen::Vector3d(0.0, y, z);
        result.thicknessMeters = thickness;
        result.normalWorld = Eigen::Vector3d(0.0, normalY, normalZ).normalized();
        result.arcLengthMeters = arc;
        result.triangleIndex = triangle;
        return result;
    }

    void requireUnchanged(const std::vector<ThicknessPickReportSample>& actual,
        const std::vector<ThicknessPickReportSample>& expected)
    {
        requireSchematic(actual.size() == expected.size(), "Schematic generation changed the source sample count.");
        for(std::size_t i = 0; i < actual.size(); ++i) {
            const auto& a = actual[i];
            const auto& b = expected[i];
            requireSchematic(a.positionMeters == b.positionMeters && a.normalWorld == b.normalWorld
                && a.thicknessMeters == b.thicknessMeters && a.arcLengthMeters == b.arcLengthMeters
                && a.triangleIndex == b.triangleIndex && a.profileIndex == b.profileIndex,
                "Diagram smoothing mutated a source coordinate, thickness, normal or sample identity.");
        }
    }

    double cross(const Eigen::Vector2d& first, const Eigen::Vector2d& second)
    {
        return first.x() * second.y() - first.y() * second.x();
    }

    bool crosses(const Eigen::Vector2d& a, const Eigen::Vector2d& b,
        const Eigen::Vector2d& c, const Eigen::Vector2d& d)
    {
        if(a.cwiseMax(b).x() < c.cwiseMin(d).x() || c.cwiseMax(d).x() < a.cwiseMin(b).x()
            || a.cwiseMax(b).y() < c.cwiseMin(d).y() || c.cwiseMax(d).y() < a.cwiseMin(b).y()) return false;
        const double first = cross(b - a, c - a);
        const double second = cross(b - a, d - a);
        const double third = cross(d - c, a - c);
        const double fourth = cross(d - c, b - c);
        constexpr double tolerance = 1.0e-18;
        return ((first > tolerance && second < -tolerance) || (first < -tolerance && second > tolerance))
            && ((third > tolerance && fourth < -tolerance) || (third < -tolerance && fourth > tolerance));
    }

    void requireNoCoatingCrossings(const ThicknessSectionSchematicGeometry& geometry)
    {
        for(std::size_t i = 1; i < geometry.points.size(); ++i) {
            for(std::size_t j = i + 2; j < geometry.points.size(); ++j) {
                requireSchematic(!crosses(geometry.points[i - 1].coatingYZ, geometry.points[i].coatingYZ,
                    geometry.points[j - 1].coatingYZ, geometry.points[j].coatingYZ),
                    "The displayed coating curve crosses itself around a concave feature.");
            }
        }
    }

    void requireExactAnnotations(const ThicknessSectionSchematicGeometry& geometry,
        const std::vector<ThicknessPickReportSample>& annotations)
    {
        requireSchematic(std::isfinite(geometry.thicknessMultiplier) && geometry.thicknessMultiplier > 0.0,
            "The diagram needs one finite positive thickness multiplier.");
        requireSchematic(geometry.annotations.size() == annotations.size(),
            "Smoothing lost an annotation or merged annotations at a thickness discontinuity.");
        for(std::size_t i = 0; i < annotations.size(); ++i) {
            const auto& source = annotations[i];
            const auto& displayed = geometry.annotations[i];
            nearSchematic(displayed.arcLengthMeters, source.arcLengthMeters, 1.0e-12,
                "Diagram annotations lost their original order or arc-length location.");
            nearSchematic((displayed.baseYZ - source.positionMeters.tail<2>()).norm(), 0.0, 1.0e-12,
                "Diagram smoothing shifted the original measurement anchor.");
            nearSchematic((displayed.coatingYZ - displayed.baseYZ).norm(),
                source.thicknessMeters * geometry.thicknessMultiplier, 1.0e-12,
                "A diagram annotation used an averaged thickness or a different display multiplier.");
            nearSchematic(displayed.normalYZ.norm(), 1.0, 1.0e-10,
                "A displayed measurement direction is not a finite unit normal.");
            requireSchematic(std::any_of(geometry.points.begin(), geometry.points.end(), [&](const auto& vertex) {
                return std::abs(vertex.arcLengthMeters - displayed.arcLengthMeters) <= 1.0e-12
                    && (vertex.baseYZ - displayed.baseYZ).norm() <= 1.0e-12
                    && (vertex.coatingYZ - displayed.coatingYZ).norm() <= 1.0e-12;
            }), "A thickness annotation does not terminate on the displayed coating curve.");
            if(source.thicknessMeters == 0.0) {
                nearSchematic((displayed.coatingYZ - displayed.baseYZ).norm(), 0.0, 0.0,
                    "A true zero-thickness annotation was raised by display smoothing.");
            }
        }
    }

    void testConcaveRadiusAndRawValues()
    {
        // A long U with a 0.4 mm concave radius makes the former span-based
        // exaggeration larger than the radius and reverses the offset arc.
        constexpr double radius = 0.0004;
        constexpr double arm = 0.050;
        constexpr double thickness = 150.0e-6;
        std::vector<ThicknessPickReportSample> profile;
        double arc = 0.0;
        const auto append = [&](double y, double z, double ny, double nz) {
            if(!profile.empty()) arc += (Eigen::Vector2d(y, z) - profile.back().positionMeters.tail<2>()).norm();
            profile.push_back(point(y, z, thickness, arc, ny, nz, profile.size()));
        };
        append(radius, -arm, -1.0, 0.0);
        append(radius, 0.0, -1.0, 0.0);
        for(int i = 1; i <= 48; ++i) {
            const double angle = kPi * static_cast<double>(i) / 48.0;
            append(radius * std::cos(angle), radius * std::sin(angle), -std::cos(angle), -std::sin(angle));
        }
        append(-radius, -arm, 1.0, 0.0);
        std::vector<ThicknessPickReportSample> annotations;
        for(std::size_t i = 2; i + 1 < profile.size(); i += 4) annotations.push_back(profile[i]);
        const auto profileBefore = profile;
        const auto annotationsBefore = annotations;
        const double oldMultiplier = (arm + radius) * 0.07 / thickness;
        bool oldFold = false;
        for(std::size_t i = 3; i + 1 < profile.size(); ++i) {
            const Eigen::Vector2d first = profile[i - 1].positionMeters.tail<2>();
            const Eigen::Vector2d second = profile[i].positionMeters.tail<2>();
            const Eigen::Vector2d firstOffset = first + profile[i - 1].normalWorld.tail<2>() * thickness * oldMultiplier;
            const Eigen::Vector2d secondOffset = second + profile[i].normalWorld.tail<2>() * thickness * oldMultiplier;
            oldFold |= (second - first).dot(secondOffset - firstOffset) < 0.0;
        }
        requireSchematic(oldFold, "The concave fixture does not reproduce the former offset fold.");
        ThicknessSectionSchematicOptions unsafeFixed;
        unsafeFixed.automaticThicknessMultiplier = false;
        unsafeFixed.fixedThicknessMultiplier = 25.0;
        bool unsafeRejected = false;
        try { buildThicknessSectionSchematic(profile, annotations, unsafeFixed); }
        catch(const ThicknessDisplayMultiplierError&) { unsafeRejected = true; }
        requireSchematic(unsafeRejected,
            "Unsafe fixed 25 x must be rejected without silently reducing the multiplier.");
        const auto geometry = buildThicknessSectionSchematic(profile, annotations);
        requireSchematic(geometry.thicknessMultiplier < oldMultiplier,
            "The concave feature did not constrain excessive thickness exaggeration.");
        requireExactAnnotations(geometry, annotations);
        requireNoCoatingCrossings(geometry);
        for(std::size_t i = 1; i < geometry.annotations.size(); ++i) {
            const auto& first = geometry.annotations[i - 1];
            const auto& second = geometry.annotations[i];
            requireSchematic((second.baseYZ - first.baseYZ).dot(second.coatingYZ - first.coatingYZ) >= -1.0e-14,
                "The smoothed coating still folds backward inside a concave radius.");
        }
        requireUnchanged(profile, profileBefore);
        requireUnchanged(annotations, annotationsBefore);
    }

    void testTriangleNormalJitterAndZeros()
    {
        std::vector<ThicknessPickReportSample> profile;
        std::vector<ThicknessPickReportSample> annotations;
        for(int i = 0; i <= 100; ++i) {
            const double s = i * 0.001;
            const double angle = i % 2 == 0 ? 0.08 : -0.08;
            profile.push_back(point(0.0, s, 120.0e-6, s, std::cos(angle), std::sin(angle), i));
            if(i % 10 == 0) annotations.push_back(profile.back());
        }
        const auto original = profile;
        const auto geometry = buildThicknessSectionSchematic(profile, annotations);
        requireExactAnnotations(geometry, annotations);
        for(const auto& displayed : geometry.points) {
            nearSchematic(displayed.coatingYZ.x(), 120.0e-6 * geometry.thicknessMultiplier, 1.0e-10,
                "Alternating triangle normals made a straight coating edge zigzag.");
            nearSchematic(displayed.coatingYZ.y(), displayed.baseYZ.y(), 1.0e-10,
                "Triangle-normal jitter tilted a straight measurement away from the section normal.");
        }
        requireUnchanged(profile, original);

        for(auto& sample : profile) sample.thicknessMeters = 0.0;
        annotations = { profile.front(), profile[50], profile.back() };
        const auto zero = buildThicknessSectionSchematic(profile, annotations);
        requireExactAnnotations(zero, annotations);
        for(const auto& displayed : zero.points) {
            nearSchematic((displayed.coatingYZ - displayed.baseYZ).norm(), 0.0, 0.0,
                "The display created a coating band on an entirely zero-thickness profile.");
        }
    }

    Eigen::Vector2d coatingAt(const ThicknessSectionSchematicGeometry& geometry, double arc)
    {
        auto next = std::upper_bound(geometry.points.begin(), geometry.points.end(), arc,
            [](double value, const auto& vertex) { return value < vertex.arcLengthMeters; });
        if(next == geometry.points.begin()) return next->coatingYZ;
        if(next == geometry.points.end()) return geometry.points.back().coatingYZ;
        const auto& previous = *(next - 1);
        const double length = next->arcLengthMeters - previous.arcLengthMeters;
        if(length <= 0.0) return next->coatingYZ;
        const double ratio = (arc - previous.arcLengthMeters) / length;
        return previous.coatingYZ * (1.0 - ratio) + next->coatingYZ * ratio;
    }

    void testNonuniformSamplingAndIndependentProfiles()
    {
        std::vector<ThicknessPickReportSample> sparse;
        double arc = 0.0;
        for(int i = 0; i <= 40; ++i) {
            const double angle = (kPi * 0.5) * static_cast<double>(i) / 40.0;
            const Eigen::Vector2d position(0.020 * std::cos(angle), 0.020 * std::sin(angle));
            if(!sparse.empty()) arc += (position - sparse.back().positionMeters.tail<2>()).norm();
            sparse.push_back(point(position.x(), position.y(), 80.0e-6 + arc * 0.002,
                arc, std::cos(angle), std::sin(angle), i));
        }
        std::vector<ThicknessPickReportSample> dense;
        dense.push_back(sparse.front());
        for(std::size_t edge = 1; edge < sparse.size(); ++edge) {
            const int divisions = edge % 3 == 0 ? 29 : edge % 3 == 1 ? 1 : 7;
            for(int index = 1; index <= divisions; ++index) {
                const double ratio = static_cast<double>(index) / divisions;
                auto sample = sparse[edge];
                sample.positionMeters = (1.0 - ratio) * sparse[edge - 1].positionMeters + ratio * sparse[edge].positionMeters;
                sample.normalWorld = ((1.0 - ratio) * sparse[edge - 1].normalWorld + ratio * sparse[edge].normalWorld).normalized();
                sample.arcLengthMeters = (1.0 - ratio) * sparse[edge - 1].arcLengthMeters + ratio * sparse[edge].arcLengthMeters;
                sample.thicknessMeters = (1.0 - ratio) * sparse[edge - 1].thicknessMeters + ratio * sparse[edge].thicknessMeters;
                dense.push_back(sample);
            }
        }
        const std::vector<ThicknessPickReportSample> annotations{
            sparse.front(), sparse[10], sparse[20], sparse[30], sparse.back() };
        const auto sparseDiagram = buildThicknessSectionSchematic(sparse, annotations);
        const auto denseDiagram = buildThicknessSectionSchematic(dense, annotations);
        requireExactAnnotations(sparseDiagram, annotations);
        requireExactAnnotations(denseDiagram, annotations);
        nearSchematic(sparseDiagram.thicknessMultiplier, denseDiagram.thicknessMultiplier, 1.0e-9,
            "Identical geometry acquired a different display multiplier after mesh subdivision.");
        for(int i = 0; i <= 200; ++i) {
            const double s = arc * static_cast<double>(i) / 200.0;
            nearSchematic((coatingAt(sparseDiagram, s) - coatingAt(denseDiagram, s)).norm(), 0.0, 5.0e-6,
                "Nonuniform triangle sampling changed the smoothed coating instead of using arc length.");
        }

        auto independent = sparse;
        for(auto& sample : independent) {
            sample.positionMeters.y() += 1.0;
            sample.positionMeters.z() += 2.0;
            sample.thicknessMeters = 0.0;
            sample.profileIndex = 7;
        }
        const auto independentBefore = independent;
        const auto otherDiagram = buildThicknessSectionSchematic(independent,
            { independent.front(), independent.back() });
        for(const auto& displayed : otherDiagram.points) {
            requireSchematic(displayed.baseYZ.x() >= 1.0 - 1.0e-12 && displayed.baseYZ.y() >= 2.0 - 1.0e-12,
                "Display smoothing joined coordinates from independent section profiles.");
            nearSchematic((displayed.coatingYZ - displayed.baseYZ).norm(), 0.0, 0.0,
                "An independent zero-thickness profile inherited another profile's coating.");
        }
        const auto repeated = buildThicknessSectionSchematic(sparse, annotations);
        requireSchematic(repeated.points.size() == sparseDiagram.points.size(),
            "Processing an independent profile changed another profile's smoothing state.");
        for(std::size_t i = 0; i < repeated.points.size(); ++i) {
            nearSchematic((repeated.points[i].coatingYZ - sparseDiagram.points[i].coatingYZ).norm(), 0.0, 0.0,
                "Diagram geometry depends on another profile processed earlier.");
        }
        requireUnchanged(independent, independentBefore);
    }

    void testFixedMultiplierAndDefaultCompatibility()
    {
        const std::vector<ThicknessPickReportSample> profile{
            point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            point(0.0, 0.040, 100.0e-6, 0.040, 1.0, 0.0),
            point(0.0, 0.080, 150.0e-6, 0.080, 1.0, 0.0) };
        const auto annotations = profile;
        const auto original = profile;
        const auto automatic = buildThicknessSectionSchematic(profile, annotations);
        ThicknessSectionSchematicOptions options;
        options.fixedThicknessMultiplier = std::numeric_limits<double>::quiet_NaN();
        const auto explicitAutomatic = buildThicknessSectionSchematic(profile, annotations, options);
        nearSchematic(explicitAutomatic.thicknessMultiplier, automatic.thicknessMultiplier, 0.0,
            "Default automatic rendering changed when options were supplied explicitly.");
        requireSchematic(explicitAutomatic.points.size() == automatic.points.size(),
            "Explicit automatic mode changed the display sampling.");
        for(std::size_t i = 0; i < automatic.points.size(); ++i) {
            nearSchematic((automatic.points[i].coatingYZ - explicitAutomatic.points[i].coatingYZ).norm(),
                0.0, 0.0, "Automatic mode must ignore the inactive fixed multiplier.");
        }
        options.automaticThicknessMultiplier = false;
        for(const double multiplier : { 1.0, 25.0 }) {
            options.fixedThicknessMultiplier = multiplier;
            const auto fixed = buildThicknessSectionSchematic(profile, annotations, options);
            nearSchematic(fixed.thicknessMultiplier, multiplier, 0.0,
                "A safe fixed multiplier was silently changed.");
            requireExactAnnotations(fixed, annotations);
            requireNoCoatingCrossings(fixed);
            requireUnchanged(profile, original);
        }
        for(const double invalid : { 0.0, -1.0, 0.5, 1001.0,
            std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() }) {
            options.fixedThicknessMultiplier = invalid;
            bool rejected = false;
            try { buildThicknessSectionSchematic(profile, annotations, options); }
            catch(const std::exception&) { rejected = true; }
            requireSchematic(rejected, "Invalid fixed display multiplier was accepted.");
        }
    }

    void testSharpCornerAndThicknessDiscontinuity()
    {
        const std::vector<ThicknessPickReportSample> profile{
            point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0),
            point(0.0, 0.010, 0.0, 0.010, 1.0, 0.0, 0),
            point(0.0, 0.010, 160.0e-6, 0.010, 1.0, 0.0, 1),
            point(0.0, 0.020, 160.0e-6, 0.020, 1.0, 0.0, 1),
            point(0.010, 0.020, 40.0e-6, 0.030, 0.0, -1.0, 2) };
        const std::vector<ThicknessPickReportSample> annotations = profile;
        const auto original = profile;
        const auto geometry = buildThicknessSectionSchematic(profile, annotations);
        requireExactAnnotations(geometry, annotations);
        requireNoCoatingCrossings(geometry);
        requireSchematic((geometry.annotations[1].coatingYZ - geometry.annotations[1].baseYZ).norm() == 0.0
            && (geometry.annotations[2].coatingYZ - geometry.annotations[2].baseYZ).norm() > 0.0,
            "The two sides of a zero-to-positive thickness seam were averaged together.");
        for(const auto& displayed : geometry.points) {
            requireSchematic(displayed.baseYZ.allFinite() && displayed.coatingYZ.allFinite()
                && displayed.normalYZ.allFinite(), "A sharp corner created non-finite schematic geometry.");
            if(displayed.arcLengthMeters < 0.010 - 1.0e-12) {
                nearSchematic((displayed.coatingYZ - displayed.baseYZ).norm(), 0.0, 0.0,
                    "Smoothing spread positive thickness across the boundary of a true zero span.");
            }
        }
        requireUnchanged(profile, original);

        const std::vector<ThicknessPickReportSample> crossing{
            point(0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            point(0.010, 0.010, 0.0, std::sqrt(2.0) * 0.010, 1.0, 0.0),
            point(0.0, 0.010, 0.0, std::sqrt(2.0) * 0.010 + 0.010, 1.0, 0.0),
            point(0.010, 0.0, 0.0, std::sqrt(2.0) * 0.020 + 0.010, 1.0, 0.0) };
        bool rejected = false;
        try { buildThicknessSectionSchematic(crossing, { crossing.front(), crossing.back() }); }
        catch(const std::exception&) { rejected = true; }
        requireSchematic(rejected, "A self-crossing base contour must be rejected instead of drawing a misleading coating.");
    }
}

void runSectionSchematicTests()
{
    testConcaveRadiusAndRawValues();
    testTriangleNormalJitterAndZeros();
    testNonuniformSamplingAndIndependentProfiles();
    testSharpCornerAndThicknessDiscontinuity();
    testFixedMultiplierAndDefaultCompatibility();
}
