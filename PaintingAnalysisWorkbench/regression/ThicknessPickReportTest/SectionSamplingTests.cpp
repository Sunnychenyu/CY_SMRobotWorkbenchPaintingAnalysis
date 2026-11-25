#include <ThicknessPickSampling.h>

#include <AssetCore/ModelDesc.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    using namespace robot_qt_viewer;

    void requireSection(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    void nearSection(double actual, double expected, double tolerance, const char* message)
    {
        requireSection(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
    }

    template<class Action>
    void rejectsSection(Action action, const char* message)
    {
        bool rejected = false;
        try { action(); } catch(const std::exception&) { rejected = true; }
        requireSection(rejected, message);
    }

    ThicknessPickSectionSegment segment(double y0, double z0, double y1, double z1,
        double thickness0 = 0.0, double thickness1 = 0.0, std::size_t triangle = 0)
    {
        ThicknessPickSectionSegment result{
            { triangle, Eigen::Vector3d(0.0, y0, z0), thickness0 },
            { triangle, Eigen::Vector3d(0.0, y1, z1), thickness1 } };
        Eigen::Vector3d normal(0.0, z1 - z0, y0 - y1);
        normal.normalize();
        result.first.normalWorld = result.second.normalWorld = normal;
        return result;
    }

    std::vector<Eigen::Vector2d> rectangle(double y0, double z0, double y1, double z1)
    {
        return { { y0, z0 }, { y1, z0 }, { y1, z1 }, { y0, z1 } };
    }

    void testSelectedSurfaceAndArcLength()
    {
        // Three touching faces form a closed part outline. The lasso contains
        // only the outer front boundary and must retain its uncoated lower run.
        const std::vector<ThicknessPickSectionSegment> source{
            segment(0.01, 0.0, 0.01, 0.024, 0.0, 0.0),
            segment(0.01, 0.024, 0.019, 0.036, 0.0, 150.0e-6, 1),
            segment(0.019, 0.036, 0.019, 0.05, 150.0e-6, 150.0e-6, 2),
            segment(-0.01, 0.0, -0.01, 0.05, 70.0e-6, 70.0e-6, 3),
            segment(-0.01, 0.0, 0.01, 0.0, 80.0e-6, 80.0e-6, 4),
            segment(-0.01, 0.05, 0.019, 0.05, 90.0e-6, 90.0e-6, 5) };
        const auto polygon = rectangle(0.009, 0.001, 0.021, 0.049);
        const auto selected = buildSelectedYzThicknessSection(source, polygon, 0.01);
        requireSection(selected.profiles.size() == 1, "Surface selection bridged another face.");
        const auto& detail = selected.profiles.front();
        requireSection(detail.size() > 200, "Section detail lost the 0.2 mm sampling density.");
        bool zero = false;
        for(std::size_t i = 0; i < detail.size(); ++i) {
            const auto& point = detail[i];
            requireSection(point.triangleIndex <= 2 && point.positionMeters.y() >= 0.01 - 1.0e-12,
                "The report included a back, end, or base face outside the lasso.");
            zero |= point.thicknessMeters == 0.0;
            if(i > 0) {
                requireSection(point.arcLengthMeters >= detail[i - 1].arcLengthMeters,
                    "Detail points must follow ordered arc length.");
                requireSection((point.positionMeters - detail[i - 1].positionMeters).norm() <= 0.0002 + 1.0e-12,
                    "Ordered detail has a gap greater than 0.2 mm.");
            }
        }
        requireSection(zero, "The selected front surface lost valid zero thickness.");
        // Cropped length: .023 + hypot(.009,.012) + .013 = .051 m.
        nearSection(detail.back().arcLengthMeters, 0.051, 1.0e-12, "Selected contour arc length is incorrect.");
        requireSection(selected.annotations.size() == 7, "Exact 10 mm annotations plus endpoint were not created.");
        for(std::size_t i = 0; i < 6; ++i) {
            nearSection(selected.annotations[i].arcLengthMeters, i * 0.01, 1.0e-12,
                "Annotation positions must use an exact arc-length lattice across triangle boundaries.");
        }
        nearSection(selected.annotations.back().arcLengthMeters, 0.051, 1.0e-12,
            "The short final endpoint interval must remain explicit.");
        const auto& bend = selected.annotations[3];
        nearSection(bend.positionMeters.y(), 0.0142, 1.0e-12, "Equal arc sampling failed around the bend.");
        nearSection(bend.positionMeters.z(), 0.0296, 1.0e-12, "Equal arc sampling failed around the bend.");
        nearSection(bend.thicknessMeters, 70.0e-6, 1.0e-12, "Thickness interpolation failed around the bend.");
    }

    void testDisconnectedAndSeamValues()
    {
        const auto all = rectangle(-0.1, -0.1, 0.1, 0.1);
        const std::vector<ThicknessPickSectionSegment> separate{
            segment(0.0, 0.0, 0.0, 0.005, 0.0, 0.0),
            segment(0.0, 0.02, 0.0, 0.025, 100.0e-6, 100.0e-6) };
        const auto selected = buildSelectedYzThicknessSection(separate, all, 0.003);
        requireSection(selected.profiles.size() == 2, "Disconnected section components were bridged.");
        requireSection(selected.annotations.size() == 6, "Each component needs an independent sampling origin.");
        nearSection(selected.annotations[3].arcLengthMeters, 0.0, 0.0, "Second component did not restart at arc zero.");
        requireSection(selected.annotations[3].profileIndex == 1, "Annotations lost their component identity.");

        const std::vector<ThicknessPickSectionSegment> seam{
            segment(0.0, 0.0, 0.0, 0.01, 25.0e-6, 25.0e-6),
            segment(0.0, 0.01, 0.0, 0.02, 75.0e-6, 75.0e-6, 1) };
        const auto jump = buildSelectedYzThicknessSection(seam, all, 0.005);
        std::vector<double> seamValues;
        for(const auto& value : jump.profiles.front()) {
            if(std::abs(value.positionMeters.z() - 0.01) < 1.0e-12) seamValues.push_back(value.thicknessMeters);
        }
        requireSection(seamValues.size() == 2, "A discontinuous cell seam must retain both endpoint values.");
        nearSection(seamValues[0], 25.0e-6, 1.0e-12, "First side of thickness jump was averaged.");
        nearSection(seamValues[1], 75.0e-6, 1.0e-12, "Second side of thickness jump was averaged.");
        auto duplicate = seam;
        duplicate.push_back(seam.front());
        requireSection(buildSelectedYzThicknessSection(duplicate, all, 0.005).profiles.size() == 1,
            "Identical duplicate edges should be deduplicated.");
        duplicate.back().first.thicknessMeters = duplicate.back().second.thicknessMeters = 99.0e-6;
        rejectsSection([&] { buildSelectedYzThicknessSection(duplicate, all, 0.005); },
            "Conflicting coincident faces must not be silently mixed.");
        auto branch = seam;
        branch.push_back(segment(0.0, 0.01, 0.01, 0.01));
        rejectsSection([&] { buildSelectedYzThicknessSection(branch, all, 0.005); },
            "A branching section must not be ordered by arbitrary triangle insertion.");
    }

    void testExtractionCoplanarAndTransform()
    {
        assetcore::SubMeshDesc mesh;
        mesh.geometry.positions = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.02f, 0.0f },
            { 0.0f, 0.02f, 0.03f }, { 0.0f, 0.0f, 0.03f } };
        mesh.geometry.indices = { 0, 1, 2, 0, 2, 3 };
        assetcore::ModelDesc model; model.addSubMesh(mesh);
        smrobot::visualization::SurfaceScalarOverlay field;
        field.unit = "m"; field.subMeshes.resize(1); field.subMeshes[0].values = { 0.0, 10.0e-6, 20.0e-6, 30.0e-6 };
        auto world = Eigen::Isometry3d::Identity();
        world.translation() = Eigen::Vector3d(0.0, 0.1, -0.2);
        const auto extracted = extractYzThicknessSegments(model, field, world);
        requireSection(extracted.size() == 4, "Coplanar triangulation diagonal leaked into the section contour.");
        for(const auto& edge : extracted) {
            const auto delta = edge.second.positionMeters - edge.first.positionMeters;
            requireSection(std::abs(delta.y()) < 1.0e-12 || std::abs(delta.z()) < 1.0e-12,
                "Coplanar mesh internal diagonal was exported.");
            requireSection(edge.first.positionMeters.y() >= 0.1 - 1.0e-12
                && edge.first.positionMeters.z() <= -0.17 + 1.0e-8, "World transform was not applied to section extraction.");
            nearSection(edge.first.normalWorld.norm(), 1.0, 1.0e-12,
                "Coplanar boundary needs a unit outward section normal.");
        }
        const auto section = buildSelectedYzThicknessSection(extracted, rectangle(0.09, -0.21, 0.13, -0.16), 0.01);
        requireSection(section.profiles.size() == 1, "Coplanar boundary did not form one closed contour.");
        nearSection(section.profiles.front().back().arcLengthMeters, 0.1, 1.0e-8,
            "Coplanar contour perimeter is incorrect.");

        auto capped = model;
        assetcore::SubMeshDesc side;
        side.geometry.positions = { { 0.0f, 0.02f, 0.0f }, { 0.0f, 0.02f, 0.03f },
            { 0.01f, 0.02f, 0.0f } };
        side.geometry.indices = { 0, 1, 2 }; capped.addSubMesh(side);
        auto cappedField = field;
        cappedField.subMeshes.emplace_back();
        cappedField.subMeshes.back().values = { 125.0e-6, 125.0e-6, 125.0e-6 };
        const auto withSide = extractYzThicknessSegments(capped, cappedField, world);
        requireSection(withSide.size() == 4, "A coplanar cut face duplicated the coated side boundary.");
        const auto sideOnly = buildSelectedYzThicknessSection(withSide,
            rectangle(0.119, -0.199, 0.121, -0.171), 0.01);
        requireSection(sideOnly.profiles.size() == 1, "Coated side selection should remain one contour.");
        for(const auto& sample : sideOnly.profiles.front()) {
            nearSection(sample.thicknessMeters, 125.0e-6, 1.0e-12,
                "Coplanar cut-face thickness contaminated the coated side boundary.");
        }

        // A local YZ face rotated about Y becomes an XY face, then the inverse
        // world rotation restores the same section, with an independent offset.
        model.get_local() = glm::mat4(1.0f);
        model.get_local()[0][0] = 0.0f; model.get_local()[0][2] = -1.0f;
        model.get_local()[2][0] = 1.0f; model.get_local()[2][2] = 0.0f;
        world.linear() << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
        requireSection(extractYzThicknessSegments(model, field, world).size() == 4,
            "Local and world rotations were not composed correctly.");
    }

    void testClippingAndFailures()
    {
        const std::vector<ThicknessPickSectionSegment> source{ segment(0.0, -0.02, 0.0, 0.02, 0.0, 40.0e-6) };
        const auto polygon = rectangle(-0.01, -0.005, 0.01, 0.005);
        const auto clipped = clipYzThicknessSegments(source, polygon);
        requireSection(clipped.size() == 1, "Polygon must clip segments even when neither endpoint is inside.");
        nearSection(clipped[0].first.thicknessMeters, 15.0e-6, 1.0e-12, "Clipped entry thickness is incorrect.");
        nearSection(clipped[0].second.thicknessMeters, 25.0e-6, 1.0e-12, "Clipped exit thickness is incorrect.");
        const auto outside = rectangle(0.1, 0.1, 0.2, 0.2);
        requireSection(clipYzThicknessSegments(source, outside).empty(), "Nonmatching lasso should produce no segments.");
        rejectsSection([&] { buildSelectedYzThicknessSection(source, outside, 0.01); },
            "Empty section selection must not export a misleading workbook.");
        rejectsSection([&] { buildSelectedYzThicknessSection(source, {}, 0.01); }, "Missing lasso was accepted.");
        for(double bad : { 0.0, -1.0, std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN(), 1.0e-12 }) {
            rejectsSection([&] { buildSelectedYzThicknessSection(source, polygon, bad); },
                "Invalid or excessive annotation spacing must be rejected.");
        }
        auto invalid = source; invalid[0].first.thicknessMeters = -1.0;
        rejectsSection([&] { buildSelectedYzThicknessSection(invalid, polygon, 0.01); }, "Negative thickness was accepted.");
    }
}

void runSectionSamplingTests()
{
    testSelectedSurfaceAndArcLength();
    testDisconnectedAndSeamValues();
    testExtractionCoplanarAndTransform();
    testClippingAndFailures();
}
