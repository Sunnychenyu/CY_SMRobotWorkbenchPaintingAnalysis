#include <ThicknessPickReport.h>
#include <ThicknessPickSampling.h>

#include <AssetCore/ModelDesc.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>
#include <QTemporaryDir>
#include <QStringList>
#include <QXmlStreamReader>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

void runDynamicSurfaceReportTests();
void runSectionSamplingTests();
void runSectionDialogTests(const QString& outputDirectory);
void runRealSectionMeshTest();
void runSectionSchematicTests();
void reproduceSectionReport(const QString& outputDirectory);

namespace
{
    using robot_qt_viewer::ThicknessPickReportSample;
    using robot_qt_viewer::exportThicknessPickReport;
    using robot_qt_viewer::sampleYzThickness;
    using Overlay = smrobot::visualization::SurfaceScalarOverlay;

    void require(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    void near(double actual, double expected, double tolerance, const char* message)
    {
        require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
    }

    template<typename Action>
    void rejects(Action action, const char* message)
    {
        bool rejected = false;
        try { action(); }
        catch(const std::exception&) { rejected = true; }
        require(rejected, message);
    }

    assetcore::ModelDesc triangleModel(const std::array<Eigen::Vector3f, 3>& positions,
        bool indexed = true)
    {
        assetcore::SubMeshDesc mesh;
        mesh.geometry.positions.assign(positions.begin(), positions.end());
        if(indexed) mesh.geometry.indices = { 0, 1, 2 };
        assetcore::ModelDesc model;
        model.addSubMesh(mesh);
        return model;
    }

    assetcore::ModelDesc crossingModel(bool indexed = true)
    {
        return triangleModel({ Eigen::Vector3f(-0.001f, 0.0f, 0.0f),
            Eigen::Vector3f(0.003f, 0.002f, 0.0f),
            Eigen::Vector3f(-0.001f, 0.002f, 0.002f) }, indexed);
    }

    Overlay overlay(const std::vector<double>& values)
    {
        Overlay result;
        result.objectId = "fixture";
        result.quantityName = "Thickness";
        result.unit = "m";
        result.subMeshes.resize(1);
        result.subMeshes.front().values = values;
        return result;
    }

    void checkCrossing(std::vector<ThicknessPickReportSample> samples,
        double yOffset = 0.0, double zOffset = 0.0, double tolerance = 2.0e-10)
    {
        require(samples.size() >= 3, "The triangle crossing must contain interior samples.");
        std::sort(samples.begin(), samples.end(), [](const auto& first, const auto& second) {
            return first.positionMeters.y() < second.positionMeters.y();
        });
        near(samples.front().positionMeters.y(), yOffset + 0.0005, tolerance,
            "The first non-vertex plane intersection is incorrect.");
        near(samples.front().positionMeters.z(), zOffset, tolerance,
            "The first intersection has the wrong height.");
        near(samples.back().positionMeters.y(), yOffset + 0.002, tolerance,
            "The last non-vertex plane intersection is incorrect.");
        near(samples.back().positionMeters.z(), zOffset + 0.0015, tolerance,
            "The last intersection has the wrong height.");
        for(std::size_t index = 0; index < samples.size(); ++index) {
            const auto& sample = samples[index];
            require(sample.triangleIndex == 0, "Single-triangle provenance changed.");
            require(sample.positionMeters.x() == 0.0, "Report positions must lie exactly on X = 0.");
            const double y = sample.positionMeters.y() - yOffset;
            const double z = sample.positionMeters.z() - zOffset;
            near(z, y - 0.0005, tolerance, "A sample left the analytical intersection line.");
            // The source field is t = 10 um + 0.01 X + 0.02 Y + 0.03 Z.
            // This independently checks the interior field, not just endpoint values.
            near(sample.thicknessMeters, 10.0e-6 + 0.02 * y + 0.03 * z,
                tolerance * 0.1, "Triangle interpolation disagrees with the analytical field.");
            if(index > 0) {
                const double distance = (sample.positionMeters - samples[index - 1].positionMeters).norm();
                require(distance > 0.0 && distance <= 0.0002 + tolerance,
                    "Samples must cover the crossing with spacing at most 200 um.");
            }
        }
    }

    void testCrossingAndTransforms()
    {
        const auto field = overlay({ 0.0, 80.0e-6, 100.0e-6 });
        for(bool indexed : { true, false }) {
            const auto model = crossingModel(indexed);
            checkCrossing(sampleYzThickness(model, field, Eigen::Isometry3d::Identity()));
        }

        auto model = triangleModel({ Eigen::Vector3f(0.0f, 0.0f, -0.001f),
            Eigen::Vector3f(0.002f, 0.0f, 0.003f),
            Eigen::Vector3f(0.002f, 0.002f, -0.001f) });
        // Local: (x,y,z) -> (-y + .003, x + .004, z + .005).
        auto& local = model.get_local();
        local = glm::mat4(1.0f);
        local[0][0] = 0.0f; local[0][1] = 1.0f;
        local[1][0] = -1.0f; local[1][1] = 0.0f;
        local[3][0] = 0.003f; local[3][1] = 0.004f; local[3][2] = 0.005f;
        // World: (x,y,z) -> (z - .005, y + .006, -x + .007).
        Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
        world.linear() << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
        world.translation() = Eigen::Vector3d(-0.005, 0.006, 0.007);
        // Composed geometry is (z, x + .010, y + .004), including the float SDK transform.
        checkCrossing(sampleYzThickness(model, field, world), 0.010, 0.004, 2.0e-9);
    }

    void testCoplanarZeroAndHardEdges()
    {
        const auto coplanar = triangleModel({ Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            Eigen::Vector3f(0.0f, 0.001f, 0.0f),
            Eigen::Vector3f(0.0f, 0.0f, 0.001f) });
        const auto samples = sampleYzThickness(coplanar,
            overlay({ 0.0, 20.0e-6, 30.0e-6 }), Eigen::Isometry3d::Identity());
        require(samples.size() > 3, "Coplanar triangle edges require interior samples.");
        bool hasOrigin = false, hasYCorner = false, hasZCorner = false;
        for(const auto& sample : samples) {
            const double y = sample.positionMeters.y(), z = sample.positionMeters.z();
            require(sample.positionMeters.x() == 0.0, "Coplanar sample left the YZ plane.");
            require(std::abs(y) < 1.0e-10 || std::abs(z) < 1.0e-10 ||
                std::abs(y + z - 0.001) < 1.0e-10,
                "Coplanar sampling must follow triangle edges.");
            near(sample.thicknessMeters, 0.02 * y + 0.03 * z, 1.0e-11,
                "Coplanar edge interpolation is incorrect.");
            hasOrigin |= y == 0.0 && z == 0.0;
            hasYCorner |= std::abs(y - 0.001) < 1.0e-10 && z == 0.0;
            hasZCorner |= y == 0.0 && std::abs(z - 0.001) < 1.0e-10;
        }
        require(hasOrigin && hasYCorner && hasZCorner, "Coplanar triangle corners were omitted.");

        auto model = crossingModel();
        auto field = overlay({ 0.0, 0.0, 0.0 });
        const auto zero = sampleYzThickness(model, field, Eigen::Isometry3d::Identity());
        require(!zero.empty() && std::all_of(zero.begin(), zero.end(), [](const auto& sample) {
            return sample.thicknessMeters == 0.0;
        }), "Uncoated zero thickness must be retained.");

        const assetcore::SubMeshDesc duplicate = model.subMesh(0);
        model.addSubMesh(duplicate);
        field.subMeshes[0].values.assign(3, 25.0e-6);
        field.subMeshes.push_back(field.subMeshes[0]);
        field.subMeshes[1].values.assign(3, 75.0e-6);
        const auto hard = sampleYzThickness(model, field, Eigen::Isometry3d::Identity());
        require(hard.size() == zero.size() * 2, "Coincident hard-edge values were merged or omitted.");
        for(const auto& first : hard) {
            require(first.triangleIndex < 2, "Triangle numbering must span submeshes.");
            near(first.thicknessMeters, first.triangleIndex == 0 ? 25.0e-6 : 75.0e-6,
                1.0e-14, "Hard-edge thickness was blended between independent triangles.");
            require(std::any_of(hard.begin(), hard.end(), [&](const auto& other) {
                return other.triangleIndex != first.triangleIndex &&
                    (other.positionMeters - first.positionMeters).norm() < 1.0e-12;
            }), "Both thickness values at a hard-edge position must survive.");
        }
        field.subMeshes[1].values.assign(3, 25.0e-6);
        require(sampleYzThickness(model, field, Eigen::Isometry3d::Identity()).size() == zero.size(),
            "Equal position and thickness duplicates should be exported once.");
    }

    void testSmallScalePlaneGeometry()
    {
        const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
        const auto field = overlay({ 0.0, 20.0e-6, 40.0e-6 });
        const auto positive = triangleModel({ Eigen::Vector3f(1.0e-13f, 0.0f, 0.0f),
            Eigen::Vector3f(1.0e-13f, 1.0e-13f, 0.0f),
            Eigen::Vector3f(1.0e-13f, 0.0f, 1.0e-13f) });
        rejects([&] { sampleYzThickness(positive, field, identity); },
            "A tiny triangle wholly on positive X must not be snapped onto X = 0.");

        const auto crossing = triangleModel({ Eigen::Vector3f(-1.0e-13f, 0.0f, 0.0f),
            Eigen::Vector3f(1.0e-13f, 1.0e-13f, 0.0f),
            Eigen::Vector3f(1.0e-13f, 0.0f, 1.0e-13f) });
        const auto samples = sampleYzThickness(crossing, field, identity);
        require(samples.size() == 2, "The two endpoints of a tiny crossing must remain distinct.");
        require((samples[0].positionMeters - samples[1].positionMeters).stableNorm() > 1.0e-14,
            "Tiny intersection endpoints were merged by an absolute tolerance.");
        for(const auto& sample : samples) {
            require(sample.positionMeters.x() == 0.0, "A tiny crossing sample must lie exactly on X = 0.");
            near(sample.positionMeters.y() + sample.positionMeters.z(), 5.0e-14, 1.0e-20,
                "A tiny crossing endpoint has incorrect coordinates.");
            near(sample.thicknessMeters, sample.positionMeters.y() > 0.0 ? 10.0e-6 : 20.0e-6,
                1.0e-14, "Tiny crossing interpolation is incorrect.");
        }

        const auto collapsed = triangleModel({ Eigen::Vector3f::Zero(),
            Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero() });
        rejects([&] { sampleYzThickness(collapsed, field, identity); },
            "A triangle with all three vertices at the origin must be rejected.");
    }

    void testSamplingFailures()
    {
        const auto model = crossingModel();
        const auto good = overlay({ 0.0, 80.0e-6, 100.0e-6 });
        const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
        auto invalid = good;
        invalid.subMeshes.clear();
        rejects([&] { sampleYzThickness(model, invalid, identity); }, "Missing overlay must be rejected.");
        invalid = good;
        invalid.unit = "%";
        rejects([&] { sampleYzThickness(model, invalid, identity); }, "A non-thickness overlay must be rejected.");
        invalid = good;
        invalid.subMeshes.front().values.pop_back();
        rejects([&] { sampleYzThickness(model, invalid, identity); }, "Mismatched field size must be rejected.");
        for(double bad : { -1.0e-6, std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity() }) {
            invalid = good;
            invalid.subMeshes.front().values[1] = bad;
            rejects([&] { sampleYzThickness(model, invalid, identity); }, "Invalid vertex thickness must be rejected.");
        }
        auto shifted = identity;
        shifted.translation().x() = 0.1;
        rejects([&] { sampleYzThickness(model, good, shifted); }, "A part outside X = 0 must be rejected.");
        auto brokenTransform = identity;
        brokenTransform.translation().x() = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { sampleYzThickness(model, good, brokenTransform); }, "Non-finite transform must be rejected.");
        auto brokenMesh = model;
        brokenMesh.subMeshes().front().geometry.indices[2] = 999;
        rejects([&] { sampleYzThickness(brokenMesh, good, identity); }, "Invalid mesh index must be rejected.");
    }

    std::vector<ThicknessPickReportSample> exportFixture()
    {
        return {
            { 0, Eigen::Vector3d(0.0, -0.00125, 0.0025), 0.0 },
            { 7, Eigen::Vector3d(0.0, 0.002, 0.000125), 12.5e-6 },
            { 8, Eigen::Vector3d(0.0, 0.002, 0.000125), 87.75e-6 },
            { 999, Eigen::Vector3d(0.0, -1.25e-7, 3.5e-6), 1.0e-15 }
        };
    }

    QString fixtureModelName()
    {
        return QStringLiteral("\u6d4b\u8bd5\u96f6\u4ef6 <A&B> \"quoted\"");
    }

    void checkExport(const robot_qt_viewer::ThicknessPickReportExportResult& result,
        const QString& directory, std::size_t expectedCount)
    {
        require(result.sampleCount == expectedCount, "Export count differs from the source samples.");
        const QFileInfo info(result.filePath);
        require(info.exists() && info.isFile() && info.suffix() == QStringLiteral("xlsx") &&
            info.absolutePath() == QDir(directory).absolutePath(), "The XLSX must exist in the requested directory.");
        QFile file(result.filePath);
        require(file.open(QIODevice::ReadOnly) && file.size() > 1000 &&
            file.read(4) == QByteArray("PK\003\004", 4), "The report is not an XLSX ZIP container.");
    }

    void testExports(const QString& temporaryRoot)
    {
        const QString directory = QDir(temporaryRoot).filePath(QStringLiteral("\u62fe\u53d6\u539a\u5ea6\u6d4b\u8bd5"));
        const auto samples = exportFixture();
        const QString modelName = QStringLiteral("taijie_\u7c73");
        const auto first = exportThicknessPickReport(samples, modelName, directory);
        QFile original(first.filePath);
        require(original.open(QIODevice::ReadOnly), "Cannot read the original report.");
        const auto originalBytes = original.readAll();
        original.close();
        const auto second = exportThicknessPickReport(samples, modelName, directory);
        const auto third = exportThicknessPickReport(samples, modelName, directory);
        checkExport(first, directory, samples.size());
        checkExport(second, directory, samples.size());
        checkExport(third, directory, samples.size());
        const QString stem = QStringLiteral("\u53f0\u9636\u4ef6_\u7c73_YZ\u622a\u9762\u539a\u5ea6\u62a5\u544a");
        require(QFileInfo(first.filePath).fileName() == stem + QStringLiteral(".xlsx") &&
            QFileInfo(second.filePath).fileName() == stem + QStringLiteral("_01.xlsx") &&
            QFileInfo(third.filePath).fileName() == stem + QStringLiteral("_02.xlsx"),
            "Report filenames must identify the model and use numbered duplicates without timestamps.");
        require(QDir(directory).entryList(QDir::Files).size() == 3 &&
            original.open(QIODevice::ReadOnly) && original.readAll() == originalBytes,
            "Repeated exports must preserve the original report bytes.");

        std::vector<std::vector<ThicknessPickReportSample>> invalidCases{ {} };
        for(int kind = 0; kind < 7; ++kind) {
            auto invalid = samples;
            switch(kind) {
            case 0: invalid[0].thicknessMeters = -1.0e-6; break;
            case 1: invalid[0].thicknessMeters = std::numeric_limits<double>::quiet_NaN(); break;
            case 2: invalid[0].positionMeters.y() = std::numeric_limits<double>::infinity(); break;
            case 3: invalid[0].positionMeters.x() = 1.0e-15; break;
            case 4: invalid[0].thicknessMeters = std::numeric_limits<double>::max(); break;
            case 5: invalid[0].positionMeters.z() = std::numeric_limits<double>::max(); break;
            case 6: invalid[0].triangleIndex = std::numeric_limits<std::size_t>::max(); break;
            }
            invalidCases.push_back(std::move(invalid));
        }
        for(std::size_t index = 0; index < invalidCases.size(); ++index) {
            const QString rejectedDirectory = QDir(temporaryRoot).filePath(QStringLiteral("invalid_%1").arg(static_cast<qulonglong>(index)));
            rejects([&] { exportThicknessPickReport(invalidCases[index], fixtureModelName(), rejectedDirectory); },
                "Invalid report data must be rejected.");
            require(!QFileInfo::exists(rejectedDirectory), "Invalid report data must not create a directory or partial file.");
        }
        const QString tooLongDirectory = QDir(temporaryRoot).filePath(QStringLiteral("invalid_name"));
        rejects([&] { exportThicknessPickReport(samples, QString(32768, QChar('x')), tooLongDirectory); },
            "A model name beyond the Excel cell limit must be rejected.");
        require(!QFileInfo::exists(tooLongDirectory), "An invalid model name must not leave output files.");
        rejects([&] { exportThicknessPickReport(samples, fixtureModelName(), QString{}); },
            "An empty output directory must be rejected.");

        const QString blockedPath = QDir(temporaryRoot).filePath(QStringLiteral("ordinary-file"));
        QFile blocked(blockedPath);
        require(blocked.open(QIODevice::WriteOnly) && blocked.write("sentinel") == 8,
            "Cannot create blocked-output fixture.");
        blocked.close();
        rejects([&] { exportThicknessPickReport(samples, fixtureModelName(), blockedPath); },
            "An ordinary file must not be accepted as an output directory.");
        require(blocked.open(QIODevice::ReadOnly) && blocked.readAll() == QByteArray("sentinel"),
            "A failed export must preserve existing files.");
    }

    robot_qt_viewer::ThicknessPickReportSection sectionExportFixture()
    {
        robot_qt_viewer::ThicknessPickReportSection section;
        for(std::size_t profileIndex = 0; profileIndex < 2; ++profileIndex) {
            std::vector<ThicknessPickReportSample> profile;
            const int count = profileIndex == 0 ? 680 : 100;
            for(int index = 0; index <= count; ++index) {
                const double t = static_cast<double>(index) / count;
                ThicknessPickReportSample sample;
                sample.triangleIndex = profileIndex * 1000 + index;
                sample.profileIndex = profileIndex;
                sample.positionMeters = profileIndex == 0
                    ? Eigen::Vector3d(0, 0.085 + 0.018 * std::exp(-std::pow((t - 0.28) / 0.045, 2)) +
                        0.025 * std::exp(-std::pow((t - 0.76) / 0.045, 2)) + 0.015 * t, 0.34 * t)
                    : Eigen::Vector3d(0, 0.14 + 0.012 * std::sin(t * 1.5707963267948966), 0.4 + 0.05 * t);
                sample.thicknessMeters = t < 0.035 ? 0.0 : (80.0 + 100.0 * std::pow(std::sin(t * 7.0), 2)) * 1e-6;
                if(!profile.empty()) sample.arcLengthMeters = profile.back().arcLengthMeters +
                    (sample.positionMeters - profile.back().positionMeters).norm();
                profile.push_back(sample);
            }
            for(std::size_t index = 0; index < profile.size(); ++index) {
                const auto tangent = profile[std::min(index + 1, profile.size() - 1)].positionMeters -
                    profile[index == 0 ? 0 : index - 1].positionMeters;
                profile[index].normalWorld = Eigen::Vector3d(0.0, tangent.z(), -tangent.y()).normalized();
            }
            const auto atDistance = [&](double distance) {
                auto next = std::lower_bound(profile.begin(), profile.end(), distance,
                    [](const auto& sample, double value) { return sample.arcLengthMeters < value; });
                if(next == profile.begin()) return profile.front();
                if(next == profile.end()) return profile.back();
                const auto& previous = *(next - 1);
                const double t = (distance - previous.arcLengthMeters) / (next->arcLengthMeters - previous.arcLengthMeters);
                auto sample = previous;
                sample.positionMeters = previous.positionMeters + t * (next->positionMeters - previous.positionMeters);
                sample.normalWorld = (previous.normalWorld + t * (next->normalWorld - previous.normalWorld)).normalized();
                sample.thicknessMeters = previous.thicknessMeters + t * (next->thicknessMeters - previous.thicknessMeters);
                sample.arcLengthMeters = distance;
                return sample;
            };
            for(double distance = 0; distance < profile.back().arcLengthMeters; distance += section.annotationSpacingMeters) {
                section.annotations.push_back(atDistance(distance));
            }
            section.annotations.push_back(profile.back());
            section.profiles.push_back(std::move(profile));
        }
        return section;
    }

    QMap<QByteArray, QByteArray> storedZipParts(const QString& path)
    {
        QFile file(path);
        require(file.open(QIODevice::ReadOnly), "Cannot read section XLSX.");
        const QByteArray bytes = file.readAll();
        const auto value = [&](int offset, int size) {
            require(offset >= 0 && offset + size <= bytes.size(), "Truncated XLSX ZIP header.");
            quint32 result = 0;
            for(int index = 0; index < size; ++index) result |= static_cast<quint32>(static_cast<unsigned char>(bytes[offset + index])) << (8 * index);
            return result;
        };
        QMap<QByteArray, QByteArray> parts;
        int offset = 0;
        while(offset + 4 <= bytes.size() && value(offset, 4) == 0x04034b50) {
            require(value(offset + 8, 2) == 0, "Unexpected compressed test ZIP entry.");
            const int size = static_cast<int>(value(offset + 18, 4));
            const int nameLength = value(offset + 26, 2);
            const int extraLength = value(offset + 28, 2);
            const int dataOffset = offset + 30 + nameLength + extraLength;
            require(size >= 0 && dataOffset >= offset && dataOffset + size <= bytes.size(), "Truncated XLSX ZIP entry.");
            parts.insert(bytes.mid(offset + 30, nameLength), bytes.mid(dataOffset, size));
            offset = dataOffset + size;
        }
        return parts;
    }

    void testSectionExports(const QString& outputRoot, bool saveManifest)
    {
        auto section = sectionExportFixture();
        std::vector<ThicknessPickReportSample> samples;
        for(const auto& profile : section.profiles) samples.insert(samples.end(), profile.begin(), profile.end());
        const QString directory = QDir(outputRoot).filePath(QStringLiteral("section-fixture"));
        const auto result = exportThicknessPickReport(samples, QStringLiteral("Synthetic curved section - verification only"), directory, &section);
        checkExport(result, directory, samples.size());
        const auto parts = storedZipParts(result.filePath);
        require(parts.contains("xl/worksheets/sheet2.xml") && parts.contains("xl/worksheets/sheet3.xml") &&
            parts.contains("xl/drawings/drawing1.xml") && parts.contains("xl/drawings/_rels/drawing1.xml.rels") &&
            parts.contains("xl/worksheets/_rels/sheet2.xml.rels"), "The section XLSX must contain diagram, annotation and drawing relationship parts.");
        int images = 0;
        for(auto iterator = parts.begin(); iterator != parts.end(); ++iterator) {
            if(iterator.key().endsWith(".xml") || iterator.key().endsWith(".rels")) {
                QXmlStreamReader reader(iterator.value());
                while(!reader.atEnd()) reader.readNext();
                require(!reader.hasError(), "A generated XLSX XML part is not well formed.");
            }
            if(iterator.key().endsWith(".png")) {
                const QImage image = QImage::fromData(iterator.value(), "PNG");
                require(!image.isNull() && image.width() == 1440 && image.height() == 1500,
                    "An embedded schematic image is missing or invalid.");
                int titlePixels = 0;
                for(int y = 25; y < 70; ++y) {
                    for(int x = 40; x < 1400; ++x) {
                        if(qGray(image.pixel(x, y)) < 150) ++titlePixels;
                    }
                }
                require(titlePixels > 200, "The schematic must contain visible text, not only geometry and empty labels.");
                ++images;
            }
        }
        require(images == 4, "A long contour must have overview plus two detail panels, and the second contour a separate panel.");
        const auto annotationXml = parts.value("xl/worksheets/sheet3.xml");
        require(annotationXml.contains("P001") && annotationXml.contains("<c r=\"H6\" s=\"6\"><v>0</v></c>") &&
            annotationXml.contains("<c r=\"C7\" s=\"5\"><v>10</v></c>"),
            "Annotation data must retain true zero thickness and exact 10 mm arc spacing.");
        auto invalidSection = section;
        invalidSection.annotations.front().normalWorld.setZero();
        const QString rejected = QDir(outputRoot).filePath(QStringLiteral("invalid-section"));
        rejects([&] { exportThicknessPickReport(samples, QStringLiteral("Invalid section"), rejected, &invalidSection); },
            "A section with an undefined normal must not produce a misleading drawing.");
        require(!QFileInfo::exists(rejected), "Invalid section export must not create partial files.");
        if(saveManifest) {
            QJsonArray annotations;
            for(const auto& sample : section.annotations) annotations.append(QJsonArray{
                static_cast<double>(sample.profileIndex + 1), sample.arcLengthMeters * 1000.0,
                static_cast<double>(sample.triangleIndex), 0.0, sample.positionMeters.y() * 1000.0,
                sample.positionMeters.z() * 1000.0, sample.thicknessMeters * 1e6 });
            QJsonObject manifest;
            manifest.insert(QStringLiteral("xlsxPath"), result.filePath);
            manifest.insert(QStringLiteral("syntheticFixture"), true);
            manifest.insert(QStringLiteral("sampleCount"), static_cast<double>(samples.size()));
            manifest.insert(QStringLiteral("spacingMm"), section.annotationSpacingMeters * 1000.0);
            manifest.insert(QStringLiteral("imageCount"), images);
            manifest.insert(QStringLiteral("annotations"), annotations);
            const QString manifestPath = QDir(outputRoot).filePath(QStringLiteral("section-fixture-manifest.json"));
            QFile file(manifestPath);
            const QByteArray data = QJsonDocument(manifest).toJson(QJsonDocument::Indented);
            require(file.open(QIODevice::WriteOnly) && file.write(data) == data.size(), "Cannot save section fixture manifest.");
            std::cout << "Section fixture manifest: " << manifestPath.toUtf8().constData() << '\n';
        }
    }

    void saveFixture(const QString& outputRoot)
    {
        require(QDir().mkpath(outputRoot), "Cannot create fixture output directory.");
        const auto samples = exportFixture();
        const QString directory = QDir(outputRoot).filePath(QStringLiteral("\u62fe\u53d6\u539a\u5ea6\u6d4b\u8bd5"));
        const auto result = exportThicknessPickReport(samples, fixtureModelName(), directory);
        checkExport(result, directory, samples.size());
        QJsonArray rows;
        for(std::size_t index = 0; index < samples.size(); ++index) {
            const auto& sample = samples[index];
            rows.append(QJsonArray{ static_cast<double>(index + 1),
                static_cast<double>(sample.triangleIndex), 0.0,
                sample.positionMeters.y() * 1000.0, sample.positionMeters.z() * 1000.0,
                sample.thicknessMeters * 1.0e6 });
        }
        QJsonObject manifest;
        manifest.insert(QStringLiteral("xlsxPath"), result.filePath);
        manifest.insert(QStringLiteral("modelName"), fixtureModelName());
        manifest.insert(QStringLiteral("headerRow"), 8);
        manifest.insert(QStringLiteral("firstDataRow"), 9);
        manifest.insert(QStringLiteral("expectedRows"), rows);
        const QString manifestPath = QDir(outputRoot).filePath(QStringLiteral("fixture-manifest.json"));
        QFile file(manifestPath);
        const QByteArray json = QJsonDocument(manifest).toJson(QJsonDocument::Indented);
        require(file.open(QIODevice::WriteOnly) && file.write(json) == json.size(), "Cannot save independent-validation manifest.");
        std::cout << "Fixture manifest: " << manifestPath.toUtf8().constData() << '\n';
    }
}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    try {
        QString output;
        const QStringList arguments = application.arguments();
        if(arguments.size() != 1) {
            require(arguments.size() == 3 && arguments[1] == QStringLiteral("--output"),
                "Usage: ThicknessPickReportTest [--output <directory>]");
            output = arguments[2];
        }
        QTemporaryDir temporary;
        require(temporary.isValid(), "Cannot create regression temporary directory.");
        testCrossingAndTransforms();
        testCoplanarZeroAndHardEdges();
        testSmallScalePlaneGeometry();
        testSamplingFailures();
        testExports(temporary.path());
        runDynamicSurfaceReportTests();
        runSectionSamplingTests();
        runSectionDialogTests(output);
        runRealSectionMeshTest();
        runSectionSchematicTests();
        reproduceSectionReport(output);
        testSectionExports(temporary.path(), false);
        if(!output.isEmpty()) {
            saveFixture(output);
            testSectionExports(output, true);
        }
        std::cout << "ThicknessPickReportTest passed.\n";
        return 0;
    } catch(const std::exception& exception) {
        std::cerr << "ThicknessPickReportTest failed: " << exception.what() << '\n';
        return 1;
    }
}
