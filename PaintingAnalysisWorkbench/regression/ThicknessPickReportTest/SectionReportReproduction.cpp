#include <ThicknessPickReport.h>
#include <ThicknessSectionSchematic.h>

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
    using robot_qt_viewer::ThicknessPickReportSample;
    void requireReproduction(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    ThicknessPickReportSample readPoint(const QJsonValue& value)
    {
        const QJsonArray row = value.toArray();
        requireReproduction(row.size() == 6, "Real report fixture row must contain six fields.");
        ThicknessPickReportSample sample;
        sample.triangleIndex = static_cast<std::size_t>(row[0].toDouble());
        sample.positionMeters = Eigen::Vector3d(row[1].toDouble(), row[2].toDouble(), row[3].toDouble());
        sample.thicknessMeters = row[4].toDouble();
        sample.arcLengthMeters = row[5].toDouble();
        return sample;
    }

    // The selected real fixture is ordered bottom-to-top on the right surface.
    // Original report rows do not store mesh normals; reconstruct its outward
    // side from the ordered YZ contour, as verified against the source image.
    Eigen::Vector3d fixtureNormal(const std::vector<ThicknessPickReportSample>& profile, std::size_t index)
    {
        std::size_t left = index, right = index;
        while(left > 0 && (profile[index].positionMeters - profile[left].positionMeters).norm() < 1e-12) --left;
        while(right + 1 < profile.size() && (profile[index].positionMeters - profile[right].positionMeters).norm() < 1e-12) ++right;
        const Eigen::Vector3d tangent = profile[right].positionMeters - profile[left].positionMeters;
        requireReproduction(tangent.norm() > 0.0, "Cannot reconstruct the real fixture section direction.");
        return Eigen::Vector3d(0.0, tangent.z(), -tangent.y()).normalized();
    }

    double cross2(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        return a.x() * b.y() - a.y() * b.x();
    }

    std::size_t countStrictCrossings(const std::vector<Eigen::Vector2d>& points)
    {
        std::size_t count = 0;
        for(std::size_t i = 1; i < points.size(); ++i) {
            const auto a = points[i - 1]; const auto b = points[i];
            for(std::size_t j = i + 2; j < points.size(); ++j) {
                const auto c = points[j - 1]; const auto d = points[j];
                if(std::max(a.x(), b.x()) < std::min(c.x(), d.x()) ||
                    std::max(c.x(), d.x()) < std::min(a.x(), b.x()) ||
                    std::max(a.y(), b.y()) < std::min(c.y(), d.y()) ||
                    std::max(c.y(), d.y()) < std::min(a.y(), b.y())) continue;
                const double p = cross2(b - a, c - a), q = cross2(b - a, d - a);
                const double r = cross2(d - c, a - c), s = cross2(d - c, b - c);
                if(p * q < 0.0 && r * s < 0.0) ++count;
            }
        }
        return count;
    }
}

void reproduceSectionReport(const QString& outputDirectory)
{
    using namespace robot_qt_viewer;
    const QString fixturePath = QString::fromUtf8(qgetenv("THICKNESS_PICK_REPORT_FIXTURE"));
    if(fixturePath.isEmpty()) return;
    requireReproduction(!outputDirectory.isEmpty(), "Real report reproduction needs --output.");
    QFile source(fixturePath);
    requireReproduction(source.open(QIODevice::ReadOnly), "Cannot open the real report fixture.");
    const QJsonObject fixture = QJsonDocument::fromJson(source.readAll()).object();
    ThicknessPickReportSection section;
    section.annotationSpacingMeters = fixture.value(QStringLiteral("spacingMeters")).toDouble();
    section.profiles.emplace_back();
    auto& profile = section.profiles.front();
    for(const auto& row : fixture.value(QStringLiteral("samples")).toArray()) profile.push_back(readPoint(row));
    requireReproduction(profile.size() > 2, "Real report fixture has no usable contour.");
    for(std::size_t i = 0; i < profile.size(); ++i) profile[i].normalWorld = fixtureNormal(profile, i);
    for(const auto& row : fixture.value(QStringLiteral("annotations")).toArray()) {
        auto sample = readPoint(row);
        const auto it = std::lower_bound(profile.begin(), profile.end(), sample.arcLengthMeters,
            [](const auto& point, double arc) { return point.arcLengthMeters < arc; });
        const auto index = std::min<std::size_t>(std::distance(profile.begin(), it), profile.size() - 1);
        sample.normalWorld = fixtureNormal(profile, index);
        section.annotations.push_back(sample);
    }
    const auto geometry = buildThicknessSectionSchematic(profile, section.annotations);
    std::vector<Eigen::Vector2d> coating;
    QJsonArray geometryRows;
    for(const auto& point : geometry.points) {
        coating.push_back(point.coatingYZ);
        geometryRows.push_back(QJsonArray{ point.arcLengthMeters, point.baseYZ.x(), point.baseYZ.y(),
            point.coatingYZ.x(), point.coatingYZ.y(), point.normalYZ.x(), point.normalYZ.y() });
    }
    const auto crossings = countStrictCrossings(coating);
    requireReproduction(crossings == 0, "The real report's smoothed coating still crosses itself.");
    requireReproduction(geometry.annotations.size() == section.annotations.size(), "Real annotations were dropped.");
    for(std::size_t i = 0; i < section.annotations.size(); ++i) {
        const auto& rendered = geometry.annotations[i];
        const auto& raw = section.annotations[i];
        requireReproduction((rendered.baseYZ - raw.positionMeters.tail<2>()).norm() < 1e-12,
            "The real measurement point moved during drawing smoothing.");
        requireReproduction(std::abs((rendered.coatingYZ - rendered.baseYZ).norm() -
            raw.thicknessMeters * geometry.thicknessMultiplier) < 1e-12,
            "The real label's dimension no longer represents the original thickness.");
    }
    const auto result = exportThicknessPickReport(profile, fixture.value(QStringLiteral("modelName")).toString(),
        QDir(outputDirectory).filePath(QStringLiteral("reproduced-user-report")), &section);
    QJsonObject manifest;
    manifest.insert(QStringLiteral("xlsxPath"), result.filePath);
    manifest.insert(QStringLiteral("sampleCount"), static_cast<double>(profile.size()));
    manifest.insert(QStringLiteral("annotationCount"), static_cast<double>(section.annotations.size()));
    manifest.insert(QStringLiteral("displayMultiplier"), geometry.thicknessMultiplier);
    manifest.insert(QStringLiteral("smoothingWindowMeters"), geometry.smoothingWindowMeters);
    manifest.insert(QStringLiteral("crossings"), static_cast<double>(crossings));
    manifest.insert(QStringLiteral("geometry"), geometryRows);
    QFile output(QDir(outputDirectory).filePath(QStringLiteral("real-section-reproduction.json")));
    const QByteArray bytes = QJsonDocument(manifest).toJson(QJsonDocument::Compact);
    requireReproduction(output.open(QIODevice::WriteOnly) && output.write(bytes) == bytes.size(),
        "Cannot write the real report reproduction diagnostics.");
    std::cout << "Real report reproduction: " << profile.size() << " rows, " << section.annotations.size()
        << " labels, display x" << geometry.thicknessMultiplier << ", " << crossings << " crossings.\n";
}
