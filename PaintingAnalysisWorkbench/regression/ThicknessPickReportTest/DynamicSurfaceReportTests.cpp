#include <DynamicSurfaceDisplayAdapter.h>
#include <CoatingAnalysisSession.h>
#include <ThicknessPickSampling.h>
#include <LocalDepositionCorrectionAdapter.h>
#include <QDataStream>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTemporaryDir>

#include <AssetCore/ModelDesc.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
    using robot_qt_viewer::DynamicSurfaceView;
    using robot_qt_viewer::ThicknessPickReportSample;
    using robot_qt_viewer::makeDynamicSurfaceReportView;
    using robot_qt_viewer::sampleYzThickness;

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

    spraythickness::DynamicSurfaceSnapshot snapshotFixture()
    {
        auto substrate = std::make_shared<sprayworkpiece::WorkpieceModel>();
        substrate->samples.resize(3);
        // The world X=0 cut runs from (Y,Z)=(.0105,.020) to (.012,.0215).
        // Its independent analytical thickness field is t=.05*(Z-.020).
        substrate->samples[0].position = Eigen::Vector3d(-0.001, 0.010, 0.020);
        substrate->samples[1].position = Eigen::Vector3d(0.003, 0.012, 0.020);
        substrate->samples[2].position = Eigen::Vector3d(-0.001, 0.012, 0.022);
        substrate->triangleIndices = { 0, 1, 2 };
        for(auto& sample : substrate->samples) sample.normal = Eigen::Vector3d::UnitZ();
        spraythickness::DynamicSurfaceSnapshot snapshot;
        snapshot.substrate = substrate;
        snapshot.thicknessMeters = { 0.0, 0.0, 100.0e-6 };
        // The evolved position follows a curved growth history: displacement
        // is deliberately different from accumulated normal thickness.
        for(const auto& sample : substrate->samples) {
            snapshot.positions.push_back(sample.position + Eigen::Vector3d(0.0004, 0.0002, 0.0008));
            snapshot.normals.push_back(Eigen::Vector3d(0.2, 0.1, 1.0).normalized());
        }
        return snapshot;
    }

    Eigen::Isometry3d worldTransform()
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() << 0.0, -1.0, 0.0,
                              1.0,  0.0, 0.0,
                              0.0,  0.0, 1.0;
        transform.translation() = Eigen::Vector3d(0.03, -0.04, 0.05);
        return transform;
    }

    void checkReport(const DynamicSurfaceView& report,
        const spraythickness::DynamicSurfaceSnapshot& snapshot,
        const Eigen::Isometry3d& transform)
    {
        require(report.model && report.model->subMeshCount() == 1,
            "Dynamic reports must expose exactly one original substrate mesh.");
        require(report.overlay.objectId == "dynamic-report-fixture"
            && report.overlay.quantityName == "Thickness" && report.overlay.unit == "m",
            "Dynamic report metadata must identify actual thickness in meters.");
        require(report.overlay.subMeshes.size() == 1
            && report.overlay.subMeshes[0].values == snapshot.thicknessMeters,
            "Dynamic reports must retain the exact accumulated thickness field.");
        const auto& geometry = report.model->subMesh(0).geometry;
        require(geometry.positions.size() == snapshot.substrate->samples.size()
            && geometry.indices == snapshot.substrate->triangleIndices,
            "Dynamic reports must preserve substrate topology and vertex count.");
        require(report.binding.sampleIndicesBySubMesh.size() == 1
            && report.binding.sampleIndicesBySubMesh[0].size() == geometry.positions.size(),
            "Dynamic reports need an independent substrate-to-field binding.");
        for(std::size_t i = 0; i < geometry.positions.size(); ++i) {
            const Eigen::Vector3d world = transform * geometry.positions[i].cast<double>();
            require((world - snapshot.substrate->samples[i].position).norm() < 3.0e-9,
                "Dynamic report positions must be the original world substrate, without growth added twice.");
            require((world - snapshot.positions[i]).norm() > 0.0005,
                "An evolved surface must not become the report substrate.");
            require(report.binding.sampleIndicesBySubMesh[0][i] == i,
                "Dynamic report binding changed the thickness vertex identity.");
        }
        auto samples = sampleYzThickness(*report.model, report.overlay, transform);
        require(samples.size() >= 3, "Dynamic report substrate must intersect the world YZ plane.");
        std::sort(samples.begin(), samples.end(), [](const auto& first, const auto& second) {
            return first.positionMeters.y() < second.positionMeters.y();
        });
        near(samples.front().positionMeters.y(), 0.0105, 3.0e-9,
            "Transformed dynamic report starts at the wrong substrate Y position.");
        near(samples.front().positionMeters.z(), 0.020, 3.0e-9,
            "Transformed dynamic report starts on the grown surface instead of the substrate.");
        near(samples.back().positionMeters.y(), 0.012, 3.0e-9,
            "Transformed dynamic report ends at the wrong substrate Y position.");
        near(samples.back().positionMeters.z(), 0.0215, 3.0e-9,
            "Transformed dynamic report ends at the wrong substrate Z position.");
        require(samples.front().thicknessMeters == 0.0,
            "A valid zero-thickness substrate interval must remain exactly zero.");
        for(const auto& sample : samples) {
            require(sample.positionMeters.x() == 0.0,
                "Dynamic report samples must stay exactly on the world X=0 plane.");
            near(sample.thicknessMeters, 0.05 * (sample.positionMeters.z() - 0.020),
                2.0e-10, "Dynamic report interpolation must use true thickness rather than displacement.");
        }
        const auto segments = robot_qt_viewer::extractYzThicknessSegments(
            *report.model, report.overlay, transform);
        require(segments.size() == 1,
            "A dynamic report must cut only the original substrate, without an extra evolved shell.");
    }
}

void runDynamicSurfaceReportTests()
{
    const auto snapshot = snapshotFixture();
    const auto transform = worldTransform();
    // Verify the adapter honors saved region labels and the prediction frame.
    smrobot::spray::rotationbody::PublishedTrajectoryPlan plan;
    plan.section.emplace();
    plan.section->pointsYz = {{.01, 0.0}, {.02, 0.0}, {.02, .01}, {.01, .01}};
    plan.regions.emplace();
    using Label = smrobot::spray::rotationbody::RegionLabel;
    plan.regions->segmentLabels = {Label::ToothWall, Label::ToothTop, Label::Transition};
    spraythickness::LocalDepositionCorrectionOptions local;
    local.geometry = robot_qt_viewer::makePublishedToothTopCorrectionGeometry(plan, transform.inverse());
    near(spraythickness::evaluateLocalDepositionRetention(local, transform * Eigen::Vector3d(0, .02, .005)),
        .45, 1.0e-12, "Published tooth center must use the independent correction in the prediction frame.");
    near(spraythickness::evaluateLocalDepositionRetention(local, transform * Eigen::Vector3d(.02, 0, .005)),
        .45, 1.0e-12, "Full revolving surface must reuse its radial tooth section.");
    near(spraythickness::evaluateLocalDepositionRetention(local, transform * Eigen::Vector3d(0, .015, .01)),
        1.0, 1.0e-12, "A saved Transition region must never inherit the tooth-top correction.");
    auto relabeled = plan;
    relabeled.regions->segmentLabels[1] = Label::ToothWall;
    auto unchanged = local;
    unchanged.geometry = robot_qt_viewer::makePublishedToothTopCorrectionGeometry(relabeled, transform.inverse());
    near(spraythickness::evaluateLocalDepositionRetention(unchanged, transform * Eigen::Vector3d(0, .02, .005)),
        1.0, 1.0e-12, "Manual saved labels must determine whether a point is corrected.");
    {
        QTemporaryDir folder;
        require(folder.isValid(), "Temporary dynamic-report output is unavailable.");
        robot_qt_viewer::CoatingAnalysisSession session;
        auto saved = std::make_shared<spraythickness::DynamicSurfaceSnapshot>(snapshot);
        saved->historySeconds.assign(3, 0.0f);
        saved->lastUpdateSeconds.assign(3, 0.0f);
        saved->historyFactors.assign(3, 1.0f);
        session.prediction.dynamicSurface = saved;
        session.dynamicResultOptions.localCorrection = local;
        session.dynamicResultOptions.localCorrection.centerRetention = .41;
        QString error;
        const QString path = folder.filePath(QStringLiteral("corrected.dsurf"));
        require(robot_qt_viewer::saveDynamicSurface(path, session, false, &error),
            "Dynamic export with correction metadata failed.");
        QFile file(path);
        require(file.open(QIODevice::ReadOnly), "Dynamic correction export could not be reopened.");
        QDataStream stream(&file);
        stream.setVersion(QDataStream::Qt_5_15);
        stream.setByteOrder(QDataStream::LittleEndian);
        char magic[8];
        require(stream.readRawData(magic, 8) == 8, "Dynamic export signature missing.");
        QByteArray bytes;
        stream >> bytes;
        const auto metadata = QJsonDocument::fromJson(bytes).object();
        const auto correction = metadata.value(QStringLiteral("localDepositionCorrection")).toObject();
        require(correction.value(QStringLiteral("version")).toInt() == 2
            && correction.value(QStringLiteral("formula")).toString()
                == QStringLiteral("edge+(center-edge)*(1-x*x)^exponent")
            && !correction.contains(QStringLiteral("edgeBlendFraction")),
            "Dynamic export must identify the monotone model without the obsolete edge recovery setting.");
        require(correction.value(QStringLiteral("enabled")).toBool()
            && correction.value(QStringLiteral("applied")).toBool(),
            "Dynamic export must identify the correction actually applied to the result.");
        near(correction.value(QStringLiteral("centerRetention")).toDouble(), .41, 1.0e-12,
            "Dynamic export must store result parameters instead of UI defaults.");
        require(correction.value(QStringLiteral("geometry")).toObject()
            .value(QStringLiteral("toothTopSegments")).toArray().size() == 3,
            "Dynamic export must preserve the region mapping needed to audit its correction.");
    }
    const auto baseline = makeDynamicSurfaceReportView(
        QStringLiteral("dynamic-report-fixture"), snapshot, transform);
    checkReport(baseline, snapshot, transform);
    for(int view = 0; view < 3; ++view) {
        for(int coloring = 0; coloring < 3; ++coloring) {
            robot_qt_viewer::DynamicSurfaceDisplay display;
            display.view = view;
            display.coloring = coloring;
            const auto visual = robot_qt_viewer::makeDynamicSurfaceView(
                QStringLiteral("dynamic-report-fixture"), snapshot, transform, display);
            require(visual.model && visual.model->subMeshCount() == (view == 2 ? 2 : 1),
                "The dynamic display fixture must exercise actual evolved, substrate and overlay modes.");
            if(coloring == 1 && view != 1) {
                near(visual.overlay.subMeshes[0].values[0],
                    (snapshot.positions[0] - snapshot.substrate->samples[0].position).norm(),
                    1.0e-14, "The displacement display fixture must differ from the true thickness field.");
                require(visual.overlay.subMeshes[0].values[0] > 0.0005,
                    "The display fixture must exercise nonzero displacement at a true zero-thickness point.");
            }
            const auto report = makeDynamicSurfaceReportView(
                QStringLiteral("dynamic-report-fixture"), snapshot, transform);
            checkReport(report, snapshot, transform);
            require(report.overlay.subMeshes[0].values == baseline.overlay.subMeshes[0].values,
                "Display view or coloring must not change the report field.");
        }
    }
    auto invalid = snapshot;
    invalid.substrate.reset();
    rejects([&] { makeDynamicSurfaceReportView(QStringLiteral("invalid"), invalid, transform); },
        "A missing dynamic substrate must be rejected explicitly.");
    invalid = snapshot;
    invalid.thicknessMeters.pop_back();
    rejects([&] { makeDynamicSurfaceReportView(QStringLiteral("invalid"), invalid, transform); },
        "A dynamic substrate/true-thickness count mismatch must be rejected explicitly.");
}
