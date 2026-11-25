#include <ThicknessPickSectionDialog.h>
#include <ThicknessPickSampling.h>
#include <AssetCore/AssetManager.h>
#include <AssetCore/ModelDesc.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <QApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QVariant>
#include <QWidget>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
    void requireDialog(bool value, const char* message)
    {
        if(!value) throw std::runtime_error(message);
    }

    void clickPoint(QWidget* canvas, double y, double z)
    {
        const QRectF plot = canvas->property("sectionPlotRect").toRectF();
        const double scale = canvas->property("sectionPixelsPerMeter").toDouble();
        requireDialog(scale > 0.0 && !plot.isEmpty(), "The section preview has no usable projection.");
        const QPointF point = plot.center() + QPointF(
            (y - canvas->property("sectionCenterY").toDouble()) * scale,
            -(z - canvas->property("sectionCenterZ").toDouble()) * scale);
        QMouseEvent press(QEvent::MouseButtonPress, point, Qt::LeftButton,
            Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(canvas, &press);
        QMouseEvent release(QEvent::MouseButtonRelease, point, Qt::LeftButton,
            Qt::NoButton, Qt::NoModifier);
        QApplication::sendEvent(canvas, &release);
    }
}

void runSectionDialogTests(const QString& outputDirectory)
{
    using namespace robot_qt_viewer;
    std::vector<ThicknessPickSectionSegment> segments;
    const auto append = [&](double y0, double z0, double y1, double z1,
                            double t0, double t1) {
        ThicknessPickSectionSegment segment;
        segment.first.positionMeters = Eigen::Vector3d(0.0, y0, z0);
        segment.second.positionMeters = Eigen::Vector3d(0.0, y1, z1);
        segment.first.triangleIndex = segment.second.triangleIndex = segments.size();
        segment.first.thicknessMeters = t0;
        segment.second.thicknessMeters = t1;
        segment.first.normalWorld = segment.second.normalWorld =
            Eigen::Vector3d(0.0, z1 - z0, y0 - y1).normalized();
        segments.push_back(segment);
    };
    append(0.030, 0.0, 0.030, 0.035, 0.0, 0.0);
    append(0.030, 0.035, 0.045, 0.050, 0.0, 180e-6);
    append(0.045, 0.050, 0.030, 0.065, 180e-6, 120e-6);
    append(0.030, 0.065, 0.030, 0.100, 120e-6, 100e-6);
    append(-0.030, 0.0, -0.030, 0.100, 80e-6, 80e-6);
    ThicknessPickSectionDialog dialog(segments);
    dialog.setLanguageCode(QStringLiteral("zh-CN"));
    dialog.show();
    QApplication::processEvents();
    dialog.grab();
    auto* canvas = dialog.findChild<QWidget*>(QStringLiteral("thicknessSectionCanvas"));
    auto* spacing = dialog.findChild<QDoubleSpinBox*>(QStringLiteral("thicknessSectionSpacing"));
    auto* mode = dialog.findChild<QComboBox*>(QStringLiteral("thicknessSectionMultiplierMode"));
    auto* multiplier = dialog.findChild<QDoubleSpinBox*>(QStringLiteral("thicknessSectionMultiplier"));
    requireDialog(mode && multiplier && mode->currentIndex() == 0 && !multiplier->isEnabled(),
        "Thickness display must default to automatic mode.");
    mode->setCurrentIndex(1);
    requireDialog(multiplier->isEnabled() && dialog.schematicOptions().fixedThicknessMultiplier == 25.0
        && !dialog.schematicOptions().automaticThicknessMultiplier,
        "Fixed mode must enable a configurable 25 x thickness multiplier.");
    multiplier->setValue(1.0);
    requireDialog(dialog.schematicOptions().fixedThicknessMultiplier == 1.0,
        "The dialog must support an exact 1 x multiplier.");
    multiplier->setValue(25.0);
    mode->setCurrentIndex(0);
    auto* accept = dialog.findChild<QPushButton*>(QStringLiteral("thicknessSectionExport"));
    auto* clear = dialog.findChild<QPushButton*>(QStringLiteral("thicknessSectionClear"));
    auto* finish = dialog.findChild<QPushButton*>(QStringLiteral("thicknessSectionFinish"));
    requireDialog(canvas && spacing && accept && clear && finish, "The section export controls are missing.");
    requireDialog(!accept->isEnabled(), "Export must require an explicit target contour selection.");
    requireDialog(std::abs(dialog.annotationSpacingMeters() - 0.010) < 1e-12,
        "The default annotation spacing must be 10 mm.");
    clickPoint(canvas, 0.020, -0.005);
    clickPoint(canvas, 0.055, -0.005);
    clickPoint(canvas, 0.055, 0.105);
    clickPoint(canvas, 0.020, 0.105);
    finish->click();
    QApplication::processEvents();
    requireDialog(accept->isEnabled(), "A closed polygon enclosing the target contour must enable export.");
    spacing->setValue(5.0);
    requireDialog(std::abs(dialog.annotationSpacingMeters() - 0.005) < 1e-12,
        "The millimeter annotation control must convert to meters.");
    const auto section = buildSelectedYzThicknessSection(
        segments, dialog.selectionPolygon(), dialog.annotationSpacingMeters());
    requireDialog(section.profiles.size() == 1 && !section.annotations.empty(),
        "The preview selection must retain exactly the selected front profile.");
    bool hasZero = false;
    for(const auto& point : section.profiles.front()) {
        requireDialog(point.positionMeters.y() > 0.0, "A back-face point leaked through the preview selection.");
        hasZero |= point.thicknessMeters == 0.0;
    }
    requireDialog(hasZero, "The selected surface's uncoated span must remain in the report.");
    if(!outputDirectory.isEmpty()) {
        requireDialog(QDir().mkpath(outputDirectory), "Cannot create dialog screenshot directory.");
        requireDialog(dialog.grab().save(QDir(outputDirectory).filePath(QStringLiteral("selection-preview.png"))),
            "Cannot save the section selection preview.");
    }
    clear->click();
    requireDialog(!accept->isEnabled() && dialog.selectionPolygon().empty(),
        "Clearing a selection must disable export and remove the old polygon.");
    dialog.accept();
    requireDialog(dialog.result() != QDialog::Accepted,
        "Accept without a selected surface must not export the whole model.");
    clickPoint(canvas, -0.015, 0.020);
    clickPoint(canvas, 0.005, 0.020);
    clickPoint(canvas, 0.005, 0.040);
    clickPoint(canvas, -0.015, 0.040);
    finish->click();
    requireDialog(!accept->isEnabled(), "An empty closed region must not enable export.");
    dialog.reject();

    // A sharp concave turn cannot be drawn at 25 x. Reject in the still-open
    // dialog so the user can fix the multiplier without losing the selection.
    segments.clear();
    append(0.0, 0.0, 0.0, 0.020, 150e-6, 150e-6);
    append(0.0, 0.020, 0.020, 0.020, 150e-6, 150e-6);
    ThicknessPickSectionDialog unsafe(segments);
    unsafe.setLanguageCode(QStringLiteral("en"));
    unsafe.show();
    QApplication::processEvents();
    unsafe.grab();
    auto* unsafeCanvas = unsafe.findChild<QWidget*>(QStringLiteral("thicknessSectionCanvas"));
    auto* unsafeMode = unsafe.findChild<QComboBox*>(QStringLiteral("thicknessSectionMultiplierMode"));
    auto* unsafeFinish = unsafe.findChild<QPushButton*>(QStringLiteral("thicknessSectionFinish"));
    auto* status = unsafe.findChild<QLabel*>(QStringLiteral("thicknessSectionStatus"));
    requireDialog(unsafeCanvas && unsafeMode && unsafeFinish && status,
        "Fixed multiplier validation controls are missing.");
    clickPoint(unsafeCanvas, -0.001, -0.001);
    clickPoint(unsafeCanvas, 0.021, -0.001);
    clickPoint(unsafeCanvas, 0.021, 0.021);
    clickPoint(unsafeCanvas, -0.001, 0.021);
    unsafeFinish->click();
    const auto retained = unsafe.selectionPolygon();
    requireDialog(!retained.empty(), "Unsafe multiplier fixture has no selected contour.");
    unsafeMode->setCurrentIndex(1);
    unsafe.accept();
    requireDialog(unsafe.result() != QDialog::Accepted && unsafe.selectionPolygon().size() == retained.size(),
        "Unsafe fixed multiplier must keep the dialog open and retain its selection.");
    requireDialog(status->text().contains(QStringLiteral("25"))
        && status->text().contains(QStringLiteral("Automatic"))
        && status->text().contains(QStringLiteral("smaller")),
        "The English fixed multiplier failure must explain how to correct it.");
    unsafe.setLanguageCode(QStringLiteral("zh-CN"));
    unsafe.accept();
    requireDialog(status->text().contains(QStringLiteral("\u81ea\u52a8"))
        && status->text().contains(QStringLiteral("\u8f83\u5c0f"))
        && status->text().contains(QStringLiteral("\u9009\u533a\u5df2\u4fdd\u7559")),
        "The Chinese fixed multiplier failure must explain how to correct it.");
    if(!outputDirectory.isEmpty()) {
        requireDialog(unsafe.grab().save(QDir(outputDirectory).filePath(QStringLiteral("fixed-multiplier-rejected.png"))),
            "Cannot save the fixed multiplier validation preview.");
    }
    unsafeMode->setCurrentIndex(0);
    unsafe.accept();
    requireDialog(unsafe.result() == QDialog::Accepted && !unsafe.selectedSection().profiles.empty()
        && unsafe.selectedSection().schematicOptions.automaticThicknessMultiplier,
        "Switching back to automatic must reuse the retained section and export options.");
}


void runRealSectionMeshTest()
{
    using namespace robot_qt_viewer;
    const QByteArray path = qgetenv("THICKNESS_PICK_STL");
    if(path.isEmpty()) return;
    std::string error;
    const auto model = assetcore::AssetManager::instance().tryLoadModel(
        path.toStdString(), 1.0f, &error);
    requireDialog(static_cast<bool>(model), "Cannot load the supplied real section mesh fixture.");
    Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d maximum = -minimum;
    smrobot::visualization::SurfaceScalarOverlay field;
    field.unit = "m";
    field.subMeshes.resize(model->subMeshCount());
    for(std::size_t mesh = 0; mesh < model->subMeshCount(); ++mesh) {
        const auto& positions = model->subMesh(mesh).geometry.positions;
        field.subMeshes[mesh].values.assign(positions.size(), 0.0);
        for(const auto& position : positions) {
            minimum = minimum.cwiseMin(position.cast<double>());
            maximum = maximum.cwiseMax(position.cast<double>());
        }
    }
    Eigen::Isometry3d world = Eigen::Isometry3d::Identity();
    world.translation().x() = -0.5 * (minimum.x() + maximum.x());
    const auto segments = extractYzThicknessSegments(*model, field, world);
    const double lowY = minimum.y() + 0.65 * (maximum.y() - minimum.y());
    const double highY = maximum.y() + 0.01;
    const double lowZ = minimum.z() - 0.01;
    const double highZ = maximum.z() + 0.01;
    const std::vector<Eigen::Vector2d> polygon{
        { lowY, lowZ }, { highY, lowZ }, { highY, highZ }, { lowY, highZ } };
    const auto section = buildSelectedYzThicknessSection(segments, polygon, 0.01);
    requireDialog(!section.profiles.empty(), "The real model's selected side produced no section.");
    std::size_t count = 0;
    for(const auto& profile : section.profiles) {
        for(const auto& point : profile) {
            requireDialog(point.positionMeters.x() == 0.0 && point.positionMeters.y() >= lowY - 1e-10,
                "The real model report includes another side of the section.");
            requireDialog(point.thicknessMeters == 0.0,
                "The all-zero test field must preserve its physical zero values.");
            ++count;
        }
    }
    std::cout << "Real STL selected section: " << segments.size() << " segments; "
        << section.profiles.size() << " profiles; " << count << " samples; "
        << section.annotations.size() << " annotations.\n";
}
