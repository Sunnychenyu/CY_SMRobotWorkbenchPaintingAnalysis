#include "ThicknessPickReport.h"
#include "ThicknessSectionSchematic.h"
#include "../../../include/ExportFileNaming.h"

#include <QBuffer>
#include <QByteArray>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr std::size_t kMaximumSamples = 200000;
        constexpr int kHeaderRow = 8;

        [[noreturn]] void fail(const QString& message)
        {
            throw std::runtime_error(message.toUtf8().constData());
        }

        QString xmlText(const QString& text)
        {
            if(text.size() > 32767) fail(QStringLiteral("Thickness pick report: cell text exceeds Excel limits."));
            QString valid;
            valid.reserve(text.size());
            for(int index = 0; index < text.size(); ++index) {
                const QChar character = text[index];
                const ushort value = character.unicode();
                if(character.isHighSurrogate()) {
                    if(index + 1 < text.size() && text[index + 1].isLowSurrogate()) {
                        valid += character;
                        valid += text[++index];
                    }
                    else valid += QChar(0xfffd);
                }
                else if(character.isLowSurrogate() || value == 0xfffe || value == 0xffff ||
                    (value < 0x20 && value != 0x09 && value != 0x0a && value != 0x0d)) {
                    valid += QChar(0xfffd);
                }
                else valid += character;
            }
            return valid.toHtmlEscaped();
        }

        QString textCell(const QString& reference, const QString& value, int style = 0)
        {
            return QStringLiteral("<c r=\"%1\" s=\"%2\" t=\"inlineStr\"><is><t xml:space=\"preserve\">%3</t></is></c>")
                .arg(reference).arg(style).arg(xmlText(value));
        }

        QString numericCell(const QString& reference, double value, int style = 2)
        {
            if(!std::isfinite(value)) fail(QStringLiteral("Thickness pick report: converted numeric value is not finite."));
            return QStringLiteral("<c r=\"%1\" s=\"%2\"><v>%3</v></c>")
                .arg(reference).arg(style).arg(QString::number(value, 'g', 17));
        }

        QString integerCell(const QString& reference, std::size_t value)
        {
            return QStringLiteral("<c r=\"%1\"><v>%2</v></c>")
                .arg(reference).arg(static_cast<qulonglong>(value));
        }

        QByteArray makeWorksheet(const std::vector<ThicknessPickReportSample>& samples,
            const QString& modelName, const QDateTime& exportTime, bool selectedSection)
        {
            const int lastRow = kHeaderRow + static_cast<int>(samples.size());
            QString xml = QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
                "<dimension ref=\"A1:F%1\"/><sheetViews><sheetView workbookViewId=\"0\">"
                "<pane ySplit=\"8\" topLeftCell=\"A9\" activePane=\"bottomLeft\" state=\"frozen\"/>"
                "<selection pane=\"bottomLeft\" activeCell=\"A9\" sqref=\"A9\"/></sheetView></sheetViews>"
                "<sheetFormatPr defaultRowHeight=\"18\"/><cols>"
                "<col min=\"1\" max=\"1\" width=\"22\" customWidth=\"1\"/>"
                "<col min=\"2\" max=\"2\" width=\"21\" customWidth=\"1\"/>"
                "<col min=\"3\" max=\"5\" width=\"20\" customWidth=\"1\"/>"
                "<col min=\"6\" max=\"6\" width=\"23\" customWidth=\"1\"/></cols><sheetData>").arg(lastRow);
            const auto addMetadata = [&](int row, const QString& label, const QString& value, int height = 24) {
                xml += QStringLiteral("<row r=\"%1\" ht=\"%2\" customHeight=\"1\">").arg(row).arg(height);
                xml += textCell(QStringLiteral("A%1").arg(row), label, 4);
                xml += textCell(QStringLiteral("B%1").arg(row), value, 4);
                xml += QStringLiteral("</row>");
            };
            xml += QStringLiteral("<row r=\"1\" ht=\"30\" customHeight=\"1\">") +
                textCell(QStringLiteral("A1"), QStringLiteral("\u6d82\u5c42\u5206\u6790 - \u62fe\u53d6\u539a\u5ea6\u62a5\u544a"), 3) + QStringLiteral("</row>");
            addMetadata(2, QStringLiteral("\u6a21\u578b"), modelName);
            addMetadata(3, QStringLiteral("\u5bfc\u51fa\u65f6\u95f4 (UTC)"), exportTime.toString(Qt::ISODateWithMs));
            addMetadata(4, QStringLiteral("\u62fe\u53d6\u5e73\u9762"), QStringLiteral("\u4e16\u754c YZ \u5e73\u9762 (x = 0)"));
            addMetadata(5, QStringLiteral("\u6570\u636e\u6765\u6e90"), QStringLiteral("\u5f53\u524d\u6d82\u5c42\u5206\u6790\u539a\u5ea6\u573a\uff08\u4e09\u89d2\u5f62\u63d2\u503c / \u5355\u5143\u503c\uff09"));
            xml += QStringLiteral("<row r=\"6\" ht=\"36\" customHeight=\"1\">") +
                textCell(QStringLiteral("A6"), QStringLiteral("\u70b9\u6570"), 4) + integerCell(QStringLiteral("B6"), samples.size()) +
                textCell(QStringLiteral("C6"), QStringLiteral("\u5355\u4f4d\u4e0e\u7f16\u53f7"), 4) +
                textCell(QStringLiteral("D6"), QStringLiteral("\u5750\u6807\uff1amm\uff1b\u539a\u5ea6\uff1a\u03bcm\uff1b\u4e09\u89d2\u5f62\u7f16\u53f7\u4ece 0 \u5f00\u59cb"), 4) + QStringLiteral("</row>");
            if(selectedSection) {
                addMetadata(7, QStringLiteral("\u5bfc\u51fa\u8303\u56f4"), QStringLiteral("\u4ec5\u6846\u9009\u8868\u9762\u7684 YZ \u622a\u9762\uff0c\u4fdd\u7559\u8be5\u8868\u9762\u96f6\u539a\u5ea6\u70b9\uff1b\u53e6\u89c1\u622a\u9762\u793a\u610f\u56fe\u4e0e\u7b49\u8ddd\u6807\u6ce8\u5de5\u4f5c\u8868\u3002"));
            }
            xml += QStringLiteral("<row r=\"8\" ht=\"27\" customHeight=\"1\">") +
                textCell(QStringLiteral("A8"), QStringLiteral("\u5e8f\u53f7"), 1) +
                textCell(QStringLiteral("B8"), QStringLiteral("\u4e09\u89d2\u5f62\u7f16\u53f7"), 1) +
                textCell(QStringLiteral("C8"), QStringLiteral("X (mm)"), 1) +
                textCell(QStringLiteral("D8"), QStringLiteral("Y (mm)"), 1) +
                textCell(QStringLiteral("E8"), QStringLiteral("Z (mm)"), 1) +
                textCell(QStringLiteral("F8"), QStringLiteral("\u539a\u5ea6 (\u03bcm)"), 1) + QStringLiteral("</row>");
            for(std::size_t index = 0; index < samples.size(); ++index) {
                const auto& sample = samples[index];
                const QString row = QString::number(kHeaderRow + 1 + static_cast<int>(index));
                xml += QStringLiteral("<row r=\"%1\">").arg(row) +
                    integerCell(QStringLiteral("A") + row, index + 1) +
                    integerCell(QStringLiteral("B") + row, sample.triangleIndex) +
                    numericCell(QStringLiteral("C") + row, 0.0, selectedSection ? 5 : 2) +
                    numericCell(QStringLiteral("D") + row, sample.positionMeters.y() * 1000.0, selectedSection ? 5 : 2) +
                    numericCell(QStringLiteral("E") + row, sample.positionMeters.z() * 1000.0, selectedSection ? 5 : 2) +
                    numericCell(QStringLiteral("F") + row, sample.thicknessMeters * 1e6, selectedSection ? 6 : 2) + QStringLiteral("</row>");
            }
            xml += QStringLiteral("</sheetData><autoFilter ref=\"A8:F%1\"/>"
                "<mergeCells count=\"%2\"><mergeCell ref=\"A1:F1\"/><mergeCell ref=\"B2:F2\"/>"
                "<mergeCell ref=\"B3:F3\"/><mergeCell ref=\"B4:F4\"/><mergeCell ref=\"B5:F5\"/>"
                "<mergeCell ref=\"D6:F6\"/>%3</mergeCells>"
                "<pageMargins left=\"0.3\" right=\"0.3\" top=\"0.5\" bottom=\"0.5\" header=\"0.2\" footer=\"0.2\"/>"
                "</worksheet>").arg(lastRow).arg(selectedSection ? 7 : 6)
                .arg(selectedSection ? QStringLiteral("<mergeCell ref=\"B7:F7\"/>") : QString());
            return xml.toUtf8();
        }

        struct SchematicPanel
        {
            QByteArray png;
            QString description;
        };

        constexpr int kImageWidth = 1440;
        constexpr int kImageHeight = 1500;
        constexpr int kImageRows = 53;

        QPointF yzPoint(const ThicknessPickReportSample& sample)
        {
            return QPointF(sample.positionMeters.y(), sample.positionMeters.z());
        }

        QPointF yzPoint(const Eigen::Vector2d& point)
        {
            return QPointF(point.x(), point.y());
        }

        void validateSection(const ThicknessPickReportSection& section)
        {
            if(section.profiles.empty() || section.annotations.empty() ||
                !std::isfinite(section.annotationSpacingMeters) || section.annotationSpacingMeters <= 0.0 ||
                !std::isfinite(section.annotationSpacingMeters * 1000.0)) {
                fail(QStringLiteral("Thickness pick report: section requires profiles, annotations and a positive finite spacing."));
            }
            const auto validate = [&](const ThicknessPickReportSample& sample, std::size_t profileIndex) {
                if(!sample.positionMeters.allFinite() || sample.positionMeters.x() != 0.0 ||
                    !sample.normalWorld.allFinite() ||
                    !std::isfinite(std::hypot(sample.normalWorld.y(), sample.normalWorld.z())) ||
                    std::hypot(sample.normalWorld.y(), sample.normalWorld.z()) < 1e-12 ||
                    !std::isfinite(sample.thicknessMeters) || sample.thicknessMeters < 0.0 ||
                    !std::isfinite(sample.thicknessMeters * 1e6) ||
                    !std::isfinite(sample.positionMeters.y() * 1000.0) ||
                    !std::isfinite(sample.positionMeters.z() * 1000.0) ||
                    !std::isfinite(sample.arcLengthMeters) || sample.arcLengthMeters < 0.0 ||
                    !std::isfinite(sample.arcLengthMeters * 1000.0) || sample.profileIndex != profileIndex ||
                    sample.triangleIndex > 9007199254740991ULL) {
                    fail(QStringLiteral("Thickness pick report: invalid section position, normal, thickness or arc length."));
                }
            };
            std::size_t pointCount = 0;
            for(std::size_t index = 0; index < section.profiles.size(); ++index) {
                const auto& profile = section.profiles[index];
                if(profile.size() < 2) fail(QStringLiteral("Thickness pick report: a section profile needs at least two points."));
                double previous = -1.0;
                for(const auto& sample : profile) {
                    validate(sample, index);
                    if(sample.arcLengthMeters < previous) fail(QStringLiteral("Thickness pick report: section profiles must be ordered by arc length."));
                    previous = sample.arcLengthMeters;
                }
                pointCount += profile.size();
                if(pointCount > kMaximumSamples) fail(QStringLiteral("Thickness pick report: section point count exceeds 200000."));
            }
            if(section.annotations.size() > kMaximumSamples) fail(QStringLiteral("Thickness pick report: too many annotations."));
            for(const auto& sample : section.annotations) {
                if(sample.profileIndex >= section.profiles.size()) fail(QStringLiteral("Thickness pick report: invalid annotation profile index."));
                validate(sample, sample.profileIndex);
                const auto& profile = section.profiles[sample.profileIndex];
                if(sample.arcLengthMeters < profile.front().arcLengthMeters - 1e-10 ||
                    sample.arcLengthMeters > profile.back().arcLengthMeters + 1e-10) {
                    fail(QStringLiteral("Thickness pick report: annotation lies outside its profile."));
                }
            }
        }

        SchematicPanel renderPanel(const std::vector<ThicknessPickReportSample>& points,
            const std::vector<std::pair<std::size_t, ThicknessPickReportSample>>& annotations,
            const ThicknessSectionSchematicGeometry& geometry,
            std::size_t profileIndex, std::size_t panelIndex, double spacing, bool overview = false)
        {
            double minY = std::numeric_limits<double>::infinity();
            double maxY = -minY;
            double minZ = minY;
            double maxZ = -minY;
            const auto include = [&](const QPointF& point) {
                minY = std::min(minY, point.x()); maxY = std::max(maxY, point.x());
                minZ = std::min(minZ, point.y()); maxZ = std::max(maxZ, point.y());
            };
            for(const auto& sample : points) include(yzPoint(sample));
            const double baseSpan = std::max({ maxY - minY, maxZ - minZ, 1e-9 });
            const double exaggeration = geometry.thicknessMultiplier;
            for(const auto& point : geometry.points) include(yzPoint(point.coatingYZ));
            for(const auto& point : geometry.annotations) include(yzPoint(point.coatingYZ));
            const double scale = std::min(850.0 / std::max(maxY - minY, baseSpan * 0.05),
                1080.0 / std::max(maxZ - minZ, baseSpan * 0.05));
            const QPointF center((minY + maxY) * 0.5, (minZ + maxZ) * 0.5);
            const auto map = [&](const QPointF& point) {
                return QPointF(535.0 + (point.x() - center.x()) * scale,
                    755.0 - (point.y() - center.y()) * scale);
            };
            QImage image(kImageWidth, kImageHeight, QImage::Format_ARGB32_Premultiplied);
            if(image.isNull()) fail(QStringLiteral("Thickness pick report: cannot allocate schematic image."));
            image.fill(QColor(248, 250, 253));
            QPainter painter(&image);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setRenderHint(QPainter::TextAntialiasing);
            QFont font(QStringLiteral("Microsoft YaHei"));
            font.setPixelSize(26);
            font.setBold(true);
            painter.setFont(font);
            painter.setPen(QColor(29, 49, 72));
            painter.drawText(QRectF(40, 25, 1360, 45), Qt::AlignLeft | Qt::AlignVCenter,
                overview ? QStringLiteral("YZ \u622a\u9762\u539a\u5ea6\u793a\u610f\u56fe  |  \u8f6e\u5ed3 %1 \u603b\u89c8\uff08\u6807\u6ce8\u89c1\u540e\u7eed\u5206\u56fe\uff09")
                    .arg(static_cast<qulonglong>(profileIndex + 1))
                    : QStringLiteral("YZ \u622a\u9762\u539a\u5ea6\u793a\u610f\u56fe  |  \u8f6e\u5ed3 %1 / \u5206\u56fe %2")
                        .arg(static_cast<qulonglong>(profileIndex + 1)).arg(static_cast<qulonglong>(panelIndex + 1)));
            font.setPixelSize(22); font.setBold(false); painter.setFont(font);
            painter.drawText(QRectF(40, 80, 1360, 38), Qt::AlignLeft | Qt::AlignVCenter,
                QStringLiteral("\u7b49\u5f27\u957f\u6807\u6ce8\u95f4\u8ddd %1 mm  |  \u539a\u5ea6\u663e\u793a\u500d\u7387 %2  |  \u6807\u6ce8\u4e3a\u539f\u59cb\u539a\u5ea6 (\u03bcm)")
                    .arg(spacing * 1000.0, 0, 'g', 6).arg(exaggeration, 0, 'g', 4));
            const QColor baseColor(0, 145, 175);
            const QColor coatingColor(203, 56, 157);
            const QColor thicknessColor(169, 117, 0);
            const auto legend = [&](int x, const QColor& color, const QString& label) {
                painter.setPen(QPen(color, 4)); painter.drawLine(x, 145, x + 55, 145);
                painter.setPen(QColor(55, 65, 78)); painter.drawText(QRectF(x + 68, 125, 350, 40), Qt::AlignVCenter, label);
            };
            legend(45, baseColor, QStringLiteral("\u57fa\u4f53\u622a\u9762\u7ebf"));
            legend(430, coatingColor, QStringLiteral("\u6d82\u5c42\u5916\u8f6e\u5ed3\uff08\u793a\u610f\uff09"));
            legend(940, thicknessColor, QStringLiteral("\u6cd5\u5411\u539a\u5ea6"));
            painter.setPen(QPen(QColor(216, 224, 233), 1));
            painter.drawRoundedRect(QRectF(35, 185, 1370, 1185), 10, 10);

            QPainterPath basePath, coatingPath, bandPath;
            for(std::size_t index = 0; index < points.size(); ++index) {
                const QPointF base = map(yzPoint(points[index]));
                if(index == 0) basePath.moveTo(base);
                else basePath.lineTo(base);
            }
            for(std::size_t index = 0; index < geometry.points.size(); ++index) {
                const QPointF base = map(yzPoint(geometry.points[index].baseYZ));
                const QPointF coating = map(yzPoint(geometry.points[index].coatingYZ));
                if(index == 0) { coatingPath.moveTo(coating); bandPath.moveTo(base); }
                else { coatingPath.lineTo(coating); bandPath.lineTo(base); }
            }
            for(auto iterator = geometry.points.rbegin(); iterator != geometry.points.rend(); ++iterator) {
                bandPath.lineTo(map(yzPoint(iterator->coatingYZ)));
            }
            bandPath.closeSubpath();
            painter.fillPath(bandPath, QColor(203, 56, 157, 24));
            painter.setPen(QPen(coatingColor, 3)); painter.drawPath(coatingPath);
            painter.setPen(QPen(baseColor, 3)); painter.drawPath(basePath);

            struct Label { QPointF anchor; QString text; QString distance; };
            std::vector<Label> labels;
            for(std::size_t index = 0; index < annotations.size(); ++index) {
                const auto& item = annotations[index];
                const auto& sample = item.second;
                const auto& marker = geometry.annotations[index];
                const QPointF base = map(yzPoint(sample));
                const QPointF coating = map(yzPoint(marker.coatingYZ));
                painter.setPen(QPen(thicknessColor, 2));
                painter.drawLine(base, coating);
                const QPointF normal = yzPoint(marker.normalYZ);
                const QPointF tangent(normal.y() * 5.0, normal.x() * 5.0);
                painter.drawLine(base - tangent, base + tangent);
                painter.drawLine(coating - tangent, coating + tangent);
                painter.setBrush(thicknessColor); painter.drawEllipse(base, 3, 3); painter.setBrush(Qt::NoBrush);
                labels.push_back({ (base + coating) * 0.5,
                    QStringLiteral("P%1   %2 \u03bcm").arg(static_cast<qulonglong>(item.first + 1), 3, 10, QChar('0'))
                        .arg(sample.thicknessMeters * 1e6, 0, 'f', 1),
                    QStringLiteral("s = %1 mm").arg(sample.arcLengthMeters * 1000.0, 0, 'f', 2) });
            }
            std::stable_sort(labels.begin(), labels.end(), [](const Label& first, const Label& second) { return first.anchor.y() < second.anchor.y(); });
            std::vector<double> labelY;
            double previous = 190.0;
            for(const auto& label : labels) { previous = std::max(previous + 46.0, label.anchor.y()); labelY.push_back(previous); }
            if(!labelY.empty() && labelY.back() > 1315.0) {
                labelY.back() = 1315.0;
                for(std::size_t index = labelY.size() - 1; index > 0; --index) labelY[index - 1] = std::min(labelY[index - 1], labelY[index] - 46.0);
            }
            for(std::size_t index = 0; index < labels.size(); ++index) {
                const auto& label = labels[index];
                painter.setPen(QPen(QColor(135, 145, 158), 1));
                painter.drawLine(label.anchor, QPointF(1025.0, labelY[index]));
                painter.setBrush(QColor(255, 255, 255));
                painter.setPen(QPen(QColor(222, 197, 135), 1));
                painter.drawRoundedRect(QRectF(1030.0, labelY[index] - 19.0, 348.0, 43.0), 4, 4);
                painter.setBrush(Qt::NoBrush);
                font.setPixelSize(21); painter.setFont(font); painter.setPen(thicknessColor);
                painter.drawText(QRectF(1040, labelY[index] - 20, 330, 26), Qt::AlignVCenter, label.text);
                font.setPixelSize(15); painter.setFont(font); painter.setPen(QColor(102, 112, 124));
                painter.drawText(QRectF(1040, labelY[index] + 5, 330, 20), Qt::AlignVCenter, label.distance);
            }
            painter.setPen(QPen(QColor(68, 85, 107), 2));
            painter.drawLine(75, 1330, 150, 1330); painter.drawLine(75, 1330, 75, 1260);
            painter.drawLine(150, 1330, 142, 1326); painter.drawLine(150, 1330, 142, 1334);
            painter.drawLine(75, 1260, 71, 1268); painter.drawLine(75, 1260, 79, 1268);
            font.setPixelSize(18); painter.setFont(font);
            painter.drawText(157, 1336, QStringLiteral("+Y")); painter.drawText(64, 1250, QStringLiteral("+Z"));
            const double scaleLength = std::pow(10.0, std::floor(std::log10(180.0 / scale)));
            const int scaleX = 310;
            const double scalePixels = scaleLength * scale;
            painter.drawLine(QPointF(scaleX, 1330), QPointF(scaleX + scalePixels, 1330));
            painter.drawLine(scaleX, 1325, scaleX, 1335);
            painter.drawLine(QPointF(scaleX + scalePixels, 1325), QPointF(scaleX + scalePixels, 1335));
            painter.drawText(QRectF(scaleX - 20, 1337, 250, 27), QStringLiteral("%1 mm (\u57fa\u4f53)").arg(scaleLength * 1000.0, 0, 'g', 5));
            font.setPixelSize(20); painter.setFont(font); painter.setPen(QColor(68, 85, 107));
            painter.drawText(QRectF(40, 1390, 1360, 76), Qt::TextWordWrap,
                QStringLiteral("\u6d82\u5c42\u8f6e\u5ed3\u7ecf\u5e73\u6ed1\u663e\u793a\uff1b\u6807\u6ce8\u4e0e\u8868\u683c\u4e3a\u539f\u59cb\u539a\u5ea6\u3002\u5e73\u6ed1\u5f27\u957f\u7a97\u53e3 %1 mm\uff1b\u96f6\u539a\u5ea6\u4ecd\u4fdd\u7559\u3002\n\u57fa\u4f53 Y/Z \u7b49\u6bd4\u4f8b\uff1b\u539a\u5ea6\u6309\u6240\u793a\u500d\u7387\u663e\u793a\uff1b\u540c\u4e00\u8f6e\u5ed3\u603b\u89c8\u548c\u5206\u56fe\u5171\u7528\u500d\u7387\uff1b\u672b\u7aef\u8865\u70b9\u95f4\u8ddd\u53ef\u5c0f\u4e8e\u8bbe\u5b9a\u503c\u3002")
                    .arg(geometry.smoothingWindowMeters * 1000.0, 0, 'g', 4));
            painter.end();
            QByteArray png;
            QBuffer buffer(&png);
            if(!buffer.open(QIODevice::WriteOnly) || !image.save(&buffer, "PNG")) fail(QStringLiteral("Thickness pick report: cannot encode schematic PNG."));
            return { std::move(png), QStringLiteral("Profile %1, panel %2, true thickness labels, displayed thickness x%3")
                .arg(static_cast<qulonglong>(profileIndex + 1)).arg(static_cast<qulonglong>(panelIndex + 1)).arg(exaggeration, 0, 'g', 12) };
        }

        std::vector<SchematicPanel> makeSchematicPanels(const ThicknessPickReportSection& section)
        {
            if(!qobject_cast<QGuiApplication*>(QCoreApplication::instance())) {
                fail(QStringLiteral("Thickness pick report: schematic rendering requires a GUI application."));
            }
            if(QFontDatabase().families().isEmpty()) {
                fail(QStringLiteral("Thickness pick report: no fonts are available for thickness labels; use a GUI platform with font support."));
            }
            constexpr std::size_t labelsPerPanel = 24;
            std::vector<SchematicPanel> panels;
            for(std::size_t profileIndex = 0; profileIndex < section.profiles.size(); ++profileIndex) {
                const auto& profile = section.profiles[profileIndex];
                std::vector<std::pair<std::size_t, ThicknessPickReportSample>> annotations;
                for(std::size_t index = 0; index < section.annotations.size(); ++index) {
                    if(section.annotations[index].profileIndex == profileIndex) annotations.emplace_back(index, section.annotations[index]);
                }
                std::stable_sort(annotations.begin(), annotations.end(), [](const auto& first, const auto& second) {
                    return first.second.arcLengthMeters < second.second.arcLengthMeters;
                });
                std::vector<ThicknessPickReportSample> annotationSamples;
                annotationSamples.reserve(annotations.size());
                for(const auto& annotation : annotations) annotationSamples.push_back(annotation.second);
                const auto geometry = buildThicknessSectionSchematic(profile, annotationSamples, section.schematicOptions);
                if(geometry.points.size() < 2 || geometry.annotations.size() != annotations.size()) {
                    fail(QStringLiteral("Thickness pick report: invalid section schematic geometry."));
                }
                const std::size_t panelCount = std::max<std::size_t>(1, (annotations.size() + labelsPerPanel - 1) / labelsPerPanel);
                if(panelCount > 1) panels.push_back(renderPanel(profile, {}, geometry, profileIndex, 0, section.annotationSpacingMeters, true));
                for(std::size_t panelIndex = 0; panelIndex < panelCount; ++panelIndex) {
                    const std::size_t begin = panelIndex * labelsPerPanel;
                    const std::size_t end = std::min(begin + labelsPerPanel, annotations.size());
                    const double lower = begin == 0 ? profile.front().arcLengthMeters
                        : (annotations[begin - 1].second.arcLengthMeters + annotations[begin].second.arcLengthMeters) * 0.5;
                    const double upper = end == annotations.size() ? profile.back().arcLengthMeters
                        : (annotations[end - 1].second.arcLengthMeters + annotations[end].second.arcLengthMeters) * 0.5;
                    auto first = std::lower_bound(profile.begin(), profile.end(), lower,
                        [](const auto& sample, double distance) { return sample.arcLengthMeters < distance; });
                    if(first != profile.begin()) --first;
                    auto last = std::upper_bound(profile.begin(), profile.end(), upper,
                        [](double distance, const auto& sample) { return distance < sample.arcLengthMeters; });
                    if(last != profile.end()) ++last;
                    std::vector<ThicknessPickReportSample> points(first, last);
                    auto firstDrawingPoint = std::lower_bound(geometry.points.begin(), geometry.points.end(), lower,
                        [](const auto& point, double distance) { return point.arcLengthMeters < distance; });
                    if(firstDrawingPoint != geometry.points.begin()) --firstDrawingPoint;
                    auto lastDrawingPoint = std::upper_bound(geometry.points.begin(), geometry.points.end(), upper,
                        [](double distance, const auto& point) { return distance < point.arcLengthMeters; });
                    if(lastDrawingPoint != geometry.points.end()) ++lastDrawingPoint;
                    ThicknessSectionSchematicGeometry panelGeometry;
                    panelGeometry.points.assign(firstDrawingPoint, lastDrawingPoint);
                    panelGeometry.annotations.assign(geometry.annotations.begin() + begin, geometry.annotations.begin() + end);
                    panelGeometry.thicknessMultiplier = geometry.thicknessMultiplier;
                    panelGeometry.smoothingWindowMeters = geometry.smoothingWindowMeters;
                    panels.push_back(renderPanel(points, { annotations.begin() + begin, annotations.begin() + end },
                        panelGeometry, profileIndex, panelIndex, section.annotationSpacingMeters));
                    if(panels.size() > 512) fail(QStringLiteral("Thickness pick report: too many schematic panels; increase annotation spacing."));
                }
            }
            return panels;
        }

        QByteArray makeSchematicWorksheet(std::size_t panelCount)
        {
            return (QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">"
                "<dimension ref=\"A1:O%1\"/><sheetViews><sheetView workbookViewId=\"0\" showGridLines=\"0\" zoomScale=\"80\"/></sheetViews>"
                "<sheetFormatPr defaultRowHeight=\"18\"/><cols><col min=\"1\" max=\"15\" width=\"10\" customWidth=\"1\"/></cols><sheetData>"
                "<row r=\"1\" ht=\"30\" customHeight=\"1\">").arg(static_cast<qulonglong>(panelCount * kImageRows + 4)) +
                textCell(QStringLiteral("A1"), QStringLiteral("\u6240\u9009\u8868\u9762 - YZ \u622a\u9762\u539a\u5ea6\u793a\u610f\u56fe"), 3) +
                QStringLiteral("</row><row r=\"2\" ht=\"30\" customHeight=\"1\">") +
                textCell(QStringLiteral("A2"), QStringLiteral("\u7b49\u8ddd\u6807\u6ce8\u7684\u5750\u6807\u3001\u5f27\u957f\u548c\u539f\u59cb\u539a\u5ea6\u89c1\u201c\u7b49\u8ddd\u6807\u6ce8\u201d\u5de5\u4f5c\u8868\uff1b\u5404\u8f6e\u5ed3\u5206\u5f00\u7ed8\u5236\u3002"), 4) +
                QStringLiteral("</row></sheetData><mergeCells count=\"2\"><mergeCell ref=\"A1:O1\"/><mergeCell ref=\"A2:O2\"/></mergeCells>"
                "<pageMargins left=\"0.3\" right=\"0.3\" top=\"0.5\" bottom=\"0.5\" header=\"0.2\" footer=\"0.2\"/>"
                "<drawing r:id=\"rId1\"/></worksheet>")).toUtf8();
        }

        QByteArray makeAnnotationWorksheet(const ThicknessPickReportSection& section)
        {
            QString xml = QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\"?><worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
                "<dimension ref=\"A1:I%1\"/><sheetViews><sheetView workbookViewId=\"0\"><pane ySplit=\"5\" topLeftCell=\"A6\" activePane=\"bottomLeft\" state=\"frozen\"/></sheetView></sheetViews>"
                "<sheetFormatPr defaultRowHeight=\"20\"/><cols><col min=\"1\" max=\"2\" width=\"14\" customWidth=\"1\"/>"
                "<col min=\"3\" max=\"9\" width=\"21\" customWidth=\"1\"/></cols><sheetData><row r=\"1\" ht=\"30\" customHeight=\"1\">")
                .arg(static_cast<qulonglong>(section.annotations.size() + 5));
            xml += textCell(QStringLiteral("A1"), QStringLiteral("\u622a\u9762\u7b49\u5f27\u957f\u539a\u5ea6\u6807\u6ce8"), 3) + QStringLiteral("</row><row r=\"2\">") +
                textCell(QStringLiteral("A2"), QStringLiteral("\u6807\u6ce8\u95f4\u8ddd (mm)"), 4) + numericCell(QStringLiteral("C2"), section.annotationSpacingMeters * 1000.0, 5) +
                QStringLiteral("</row><row r=\"3\" ht=\"30\" customHeight=\"1\">") +
                textCell(QStringLiteral("A3"), QStringLiteral("\u539a\u5ea6\u4e3a\u62fe\u53d6\u539f\u503c\uff0c\u672a\u653e\u5927\uff1b\u5f27\u957f\u4ece\u5404\u8f6e\u5ed3\u8d77\u70b9\u8ba1\u7b97\uff0c\u672b\u7aef\u8865\u70b9\u95f4\u8ddd\u53ef\u5c0f\u4e8e\u8bbe\u5b9a\u503c\u3002"), 4) +
                QStringLiteral("</row><row r=\"5\" ht=\"27\" customHeight=\"1\">");
            const QStringList headings{ QStringLiteral("\u6807\u6ce8\u7f16\u53f7"), QStringLiteral("\u8f6e\u5ed3"), QStringLiteral("\u5f27\u957f (mm)"),
                QStringLiteral("\u4e09\u89d2\u5f62\u7f16\u53f7"), QStringLiteral("X (mm)"), QStringLiteral("Y (mm)"), QStringLiteral("Z (mm)"), QStringLiteral("\u539a\u5ea6 (\u03bcm)"),
                QStringLiteral("\u524d\u70b9\u95f4\u8ddd (mm)") };
            for(int column = 0; column < headings.size(); ++column) xml += textCell(QString(QChar('A' + column)) + QStringLiteral("5"), headings[column], 1);
            xml += QStringLiteral("</row>");
            for(std::size_t index = 0; index < section.annotations.size(); ++index) {
                const auto& sample = section.annotations[index];
                const QString row = QString::number(static_cast<qulonglong>(index + 6));
                xml += QStringLiteral("<row r=\"%1\">").arg(row) +
                    textCell(QStringLiteral("A") + row, QStringLiteral("P%1").arg(static_cast<qulonglong>(index + 1), 3, 10, QChar('0'))) +
                    integerCell(QStringLiteral("B") + row, sample.profileIndex + 1) +
                    numericCell(QStringLiteral("C") + row, sample.arcLengthMeters * 1000.0, 5) +
                    integerCell(QStringLiteral("D") + row, sample.triangleIndex) + numericCell(QStringLiteral("E") + row, 0.0, 5) +
                    numericCell(QStringLiteral("F") + row, sample.positionMeters.y() * 1000.0, 5) +
                    numericCell(QStringLiteral("G") + row, sample.positionMeters.z() * 1000.0, 5) +
                    numericCell(QStringLiteral("H") + row, sample.thicknessMeters * 1e6, 6) +
                    numericCell(QStringLiteral("I") + row, index > 0 && section.annotations[index - 1].profileIndex == sample.profileIndex
                        ? (sample.arcLengthMeters - section.annotations[index - 1].arcLengthMeters) * 1000.0 : 0.0, 5) + QStringLiteral("</row>");
            }
            xml += QStringLiteral("</sheetData><autoFilter ref=\"A5:I%1\"/><mergeCells count=\"3\"><mergeCell ref=\"A1:I1\"/>"
                "<mergeCell ref=\"A2:B2\"/><mergeCell ref=\"A3:I3\"/></mergeCells></worksheet>").arg(static_cast<qulonglong>(section.annotations.size() + 5));
            return xml.toUtf8();
        }

        void append16(QByteArray& bytes, std::uint16_t value)
        {
            bytes.append(static_cast<char>(value & 255));
            bytes.append(static_cast<char>((value >> 8) & 255));
        }

        void append32(QByteArray& bytes, std::uint32_t value)
        {
            append16(bytes, static_cast<std::uint16_t>(value));
            append16(bytes, static_cast<std::uint16_t>(value >> 16));
        }

        std::uint32_t crc32(const QByteArray& bytes)
        {
            std::uint32_t crc = 0xffffffffu;
            for(const unsigned char byte : bytes) {
                crc ^= byte;
                for(int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
            }
            return ~crc;
        }

        using ZipEntry = std::pair<QByteArray, QByteArray>;

        void writeZip(const QString& path, const std::vector<ZipEntry>& entries)
        {
            QSaveFile file(path);
            file.setDirectWriteFallback(false);
            if(!file.open(QIODevice::WriteOnly)) fail(QStringLiteral("Thickness pick report: cannot create output: ") + file.errorString());
            const auto write = [&](const QByteArray& bytes) {
                qint64 offset = 0;
                while(offset < bytes.size()) {
                    const qint64 written = file.write(bytes.constData() + offset, bytes.size() - offset);
                    if(written <= 0) fail(QStringLiteral("Thickness pick report: output write failed: ") + file.errorString());
                    offset += written;
                }
            };
            QByteArray central;
            if(entries.size() >= 65535) fail(QStringLiteral("Thickness pick report: too many ZIP entries."));
            for(const auto& entry : entries) {
                if(file.pos() > std::numeric_limits<std::uint32_t>::max() || entry.first.size() > 65535 ||
                    static_cast<quint64>(entry.second.size()) > std::numeric_limits<std::uint32_t>::max()) {
                    fail(QStringLiteral("Thickness pick report: workbook exceeds ZIP32 limits."));
                }
                const auto offset = static_cast<std::uint32_t>(file.pos());
                const auto size = static_cast<std::uint32_t>(entry.second.size());
                const auto crc = crc32(entry.second);
                QByteArray header;
                append32(header, 0x04034b50); append16(header, 20); append16(header, 0x0800);
                append16(header, 0); append16(header, 0); append16(header, 33);
                append32(header, crc); append32(header, size); append32(header, size);
                append16(header, static_cast<std::uint16_t>(entry.first.size())); append16(header, 0);
                header += entry.first;
                write(header); write(entry.second);
                append32(central, 0x02014b50); append16(central, 20); append16(central, 20);
                append16(central, 0x0800); append16(central, 0); append16(central, 0); append16(central, 33);
                append32(central, crc); append32(central, size); append32(central, size);
                append16(central, static_cast<std::uint16_t>(entry.first.size())); append16(central, 0);
                append16(central, 0); append16(central, 0); append16(central, 0); append32(central, 0);
                append32(central, offset); central += entry.first;
            }
            if(file.pos() + central.size() + 22 > std::numeric_limits<std::uint32_t>::max()) {
                fail(QStringLiteral("Thickness pick report: workbook exceeds ZIP32 limits."));
            }
            const auto directoryOffset = static_cast<std::uint32_t>(file.pos());
            write(central);
            QByteArray end;
            append32(end, 0x06054b50); append16(end, 0); append16(end, 0);
            append16(end, static_cast<std::uint16_t>(entries.size())); append16(end, static_cast<std::uint16_t>(entries.size()));
            append32(end, static_cast<std::uint32_t>(central.size())); append32(end, directoryOffset); append16(end, 0);
            write(end);
            if(!file.commit()) fail(QStringLiteral("Thickness pick report: cannot commit workbook: ") + file.errorString());
        }
    }

    ThicknessPickReportExportResult exportThicknessPickReport(
        const std::vector<ThicknessPickReportSample>& samples,
        const QString& modelName,
        const QString& outputDirectory,
        const ThicknessPickReportSection* section)
    {
        if(samples.empty()) fail(QStringLiteral("Thickness pick report: no samples lie on the world YZ plane (x = 0)."));
        if(samples.size() > kMaximumSamples) fail(QStringLiteral("Thickness pick report: sample count exceeds 200000."));
        for(const auto& sample : samples) {
            if(!sample.positionMeters.allFinite() || sample.positionMeters.x() != 0.0) {
                fail(QStringLiteral("Thickness pick report: positions must be finite and lie exactly on x = 0."));
            }
            if(!std::isfinite(sample.thicknessMeters) || sample.thicknessMeters < 0.0 ||
                !std::isfinite(sample.thicknessMeters * 1e6) ||
                !std::isfinite(sample.positionMeters.y() * 1000.0) ||
                !std::isfinite(sample.positionMeters.z() * 1000.0)) {
                fail(QStringLiteral("Thickness pick report: thickness must be finite, nonnegative and representable in report units."));
            }
            if(sample.triangleIndex > 9007199254740991ULL) {
                fail(QStringLiteral("Thickness pick report: triangle index exceeds the exact Excel integer range."));
            }
        }
        if(outputDirectory.trimmed().isEmpty()) fail(QStringLiteral("Thickness pick report: output directory is empty."));
        if(section) validateSection(*section);

        const QDateTime exportTime = QDateTime::currentDateTimeUtc();
        std::vector<ZipEntry> entries;
        entries.emplace_back("[Content_Types].xml", QByteArray("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
            "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
            "<Default Extension=\"xml\" ContentType=\"application/xml\"/>"
            "<Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
            "<Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>"
            "<Override PartName=\"/xl/worksheets/sheet1.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
            "</Types>"));
        entries.emplace_back("_rels/.rels", QByteArray("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
            "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/>"
            "</Relationships>"));
        entries.emplace_back("xl/workbook.xml", QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">"
            "<bookViews><workbookView/></bookViews><sheets><sheet name=\"\u62fe\u53d6\u539a\u5ea6\" sheetId=\"1\" r:id=\"rId1\"/></sheets></workbook>").toUtf8());
        entries.emplace_back("xl/_rels/workbook.xml.rels", QByteArray("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
            "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet1.xml\"/>"
            "<Relationship Id=\"rId2\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/></Relationships>"));
        entries.emplace_back("xl/styles.xml", QByteArray("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
            "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
            "<numFmts count=\"3\"><numFmt numFmtId=\"164\" formatCode=\"0.000000#########\"/>"
            "<numFmt numFmtId=\"165\" formatCode=\"0.000\"/><numFmt numFmtId=\"166\" formatCode=\"0.0\"/></numFmts>"
            "<fonts count=\"3\"><font><sz val=\"11\"/><name val=\"Calibri\"/></font>"
            "<font><b/><color rgb=\"FFFFFFFF\"/><sz val=\"11\"/><name val=\"Calibri\"/></font>"
            "<font><b/><sz val=\"16\"/><name val=\"Calibri\"/></font></fonts>"
            "<fills count=\"3\"><fill><patternFill patternType=\"none\"/></fill><fill><patternFill patternType=\"gray125\"/></fill>"
            "<fill><patternFill patternType=\"solid\"><fgColor rgb=\"FF203C5A\"/><bgColor indexed=\"64\"/></patternFill></fill></fills>"
            "<borders count=\"1\"><border><left/><right/><top/><bottom/><diagonal/></border></borders>"
            "<cellStyleXfs count=\"1\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\"/></cellStyleXfs>"
            "<cellXfs count=\"7\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\"/>"
            "<xf numFmtId=\"0\" fontId=\"1\" fillId=\"2\" borderId=\"0\" xfId=\"0\" applyFill=\"1\" applyFont=\"1\" applyAlignment=\"1\"><alignment vertical=\"center\"/></xf>"
            "<xf numFmtId=\"164\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\"/>"
            "<xf numFmtId=\"0\" fontId=\"2\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyFont=\"1\" applyAlignment=\"1\"><alignment vertical=\"center\"/></xf>"
            "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyAlignment=\"1\"><alignment wrapText=\"1\" vertical=\"center\"/></xf>"
            "<xf numFmtId=\"165\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\"/>"
            "<xf numFmtId=\"166\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\" applyNumberFormat=\"1\"/>"
            "</cellXfs><cellStyles count=\"1\"><cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/></cellStyles></styleSheet>"));
        entries.emplace_back("xl/worksheets/sheet1.xml", makeWorksheet(samples, modelName, exportTime, section != nullptr));
        if(section) {
            const auto panels = makeSchematicPanels(*section);
            const auto addXml = [&](const QByteArray& name, const QByteArray& closing, const QByteArray& xml) {
                for(auto& entry : entries) {
                    if(entry.first == name) { entry.second.replace(closing, xml + closing); return; }
                }
                fail(QStringLiteral("Thickness pick report: missing workbook part."));
            };
            addXml("[Content_Types].xml", "</Types>", QByteArray(
                "<Default Extension=\"png\" ContentType=\"image/png\"/>"
                "<Override PartName=\"/xl/worksheets/sheet2.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
                "<Override PartName=\"/xl/worksheets/sheet3.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
                "<Override PartName=\"/xl/drawings/drawing1.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.drawing+xml\"/>"));
            addXml("xl/workbook.xml", "</sheets>", QStringLiteral(
                "<sheet name=\"\u622a\u9762\u793a\u610f\u56fe\" sheetId=\"2\" r:id=\"rId3\"/>"
                "<sheet name=\"\u7b49\u8ddd\u6807\u6ce8\" sheetId=\"3\" r:id=\"rId4\"/>").toUtf8());
            addXml("xl/_rels/workbook.xml.rels", "</Relationships>", QByteArray(
                "<Relationship Id=\"rId3\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet2.xml\"/>"
                "<Relationship Id=\"rId4\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet3.xml\"/>"));
            entries.emplace_back("xl/worksheets/sheet2.xml", makeSchematicWorksheet(panels.size()));
            entries.emplace_back("xl/worksheets/sheet3.xml", makeAnnotationWorksheet(*section));
            entries.emplace_back("xl/worksheets/_rels/sheet2.xml.rels", QByteArray(
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?><Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/drawing\" Target=\"../drawings/drawing1.xml\"/></Relationships>"));
            QString drawing = QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                "<xdr:wsDr xmlns:xdr=\"http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing\" xmlns:a=\"http://schemas.openxmlformats.org/drawingml/2006/main\">");
            QString relationships = QStringLiteral("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">");
            for(std::size_t index = 0; index < panels.size(); ++index) {
                const QString id = QString::number(static_cast<qulonglong>(index + 1));
                drawing += QStringLiteral("<xdr:oneCellAnchor><xdr:from><xdr:col>0</xdr:col><xdr:colOff>0</xdr:colOff>"
                    "<xdr:row>%1</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from><xdr:ext cx=\"10972800\" cy=\"11430000\"/>"
                    "<xdr:pic><xdr:nvPicPr><xdr:cNvPr id=\"%2\" name=\"Section schematic %2\" descr=\"%3\"/>"
                    "<xdr:cNvPicPr><a:picLocks noChangeAspect=\"1\"/></xdr:cNvPicPr></xdr:nvPicPr>"
                    "<xdr:blipFill><a:blip xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\" r:embed=\"rId%2\"/>"
                    "<a:stretch><a:fillRect/></a:stretch></xdr:blipFill><xdr:spPr><a:prstGeom prst=\"rect\"><a:avLst/></a:prstGeom></xdr:spPr>"
                    "</xdr:pic><xdr:clientData/></xdr:oneCellAnchor>")
                    .arg(static_cast<qulonglong>(3 + index * kImageRows)).arg(id, xmlText(panels[index].description));
                relationships += QStringLiteral("<Relationship Id=\"rId%1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/image\" Target=\"../media/section%1.png\"/>").arg(id);
                entries.emplace_back(QStringLiteral("xl/media/section%1.png").arg(id).toUtf8(), panels[index].png);
            }
            drawing += QStringLiteral("</xdr:wsDr>"); relationships += QStringLiteral("</Relationships>");
            entries.emplace_back("xl/drawings/drawing1.xml", drawing.toUtf8());
            entries.emplace_back("xl/drawings/_rels/drawing1.xml.rels", relationships.toUtf8());
        }

        QDir directory(outputDirectory);
        if(!directory.exists() && !QDir().mkpath(directory.absolutePath())) {
            fail(QStringLiteral("Thickness pick report: cannot create output directory."));
        }
        const std::string stem = smrobot::exportfiles::modelLabel(modelName.toUtf8().toStdString()) +
            u8"_YZ\u622a\u9762\u539a\u5ea6\u62a5\u544a";
        const auto outputPath = smrobot::exportfiles::nextAvailablePath(
            std::filesystem::u8path(directory.absolutePath().toUtf8().toStdString()), stem, ".xlsx");
        const QString path = QString::fromUtf8(outputPath.u8string().c_str());
        if(QFileInfo::exists(path)) fail(QStringLiteral("Thickness pick report: unique output name already exists."));
        writeZip(path, entries);
        return { path, samples.size() };
    }
}
