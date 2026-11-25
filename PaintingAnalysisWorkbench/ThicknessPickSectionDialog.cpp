#include "ThicknessPickSectionDialog.h"
#include "ThicknessSectionSchematic.h"
#include "CoatingAnalysisLanguage.h"

#include <QComboBox>

#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVariant>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <utility>

namespace robot_qt_viewer
{
    class ThicknessPickSectionDialog::SectionCanvas : public QWidget
    {
    public:
        explicit SectionCanvas(
            const std::vector<ThicknessPickSectionSegment>& segments, QWidget* parent)
            : QWidget(parent), m_segments(segments)
        {
            setObjectName(QStringLiteral("thicknessSectionCanvas"));
            setMinimumSize(620, 420);
            setFocusPolicy(Qt::StrongFocus);
            setMouseTracking(true);
            setCursor(Qt::CrossCursor);
            fitView();
        }

        std::function<void()> selectionChanged;

        const std::vector<Eigen::Vector2d>& polygon() const { return m_polygon; }
        const std::vector<ThicknessPickSectionSegment>& segments() const { return m_segments; }
        bool hasSelection() const { return m_closed && !m_selected.empty(); }
        bool canFinish() const { return !m_closed && m_polygon.size() >= 3; }
        bool isClosed() const { return m_closed; }
        bool invalidPolygon() const { return m_invalidPolygon; }
        bool hasSegments() const { return !m_segments.empty(); }

        double selectedLengthMeters() const
        {
            double length = 0.0;
            for(const auto& segment : m_selected) {
                length += (segment.second.positionMeters - segment.first.positionMeters).norm();
            }
            return length;
        }

        void setChinese(bool chinese)
        {
            m_chinese = chinese;
            update();
        }

        void clearSelection()
        {
            m_polygon.clear();
            m_selected.clear();
            m_closed = false;
            m_invalidPolygon = false;
            m_pressed = false;
            m_lasso = false;
            changed();
        }

        void finishSelection()
        {
            m_pressed = false;
            m_lasso = false;
            if(m_closed || m_polygon.size() < 3) {
                changed();
                return;
            }
            m_closed = true;
            m_invalidPolygon = false;
            try {
                m_selected = clipYzThicknessSegments(m_segments, m_polygon);
            } catch(const std::exception&) {
                m_selected.clear();
                m_invalidPolygon = true;
            }
            changed();
        }

        void fitView()
        {
            Eigen::Vector2d minimum = Eigen::Vector2d::Constant(
                std::numeric_limits<double>::max());
            Eigen::Vector2d maximum = Eigen::Vector2d::Constant(
                std::numeric_limits<double>::lowest());
            for(const auto& segment : m_segments) {
                for(const auto* sample : { &segment.first, &segment.second }) {
                    const Eigen::Vector2d point(sample->positionMeters.y(), sample->positionMeters.z());
                    if(point.allFinite()) {
                        minimum = minimum.cwiseMin(point);
                        maximum = maximum.cwiseMax(point);
                    }
                }
            }
            if(minimum.x() > maximum.x()) {
                minimum = Eigen::Vector2d(-0.05, -0.05);
                maximum = Eigen::Vector2d(0.05, 0.05);
            }
            m_center = (minimum + maximum) * 0.5;
            const double longest = std::max(1.0e-6, (maximum - minimum).maxCoeff());
            m_fitSpan = (maximum - minimum)
                .cwiseMax(Eigen::Vector2d::Constant(longest * 0.05));
            m_fitSpan *= 1.2;
            m_zoom = 1.0;
            updateMappingProperties();
            update();
        }

    protected:
        void resizeEvent(QResizeEvent* event) override
        {
            QWidget::resizeEvent(event);
            updateMappingProperties();
        }

        void paintEvent(QPaintEvent*) override
        {
            updateMappingProperties();
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.fillRect(rect(), QColor(248, 250, 252));
            const QRectF plot = plotRectangle();
            painter.fillRect(plot, Qt::white);
            painter.setPen(QPen(QColor(215, 223, 231), 1.0));
            painter.drawRect(plot);

            QFont tickFont = font();
            tickFont.setPointSizeF(std::max(8.0, tickFont.pointSizeF() - 1.0));
            painter.setFont(tickFont);
            const auto tickText = [](double millimeters, double stepMillimeters) {
                if(std::abs(millimeters) >= 1.0e6) {
                    return QString::number(millimeters, 'g', 5);
                }
                const int decimals = stepMillimeters >= 1.0 ? 1
                    : std::clamp(static_cast<int>(std::ceil(-std::log10(stepMillimeters))) + 1, 1, 6);
                QString result = QString::number(millimeters, 'f', decimals);
                while(result.endsWith(QChar('0'))) result.chop(1);
                if(result.endsWith(QChar('.'))) result.chop(1);
                return result == QStringLiteral("-0") ? QStringLiteral("0") : result;
            };
            const double horizontalStep = plot.width() * 250.0 / pixelsPerMeter();
            const double verticalStep = plot.height() * 250.0 / pixelsPerMeter();
            for(int index = 0; index <= 4; ++index) {
                const double fraction = static_cast<double>(index) / 4.0;
                const double x = plot.left() + fraction * plot.width();
                const double y = plot.bottom() - fraction * plot.height();
                painter.setPen(QPen(QColor(235, 240, 245), 1.0));
                painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
                painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
                painter.setPen(QColor(80, 94, 110));
                QRectF horizontalLabel(x - 42.0, plot.bottom() + 5.0, 84.0, 20.0);
                Qt::Alignment horizontalAlignment = Qt::AlignHCenter | Qt::AlignTop;
                if(index == 0) {
                    horizontalLabel.moveLeft(plot.left());
                    horizontalAlignment = Qt::AlignLeft | Qt::AlignTop;
                } else if(index == 4) {
                    horizontalLabel.moveRight(plot.right());
                    horizontalAlignment = Qt::AlignRight | Qt::AlignTop;
                }
                painter.drawText(horizontalLabel, horizontalAlignment,
                    tickText(screenToSection(QPointF(x, plot.bottom())).x() * 1000.0, horizontalStep));
                painter.drawText(QRectF(0.0, y - 11.0, plot.left() - 7.0, 22.0),
                    Qt::AlignRight | Qt::AlignVCenter,
                    tickText(screenToSection(QPointF(plot.left(), y)).y() * 1000.0, verticalStep));
            }
            painter.setPen(QColor(50, 65, 80));
            painter.drawText(QRectF(plot.left(), plot.bottom() + 26.0, plot.width(), 22.0),
                Qt::AlignCenter, QStringLiteral("Y (mm)"));
            painter.drawText(QRectF(4.0, 0.0, 62.0, 20.0),
                Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Z (mm)"));

            painter.save();
            painter.setClipRect(plot);
            if(m_polygon.size() >= 2) {
                QPolygonF boundary;
                for(const auto& point : m_polygon) {
                    boundary << sectionToScreen(point);
                }
                painter.setPen(QPen(QColor(27, 136, 150), 1.2, Qt::DashLine));
                if(m_closed) {
                    painter.setBrush(QColor(18, 152, 168, 24));
                    painter.drawPolygon(boundary, Qt::OddEvenFill);
                } else {
                    painter.setBrush(Qt::NoBrush);
                    painter.drawPolyline(boundary);
                    if(m_hovering && !m_lasso) {
                        painter.drawLine(boundary.back(), m_cursor);
                        if(boundary.size() >= 3) {
                            painter.drawLine(m_cursor, boundary.front());
                        }
                    }
                }
            }

            painter.setPen(QPen(QColor(158, 169, 181), 1.6));
            for(const auto& segment : m_segments) {
                drawSegment(painter, segment);
            }
            painter.setPen(QPen(QColor(0, 142, 120), 3.0, Qt::SolidLine, Qt::RoundCap));
            for(const auto& segment : m_selected) {
                drawSegment(painter, segment);
            }
            if(!m_closed) {
                painter.setPen(QPen(QColor(27, 136, 150), 1.0));
                painter.setBrush(Qt::white);
                for(const auto& point : m_polygon) {
                    painter.drawEllipse(sectionToScreen(point), 3.0, 3.0);
                }
            }
            painter.restore();
            painter.setPen(QColor(90, 105, 120));
            painter.drawText(QRectF(plot.left() + 10.0, plot.top() + 9.0, plot.width() - 20.0, 24.0),
                Qt::AlignLeft | Qt::AlignVCenter,
                m_chinese
                    ? QStringLiteral("\u7070\u8272\uff1a\u5168\u90e8\u622a\u9762    \u7eff\u8272\uff1a\u5c06\u5bfc\u51fa\u7684\u8868\u9762")
                    : QStringLiteral("Gray: complete section    Green: surface to export"));
        }

        void mousePressEvent(QMouseEvent* event) override
        {
            if(!plotRectangle().contains(event->pos())) {
                return;
            }
            setFocus(Qt::MouseFocusReason);
            if(event->button() == Qt::MiddleButton) {
                m_panning = true;
                m_panPosition = event->pos();
                setCursor(Qt::ClosedHandCursor);
                return;
            }
            if(event->button() != Qt::LeftButton) {
                return;
            }
            if(m_closed) {
                clearSelection();
            }
            m_pressed = true;
            m_lasso = false;
            m_startingPolygon = m_polygon.empty();
            m_pressPosition = event->pos();
            m_cursor = event->pos();
        }

        void mouseMoveEvent(QMouseEvent* event) override
        {
            m_cursor = clampToPlot(event->pos());
            m_hovering = plotRectangle().contains(event->pos());
            if(m_panning) {
                const QPointF delta = event->pos() - m_panPosition;
                m_center += Eigen::Vector2d(-delta.x(), delta.y()) / pixelsPerMeter();
                m_panPosition = event->pos();
                updateMappingProperties();
            } else if(m_pressed && m_startingPolygon) {
                if(!m_lasso && distance(m_cursor, m_pressPosition) >= 5.0) {
                    m_lasso = true;
                    m_polygon.push_back(screenToSection(m_pressPosition));
                }
                if(m_lasso) {
                    appendPoint(m_cursor, 3.0);
                }
            }
            update();
        }

        void mouseReleaseEvent(QMouseEvent* event) override
        {
            if(event->button() == Qt::MiddleButton) {
                m_panning = false;
                setCursor(Qt::CrossCursor);
                return;
            }
            if(event->button() != Qt::LeftButton || !m_pressed) {
                return;
            }
            m_pressed = false;
            appendPoint(clampToPlot(event->pos()), 1.0);
            if(m_lasso) {
                finishSelection();
            } else {
                changed();
            }
        }

        void mouseDoubleClickEvent(QMouseEvent* event) override
        {
            if(event->button() == Qt::LeftButton && !m_closed
                && plotRectangle().contains(event->pos())) {
                appendPoint(event->pos(), 1.0);
                m_pressed = false;
                finishSelection();
            }
        }

        void leaveEvent(QEvent* event) override
        {
            m_hovering = false;
            update();
            QWidget::leaveEvent(event);
        }

        void keyPressEvent(QKeyEvent* event) override
        {
            if(event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
                finishSelection();
                event->accept();
                return;
            }
            if((event->key() == Qt::Key_Backspace || event->key() == Qt::Key_Delete)
                && !m_polygon.empty()) {
                m_closed = false;
                m_invalidPolygon = false;
                m_selected.clear();
                m_polygon.pop_back();
                changed();
                return;
            }
            if(event->key() == Qt::Key_Escape && !m_polygon.empty()) {
                clearSelection();
                return;
            }
            QWidget::keyPressEvent(event);
        }

        void wheelEvent(QWheelEvent* event) override
        {
            const QPointF point = event->position();
            if(!plotRectangle().contains(point) || event->angleDelta().y() == 0) {
                event->ignore();
                return;
            }
            const Eigen::Vector2d before = screenToSection(point);
            m_zoom = std::clamp(m_zoom * std::pow(1.15,
                static_cast<double>(event->angleDelta().y()) / 120.0), 0.25, 80.0);
            m_center += before - screenToSection(point);
            updateMappingProperties();
            update();
            event->accept();
        }

    private:
        void changed()
        {
            setProperty("selectionClosed", m_closed);
            if(selectionChanged) {
                selectionChanged();
            }
            update();
        }

        static double distance(const QPointF& first, const QPointF& second)
        {
            return std::hypot(first.x() - second.x(), first.y() - second.y());
        }

        void appendPoint(const QPointF& point, double minimumPixels)
        {
            if(m_polygon.empty() || distance(sectionToScreen(m_polygon.back()), point) >= minimumPixels) {
                m_polygon.push_back(screenToSection(point));
            }
        }

        QRectF plotRectangle() const
        {
            return QRectF(78.0, 24.0,
                std::max(1, width() - 102), std::max(1, height() - 82));
        }

        double pixelsPerMeter() const
        {
            const QRectF plot = plotRectangle();
            return m_zoom * std::min(plot.width() / m_fitSpan.x(), plot.height() / m_fitSpan.y());
        }

        QPointF sectionToScreen(const Eigen::Vector2d& point) const
        {
            const Eigen::Vector2d offset = (point - m_center) * pixelsPerMeter();
            return plotRectangle().center() + QPointF(offset.x(), -offset.y());
        }

        Eigen::Vector2d screenToSection(const QPointF& point) const
        {
            const QPointF offset = point - plotRectangle().center();
            return m_center + Eigen::Vector2d(offset.x(), -offset.y()) / pixelsPerMeter();
        }

        QPointF clampToPlot(const QPointF& point) const
        {
            const QRectF plot = plotRectangle();
            return QPointF(std::clamp(point.x(), plot.left(), plot.right()),
                std::clamp(point.y(), plot.top(), plot.bottom()));
        }

        void drawSegment(QPainter& painter, const ThicknessPickSectionSegment& segment) const
        {
            painter.drawLine(sectionToScreen(Eigen::Vector2d(
                segment.first.positionMeters.y(), segment.first.positionMeters.z())),
                sectionToScreen(Eigen::Vector2d(
                    segment.second.positionMeters.y(), segment.second.positionMeters.z())));
        }

        void updateMappingProperties()
        {
            setProperty("sectionCenterY", m_center.x());
            setProperty("sectionCenterZ", m_center.y());
            setProperty("sectionPixelsPerMeter", pixelsPerMeter());
            setProperty("sectionPlotRect", plotRectangle());
        }

        std::vector<ThicknessPickSectionSegment> m_segments;
        std::vector<ThicknessPickSectionSegment> m_selected;
        std::vector<Eigen::Vector2d> m_polygon;
        Eigen::Vector2d m_center = Eigen::Vector2d::Zero();
        Eigen::Vector2d m_fitSpan = Eigen::Vector2d::Constant(0.1);
        double m_zoom = 1.0;
        QPointF m_pressPosition;
        QPointF m_panPosition;
        QPointF m_cursor;
        bool m_closed = false;
        bool m_invalidPolygon = false;
        bool m_pressed = false;
        bool m_startingPolygon = false;
        bool m_lasso = false;
        bool m_hovering = false;
        bool m_panning = false;
        bool m_chinese = false;
    };

    ThicknessPickSectionDialog::ThicknessPickSectionDialog(
        const std::vector<ThicknessPickSectionSegment>& segments, QWidget* parent)
        : QDialog(parent)
    {
        setObjectName(QStringLiteral("thicknessPickSectionDialog"));
        resize(960, 780);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(18, 16, 18, 16);
        layout->setSpacing(10);
        m_description = new QLabel(this);
        m_description->setWordWrap(true);
        layout->addWidget(m_description);
        m_canvas = new SectionCanvas(segments, this);
        m_canvas->selectionChanged = [this]() { updateSelectionStatus(); };
        layout->addWidget(m_canvas, 1);

        auto* options = new QHBoxLayout;
        m_spacingLabel = new QLabel(this);
        options->addWidget(m_spacingLabel);
        m_spacing = new QDoubleSpinBox(this);
        m_spacing->setObjectName(QStringLiteral("thicknessSectionSpacing"));
        m_spacing->setDecimals(2);
        m_spacing->setRange(0.01, 10000.0);
        m_spacing->setSingleStep(1.0);
        m_spacing->setValue(10.0);
        m_spacing->setSuffix(QStringLiteral(" mm"));
        m_spacingLabel->setBuddy(m_spacing);
        options->addWidget(m_spacing);
        options->addStretch();
        m_finishButton = new QPushButton(this);
        m_finishButton->setObjectName(QStringLiteral("thicknessSectionFinish"));
        m_finishButton->setAutoDefault(false);
        options->addWidget(m_finishButton);
        m_fitButton = new QPushButton(this);
        m_fitButton->setObjectName(QStringLiteral("thicknessSectionFit"));
        m_fitButton->setAutoDefault(false);
        options->addWidget(m_fitButton);
        m_clearButton = new QPushButton(this);
        m_clearButton->setObjectName(QStringLiteral("thicknessSectionClear"));
        m_clearButton->setAutoDefault(false);
        options->addWidget(m_clearButton);
        layout->addLayout(options);

        auto* multiplierOptions = new QHBoxLayout;
        m_multiplierLabel = new QLabel(this);
        multiplierOptions->addWidget(m_multiplierLabel);
        m_multiplierMode = new QComboBox(this);
        m_multiplierMode->setObjectName(QStringLiteral("thicknessSectionMultiplierMode"));
        m_multiplierMode->addItem(QString());
        m_multiplierMode->addItem(QString());
        m_multiplierLabel->setBuddy(m_multiplierMode);
        multiplierOptions->addWidget(m_multiplierMode);
        m_multiplier = new QDoubleSpinBox(this);
        m_multiplier->setObjectName(QStringLiteral("thicknessSectionMultiplier"));
        m_multiplier->setRange(1.0, 1000.0);
        m_multiplier->setDecimals(2);
        m_multiplier->setSingleStep(1.0);
        m_multiplier->setValue(25.0);
        m_multiplier->setSuffix(QStringLiteral(" x"));
        m_multiplier->setEnabled(false);
        multiplierOptions->addWidget(m_multiplier);
        multiplierOptions->addStretch();
        layout->addLayout(multiplierOptions);
        m_multiplierExplanation = new QLabel(this);
        m_multiplierExplanation->setWordWrap(true);
        layout->addWidget(m_multiplierExplanation);

        m_selectionStatus = new QLabel(this);
        m_selectionStatus->setObjectName(QStringLiteral("thicknessSectionStatus"));
        m_selectionStatus->setWordWrap(true);
        m_selectionStatus->setMinimumHeight(32);
        layout->addWidget(m_selectionStatus);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        m_exportButton = buttons->button(QDialogButtonBox::Ok);
        m_exportButton->setObjectName(QStringLiteral("thicknessSectionExport"));
        m_exportButton->setAutoDefault(false);
        m_exportButton->setDefault(false);
        m_cancelButton = buttons->button(QDialogButtonBox::Cancel);
        m_cancelButton->setObjectName(QStringLiteral("thicknessSectionCancel"));
        m_cancelButton->setAutoDefault(false);
        layout->addWidget(buttons);

        connect(m_multiplierMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int index) {
                m_multiplier->setEnabled(index == 1);
                updateSelectionStatus();
            });
        connect(m_multiplier, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double) { updateSelectionStatus(); });
        connect(m_finishButton, &QPushButton::clicked, m_canvas, &SectionCanvas::finishSelection);
        connect(m_fitButton, &QPushButton::clicked, m_canvas, &SectionCanvas::fitView);
        connect(m_clearButton, &QPushButton::clicked, m_canvas, &SectionCanvas::clearSelection);
        connect(buttons, &QDialogButtonBox::accepted, this, &ThicknessPickSectionDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        setLanguageCode(QStringLiteral("en"));
    }

    std::vector<Eigen::Vector2d> ThicknessPickSectionDialog::selectionPolygon() const
    {
        return m_canvas->hasSelection() ? m_canvas->polygon() : std::vector<Eigen::Vector2d>();
    }

    double ThicknessPickSectionDialog::annotationSpacingMeters() const
    {
        return m_spacing->value() * 0.001;
    }

    ThicknessSectionSchematicOptions ThicknessPickSectionDialog::schematicOptions() const
    {
        ThicknessSectionSchematicOptions result;
        result.automaticThicknessMultiplier = m_multiplierMode->currentIndex() == 0;
        result.fixedThicknessMultiplier = m_multiplier->value();
        return result;
    }

    const ThicknessPickReportSection& ThicknessPickSectionDialog::selectedSection() const
    {
        return m_selectedSection;
    }

    void ThicknessPickSectionDialog::accept()
    {
        if(!m_canvas->hasSelection()) {
            updateSelectionStatus();
            return;
        }
        try {
            auto section = buildSelectedYzThicknessSection(m_canvas->segments(),
                selectionPolygon(), annotationSpacingMeters());
            section.schematicOptions = schematicOptions();
            if(!section.schematicOptions.automaticThicknessMultiplier) {
                // Validate before closing, so a rejected fixed multiplier can
                // be corrected directly without selecting the surface again.
                for(std::size_t index = 0; index < section.profiles.size(); ++index) {
                    std::vector<ThicknessPickReportSample> annotations;
                    for(const auto& sample : section.annotations) {
                        if(sample.profileIndex == index) annotations.push_back(sample);
                    }
                    buildThicknessSectionSchematic(section.profiles[index], annotations,
                        section.schematicOptions);
                }
            }
            m_selectedSection = std::move(section);
            QDialog::accept();
        } catch(const ThicknessDisplayMultiplierError&) {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u56fa\u5b9a %1 \u500d\u4f1a\u4f7f\u8be5\u622a\u9762\u7684\u6d82\u5c42\u8f6e\u5ed3\u4ea4\u53c9\u6216\u56de\u6298\uff0c\u672a\u5bfc\u51fa\u3002\u8bf7\u6539\u7528\u201c\u81ea\u52a8\uff08\u907f\u514d\u56de\u6298\uff09\u201d\u6216\u8f83\u5c0f\u500d\u7387\uff0c\u7136\u540e\u518d\u6b21\u5bfc\u51fa\uff1b\u5f53\u524d\u9009\u533a\u5df2\u4fdd\u7559\u3002")
                    .arg(m_multiplier->value(), 0, 'g', 6)
                : QStringLiteral("A fixed %1 x multiplier would make this coating outline cross or fold, so no report was exported. Choose Automatic (avoid folds) or a smaller multiplier, then export again. Your surface selection is retained.")
                    .arg(m_multiplier->value(), 0, 'g', 6));
            m_multiplier->setFocus();
        } catch(const std::exception& exception) {
            const QString detail = coatingAnalysisTranslate(m_chinese ? QStringLiteral("zh-CN") : QStringLiteral("en"),
                QString::fromUtf8(exception.what()));
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u65e0\u6cd5\u5bfc\u51fa\u5f53\u524d\u9009\u533a\uff1a%1").arg(detail)
                : QStringLiteral("Cannot export the current selection: %1").arg(detail));
        }
    }

    void ThicknessPickSectionDialog::setLanguageCode(const QString& languageCode)
    {
        m_chinese = languageCode.toLower().startsWith(QStringLiteral("zh"));
        setWindowTitle(m_chinese
            ? QStringLiteral("\u9009\u62e9\u8981\u5bfc\u51fa\u7684\u622a\u9762\u8868\u9762\uff08YZ\uff0cx = 0\uff09")
            : QStringLiteral("Select the section surface to export (YZ, x = 0)"));
        m_description->setText(m_chinese
            ? QStringLiteral("\u5708\u4f4f\u9700\u8981\u5bfc\u51fa\u7684\u90a3\u4e00\u4fa7\u8868\u9762\u66f2\u7ebf\uff1a\u9010\u70b9\u5355\u51fb\u540e\u53cc\u51fb\u6216\u6309 Enter \u5b8c\u6210\uff0c\u4e5f\u53ef\u6309\u4f4f\u5de6\u952e\u62d6\u52a8\u5708\u9009\u3002\u6eda\u8f6e\u7f29\u653e\uff0c\u4e2d\u952e\u62d6\u52a8\u5e73\u79fb\u3002")
            : QStringLiteral("Enclose the surface curve you want to export. Click vertices, then double-click or press Enter to finish; or hold the left mouse button and draw a lasso. Scroll to zoom; middle-drag to pan."));
        m_spacingLabel->setText(m_chinese
            ? QStringLiteral("\u793a\u610f\u56fe\u6807\u6ce8\u95f4\u8ddd\uff08\u6cbf\u622a\u9762\u5f27\u957f\uff09")
            : QStringLiteral("Diagram label spacing (along the section)"));
        m_spacing->setToolTip(m_chinese
            ? QStringLiteral("\u53ea\u8c03\u6574\u793a\u610f\u56fe\u7684\u7b49\u8ddd\u6807\u6ce8\uff0c\u5bfc\u51fa\u8868\u683c\u4fdd\u7559\u9009\u4e2d\u8868\u9762\u7684\u5b8c\u6574\u91c7\u6837\u3002")
            : QStringLiteral("Controls diagram labels only. The table retains the full sampling of the selected surface."));
        m_multiplierLabel->setText(m_chinese
            ? QStringLiteral("\u539a\u5ea6\u793a\u610f\u663e\u793a\u500d\u7387")
            : QStringLiteral("Thickness diagram multiplier"));
        m_multiplierMode->setItemText(0, m_chinese
            ? QStringLiteral("\u81ea\u52a8\uff08\u907f\u514d\u56de\u6298\uff09")
            : QStringLiteral("Automatic (avoid folds)"));
        m_multiplierMode->setItemText(1, m_chinese
            ? QStringLiteral("\u56fa\u5b9a\u500d\u7387") : QStringLiteral("Fixed multiplier"));
        m_multiplierExplanation->setText(m_chinese
            ? QStringLiteral("\u53ea\u653e\u5927\u793a\u610f\u56fe\u7684\u6d82\u5c42\u539a\u5ea6\uff0c\u4e0d\u662f\u663e\u5fae\u955c\u5149\u5b66\u500d\u7387\uff1b\u8868\u683c\u548c\u6807\u6ce8\u4ecd\u4e3a\u539f\u59cb\u539a\u5ea6\u3002\u56fa\u5b9a\u6a21\u5f0f\u53ef\u8bbe 1\u20131000 \u500d\uff08\u5982 25 \u500d\uff09\uff0c\u5982\u679c\u5bfc\u81f4\u8f6e\u5ed3\u4ea4\u53c9\u6216\u56de\u6298\u4f1a\u63d0\u793a\u8c03\u5c0f\uff0c\u4e0d\u4f1a\u81ea\u52a8\u6539\u53d8\u8bbe\u5b9a\u500d\u7387\u3002")
            : QStringLiteral("Scales coating thickness in the diagram, not microscope optical magnification. Table values and labels remain original. Fixed mode accepts 1-1000 x (for example 25 x); an intersecting or folded outline is rejected instead of silently changing your multiplier."));
        m_finishButton->setText(m_chinese
            ? QStringLiteral("\u5b8c\u6210\u5708\u9009") : QStringLiteral("Finish selection"));
        m_fitButton->setText(m_chinese
            ? QStringLiteral("\u9002\u5408\u7a97\u53e3") : QStringLiteral("Fit view"));
        m_clearButton->setText(m_chinese
            ? QStringLiteral("\u6e05\u9664\u5e76\u91cd\u9009") : QStringLiteral("Clear selection"));
        m_exportButton->setText(m_chinese
            ? QStringLiteral("\u5bfc\u51fa\u9009\u4e2d\u8868\u9762") : QStringLiteral("Export selected surface"));
        m_cancelButton->setText(m_chinese
            ? QStringLiteral("\u53d6\u6d88") : QStringLiteral("Cancel"));
        m_canvas->setChinese(m_chinese);
        updateSelectionStatus();
    }

    void ThicknessPickSectionDialog::updateSelectionStatus()
    {
        const bool valid = m_canvas->hasSelection();
        m_exportButton->setEnabled(valid);
        m_finishButton->setEnabled(m_canvas->canFinish());
        m_clearButton->setEnabled(!m_canvas->polygon().empty());
        if(valid) {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u5df2\u9009\u622a\u9762\u957f\u5ea6\uff1a%1 mm\u3002\u4ec5\u5bfc\u51fa\u7eff\u8272\u8868\u9762\uff0c\u4fdd\u7559\u8be5\u9762\u4e0a\u7684\u771f\u5b9e 0 \u03bcm\u3002").arg(m_canvas->selectedLengthMeters() * 1000.0, 0, 'f', 2)
                : QStringLiteral("Selected length: %1 mm. Export includes the green surface and retains its true 0 \u03bcm values.").arg(m_canvas->selectedLengthMeters() * 1000.0, 0, 'f', 2));
        } else if(!m_canvas->hasSegments()) {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u5f53\u524d\u6a21\u578b\u4e0e YZ\uff08x = 0\uff09\u5e73\u9762\u6ca1\u6709\u53ef\u5bfc\u51fa\u7684\u622a\u9762\u3002")
                : QStringLiteral("This model has no exportable section on the YZ plane (x = 0)."));
        } else if(m_canvas->invalidPolygon()) {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u9009\u533a\u65e0\u6548\uff0c\u8bf7\u91cd\u65b0\u5708\u51fa\u76ee\u6807\u8868\u9762\u3002")
                : QStringLiteral("The boundary is invalid. Please enclose the target surface again."));
        } else if(m_canvas->isClosed()) {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u5708\u9009\u8303\u56f4\u5185\u6ca1\u6709\u622a\u9762\u66f2\u7ebf\uff0c\u8bf7\u91cd\u65b0\u5708\u9009\u76ee\u6807\u8868\u9762\u3002")
                : QStringLiteral("No section curve lies inside this boundary. Select the target surface again."));
        } else {
            m_selectionStatus->setText(m_chinese
                ? QStringLiteral("\u8bf7\u5b8c\u6210\u5708\u9009\u540e\u5bfc\u51fa\u3002\u76ee\u6807\u9762\u4e0a\u7684\u771f\u5b9e 0 \u03bcm \u5c06\u4fdd\u7559\uff0c\u5176\u4ed6\u8868\u9762\u4e0d\u5bfc\u51fa\u3002")
                : QStringLiteral("Finish selecting a surface to enable export. True 0 \u03bcm values on that surface are retained; other surfaces are excluded."));
        }
    }
}
