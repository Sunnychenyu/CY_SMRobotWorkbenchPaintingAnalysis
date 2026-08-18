#include "AxisymmetricProfileSelectionDialog.h"

#include <QDialogButtonBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    class AxisymmetricProfileSelectionDialog::ProfileCanvas : public QWidget
    {
    public:
        explicit ProfileCanvas(
            const spraythickness::opengl::AxisymmetricProfileSlice& slice,
            QWidget* parent)
            : QWidget(parent)
            , m_slice(slice)
        {
            setMinimumSize(620, 420);
            setMouseTracking(true);
        }

        spraythickness::opengl::AxisymmetricProfileSelection selection() const
        {
            spraythickness::opengl::AxisymmetricProfileSelection result;
            if(m_selectionRectangle.isNull() || m_selectionRectangle.width() < 2
                || m_selectionRectangle.height() < 2) {
                return result;
            }
            result.enabled = true;
            const QRectF rectangle = m_selectionRectangle.normalized();
            result.minimum = screenToSection(rectangle.bottomLeft());
            result.maximum = screenToSection(rectangle.topRight());
            return result;
        }

        void clearSelection()
        {
            m_selectionRectangle = QRectF();
            update();
        }

    protected:
        void paintEvent(QPaintEvent*) override
        {
            QPainter painter(this);
            painter.fillRect(rect(), QColor(32, 35, 40));
            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.setPen(QPen(QColor(95, 100, 108), 1));
            painter.drawRect(plotRectangle());

            painter.setPen(QPen(QColor(105, 195, 255), 1));
            const QPointF axisBottom = sectionToScreen(
                Eigen::Vector2d(0.0, m_slice.minimum.y()));
            const QPointF axisTop = sectionToScreen(
                Eigen::Vector2d(0.0, m_slice.maximum.y()));
            painter.drawLine(axisBottom, axisTop);

            painter.setPen(QPen(QColor(220, 225, 230), 1.4));
            for(const auto& contour : m_slice.contours) {
                if(contour.points.size() < 2) {
                    continue;
                }
                QPainterPath path;
                path.moveTo(sectionToScreen(contour.points.front().sectionPosition));
                for(std::size_t index = 1; index < contour.points.size(); ++index) {
                    path.lineTo(sectionToScreen(contour.points[index].sectionPosition));
                }
                if(contour.closed) {
                    path.closeSubpath();
                }
                painter.drawPath(path);
            }
            const auto activeSelection = selection();
            if(activeSelection.enabled) {
                painter.setPen(QPen(QColor(75, 235, 115), 2.4));
                for(const auto& contour : m_slice.contours) {
                    if(contour.points.size() < 2) {
                        continue;
                    }
                    const std::size_t segmentCount = contour.closed
                        ? contour.points.size() : contour.points.size() - 1;
                    for(std::size_t index = 0; index < segmentCount; ++index) {
                        const auto& first = contour.points[index];
                    const auto& second = contour.points[
                        (index + 1) % contour.points.size()];
                        double minimumT = 0.0;
                        double maximumT = 1.0;
                        if(!clipSegmentToSelection(
                            activeSelection,
                            first.sectionPosition,
                            second.sectionPosition,
                            minimumT,
                            maximumT)) {
                            continue;
                        }
                        painter.drawLine(
                            sectionToScreen(first.sectionPosition
                                + minimumT * (second.sectionPosition - first.sectionPosition)),
                            sectionToScreen(first.sectionPosition
                                + maximumT * (second.sectionPosition - first.sectionPosition)));
                    }
                }
            }
            if(!m_selectionRectangle.isNull()) {
                painter.fillRect(m_selectionRectangle.normalized(), QColor(55, 190, 90, 48));
                painter.setPen(QPen(QColor(75, 235, 115), 1.5));
                painter.drawRect(m_selectionRectangle.normalized());
            }
            painter.setPen(QColor(185, 190, 198));
            painter.drawText(12, height() - 10,
                QStringLiteral("Horizontal: radius (mm)   Vertical: axis coordinate (mm)"));
        }

        void mousePressEvent(QMouseEvent* event) override
        {
            if(event->button() != Qt::LeftButton || !plotRectangle().contains(event->pos())) {
                return;
            }
            m_dragStart = event->pos();
            m_selectionRectangle = QRectF(m_dragStart, m_dragStart);
            m_dragging = true;
            update();
        }

        void mouseMoveEvent(QMouseEvent* event) override
        {
            if(!m_dragging) {
                return;
            }
            m_selectionRectangle = QRectF(m_dragStart, event->pos()).normalized()
                .intersected(plotRectangle());
            update();
        }

        void mouseReleaseEvent(QMouseEvent* event) override
        {
            if(event->button() == Qt::LeftButton && m_dragging) {
                m_dragging = false;
                m_selectionRectangle = QRectF(m_dragStart, event->pos()).normalized()
                    .intersected(plotRectangle());
                update();
            }
        }

    private:
        static bool clipSegmentToSelection(
            const spraythickness::opengl::AxisymmetricProfileSelection& selection,
            const Eigen::Vector2d& first,
            const Eigen::Vector2d& second,
            double& minimumT,
            double& maximumT)
        {
            minimumT = 0.0;
            maximumT = 1.0;
            if(!selection.enabled) {
                return true;
            }
            const Eigen::Vector2d minimum = selection.minimum.cwiseMin(selection.maximum);
            const Eigen::Vector2d maximum = selection.minimum.cwiseMax(selection.maximum);
            const Eigen::Vector2d delta = second - first;
            for(int dimension = 0; dimension < 2; ++dimension) {
                if(std::abs(delta[dimension]) <= 1.0e-15) {
                    if(first[dimension] < minimum[dimension]
                        || first[dimension] > maximum[dimension]) {
                        return false;
                    }
                    continue;
                }
                double entry = (minimum[dimension] - first[dimension]) / delta[dimension];
                double exit = (maximum[dimension] - first[dimension]) / delta[dimension];
                if(entry > exit) {
                    std::swap(entry, exit);
                }
                minimumT = std::max(minimumT, entry);
                maximumT = std::min(maximumT, exit);
                if(minimumT > maximumT) {
                    return false;
                }
            }
            minimumT = std::clamp(minimumT, 0.0, 1.0);
            maximumT = std::clamp(maximumT, 0.0, 1.0);
            return minimumT <= maximumT;
        }

        QRectF plotRectangle() const
        {
            return QRectF(48.0, 20.0,
                std::max(1, width() - 68), std::max(1, height() - 58));
        }

        QPointF sectionToScreen(const Eigen::Vector2d& point) const
        {
            const QRectF plot = plotRectangle();
            const Eigen::Vector2d span = (m_slice.maximum - m_slice.minimum).cwiseMax(
                Eigen::Vector2d::Constant(1.0e-9));
            const double x = (point.x() - m_slice.minimum.x()) / span.x();
            const double y = (point.y() - m_slice.minimum.y()) / span.y();
            return QPointF(plot.left() + x * plot.width(),
                plot.bottom() - y * plot.height());
        }

        Eigen::Vector2d screenToSection(const QPointF& point) const
        {
            const QRectF plot = plotRectangle();
            const Eigen::Vector2d span = m_slice.maximum - m_slice.minimum;
            const double x = std::clamp((point.x() - plot.left()) / plot.width(), 0.0, 1.0);
            const double y = std::clamp((plot.bottom() - point.y()) / plot.height(), 0.0, 1.0);
            return Eigen::Vector2d(
                m_slice.minimum.x() + x * span.x(),
                m_slice.minimum.y() + y * span.y());
        }

        const spraythickness::opengl::AxisymmetricProfileSlice& m_slice;
        QPointF m_dragStart;
        QRectF m_selectionRectangle;
        bool m_dragging{ false };
    };

    AxisymmetricProfileSelectionDialog::AxisymmetricProfileSelectionDialog(
        const spraythickness::opengl::AxisymmetricProfileSlice& slice,
        QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(QStringLiteral("Select Profile Prediction Region"));
        resize(720, 560);
        auto* layout = new QVBoxLayout(this);
        auto* description = new QLabel(
            QStringLiteral("Drag one rectangle over the profile region to predict. "
                "Outside the rectangle, final thickness is set to 0."), this);
        description->setWordWrap(true);
        layout->addWidget(description);
        m_canvas = new ProfileCanvas(slice, this);
        layout->addWidget(m_canvas, 1);
        auto* buttons = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        QPushButton* clearButton = buttons->addButton(
            QStringLiteral("Clear selection"), QDialogButtonBox::ResetRole);
        connect(clearButton, &QPushButton::clicked, m_canvas, &ProfileCanvas::clearSelection);
        connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        layout->addWidget(buttons);
    }

    spraythickness::opengl::AxisymmetricProfileSelection
    AxisymmetricProfileSelectionDialog::selection() const
    {
        return m_canvas != nullptr
            ? m_canvas->selection()
            : spraythickness::opengl::AxisymmetricProfileSelection();
    }
}
