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
#include <limits>
#include <utility>
#include <vector>

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
            std::vector<Eigen::Vector2d> polygon;
            polygon.reserve(m_selectionPath.size());
            for(const QPointF& screenPoint : m_selectionPath) {
                const Eigen::Vector2d sectionPoint = screenToSection(screenPoint);
                if(polygon.empty() || (sectionPoint - polygon.back()).norm() > 1.0e-9) {
                    polygon.push_back(sectionPoint);
                }
            }
            if(polygon.size() > 1 && (polygon.front() - polygon.back()).norm() <= 1.0e-9) {
                polygon.pop_back();
            }
            if(polygon.size() < 3) {
                return result;
            }

            result.enabled = true;
            result.polygon = std::move(polygon);
            result.minimum = Eigen::Vector2d::Constant(
                std::numeric_limits<double>::max());
            result.maximum = Eigen::Vector2d::Constant(
                std::numeric_limits<double>::lowest());
            for(const Eigen::Vector2d& point : result.polygon) {
                result.minimum = result.minimum.cwiseMin(point);
                result.maximum = result.maximum.cwiseMax(point);
            }
            return result;
        }

        void clearSelection()
        {
            m_selectionPath.clear();
            update();
        }

        void selectEntireProfile()
        {
            const QRectF plot = plotRectangle();
            m_selectionPath = {
                plot.topLeft(), plot.topRight(), plot.bottomRight(), plot.bottomLeft() };
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
                        for(const auto& interval : clipSegmentToSelection(
                            activeSelection, first.sectionPosition, second.sectionPosition)) {
                            painter.drawLine(
                                sectionToScreen(first.sectionPosition
                                    + interval.first * (second.sectionPosition - first.sectionPosition)),
                                sectionToScreen(first.sectionPosition
                                    + interval.second * (second.sectionPosition - first.sectionPosition)));
                        }
                    }
                }
            }

            if(m_selectionPath.size() >= 2) {
                QPolygonF polygon;
                for(const QPointF& point : m_selectionPath) {
                    polygon << point;
                }
                if(m_selectionPath.size() >= 3) {
                    painter.setBrush(QColor(55, 190, 90, 48));
                    painter.setPen(QPen(QColor(75, 235, 115), 1.5));
                    painter.drawPolygon(polygon);
                } else {
                    painter.setBrush(Qt::NoBrush);
                    painter.setPen(QPen(QColor(75, 235, 115), 1.5));
                    painter.drawPolyline(polygon);
                }
            }
            painter.setPen(QColor(185, 190, 198));
            painter.drawText(12, height() - 10,
                QStringLiteral("Horizontal: radius (mm)   Vertical: axis coordinate (mm)"));
        }

        void mousePressEvent(QMouseEvent* event) override
        {
            if(event->button() != Qt::LeftButton) {
                return;
            }
            m_selectionPath.clear();
            m_selectionPath.push_back(clampToPlot(event->pos()));
            m_dragging = true;
            update();
        }

        void mouseMoveEvent(QMouseEvent* event) override
        {
            if(!m_dragging) {
                return;
            }
            const QPointF point = clampToPlot(event->pos());
            if(m_selectionPath.empty() || screenDistance(m_selectionPath.back(), point) >= 2.0) {
                m_selectionPath.push_back(point);
                update();
            }
        }

        void mouseReleaseEvent(QMouseEvent* event) override
        {
            if(event->button() == Qt::LeftButton && m_dragging) {
                m_dragging = false;
                const QPointF point = clampToPlot(event->pos());
                if(m_selectionPath.empty()
                    || screenDistance(m_selectionPath.back(), point) >= 1.0) {
                    m_selectionPath.push_back(point);
                }
                update();
            }
        }

    private:
        static double screenDistance(const QPointF& first, const QPointF& second)
        {
            return std::hypot(first.x() - second.x(), first.y() - second.y());
        }

        static double cross2d(const Eigen::Vector2d& first, const Eigen::Vector2d& second)
        {
            return first.x() * second.y() - first.y() * second.x();
        }

        static std::vector<std::pair<double, double>> clipSegmentToSelection(
            const spraythickness::opengl::AxisymmetricProfileSelection& selection,
            const Eigen::Vector2d& first,
            const Eigen::Vector2d& second)
        {
            std::vector<std::pair<double, double>> intervals;
            if(!selection.enabled) {
                intervals.emplace_back(0.0, 1.0);
                return intervals;
            }
            if(selection.polygon.size() < 3) {
                if(selection.contains((first + second) * 0.5)) {
                    intervals.emplace_back(0.0, 1.0);
                }
                return intervals;
            }
            std::vector<double> parameters{ 0.0, 1.0 };
            const Eigen::Vector2d delta = second - first;
            for(std::size_t index = 0; index < selection.polygon.size(); ++index) {
                const Eigen::Vector2d edgeStart = selection.polygon[index];
                const Eigen::Vector2d edge = selection.polygon[
                    (index + 1) % selection.polygon.size()] - edgeStart;
                const double denominator = cross2d(delta, edge);
                if(std::abs(denominator) <= 1.0e-15) {
                    if(delta.squaredNorm() > 1.0e-24
                        && std::abs(cross2d(edgeStart - first, delta)) <= 1.0e-10) {
                        parameters.push_back(std::clamp(
                            (edgeStart - first).dot(delta) / delta.squaredNorm(), 0.0, 1.0));
                        parameters.push_back(std::clamp(
                            (edgeStart + edge - first).dot(delta) / delta.squaredNorm(),
                            0.0, 1.0));
                    }
                    continue;
                }
                const Eigen::Vector2d offset = edgeStart - first;
                const double segmentT = cross2d(offset, edge) / denominator;
                const double edgeT = cross2d(offset, delta) / denominator;
                if(segmentT >= -1.0e-12 && segmentT <= 1.0 + 1.0e-12
                    && edgeT >= -1.0e-12 && edgeT <= 1.0 + 1.0e-12) {
                    parameters.push_back(std::clamp(segmentT, 0.0, 1.0));
                }
            }
            std::sort(parameters.begin(), parameters.end());
            parameters.erase(std::unique(parameters.begin(), parameters.end(),
                [](double firstParameter, double secondParameter) {
                    return std::abs(firstParameter - secondParameter) <= 1.0e-10;
                }), parameters.end());
            for(std::size_t index = 1; index < parameters.size(); ++index) {
                const double minimumT = parameters[index - 1];
                const double maximumT = parameters[index];
                if(maximumT - minimumT <= 1.0e-12) {
                    continue;
                }
                if(selection.contains(first + 0.5 * (minimumT + maximumT) * delta)) {
                    intervals.emplace_back(minimumT, maximumT);
                }
            }
            return intervals;
        }

        QRectF plotRectangle() const
        {
            return QRectF(48.0, 20.0,
                std::max(1, width() - 68), std::max(1, height() - 58));
        }

        Eigen::Vector2d displayMinimum() const
        {
            const Eigen::Vector2d span = (m_slice.maximum - m_slice.minimum).cwiseMax(
                Eigen::Vector2d::Constant(1.0e-9));
            return m_slice.minimum - 0.08 * span;
        }

        Eigen::Vector2d displayMaximum() const
        {
            const Eigen::Vector2d span = (m_slice.maximum - m_slice.minimum).cwiseMax(
                Eigen::Vector2d::Constant(1.0e-9));
            return m_slice.maximum + 0.08 * span;
        }

        QPointF clampToPlot(const QPointF& point) const
        {
            const QRectF plot = plotRectangle();
            return QPointF(
                std::clamp(point.x(), plot.left(), plot.right()),
                std::clamp(point.y(), plot.top(), plot.bottom()));
        }

        QPointF sectionToScreen(const Eigen::Vector2d& point) const
        {
            const QRectF plot = plotRectangle();
            const Eigen::Vector2d minimum = displayMinimum();
            const Eigen::Vector2d span = (displayMaximum() - minimum).cwiseMax(
                Eigen::Vector2d::Constant(1.0e-9));
            const double x = (point.x() - minimum.x()) / span.x();
            const double y = (point.y() - minimum.y()) / span.y();
            return QPointF(plot.left() + x * plot.width(),
                plot.bottom() - y * plot.height());
        }

        Eigen::Vector2d screenToSection(const QPointF& point) const
        {
            const QRectF plot = plotRectangle();
            const Eigen::Vector2d minimum = displayMinimum();
            const Eigen::Vector2d maximum = displayMaximum();
            const Eigen::Vector2d span = maximum - minimum;
            const double x = std::clamp((point.x() - plot.left()) / plot.width(), 0.0, 1.0);
            const double y = std::clamp((plot.bottom() - point.y()) / plot.height(), 0.0, 1.0);
            const Eigen::Vector2d sectionPoint(
                minimum.x() + x * span.x(),
                minimum.y() + y * span.y());
            const Eigen::Vector2d sectionMinimum = m_slice.minimum.cwiseMin(m_slice.maximum);
            const Eigen::Vector2d sectionMaximum = m_slice.minimum.cwiseMax(m_slice.maximum);
            return sectionPoint.cwiseMax(sectionMinimum).cwiseMin(sectionMaximum);
        }

        const spraythickness::opengl::AxisymmetricProfileSlice& m_slice;
        std::vector<QPointF> m_selectionPath;
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
            QStringLiteral("Draw a closed freeform boundary over the profile region. "
                "The selected boundary is predicted; outside it, final thickness is set to 0."), this);
        description->setWordWrap(true);
        layout->addWidget(description);
        m_canvas = new ProfileCanvas(slice, this);
        layout->addWidget(m_canvas, 1);
        auto* buttons = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        QPushButton* selectAllButton = buttons->addButton(
            QStringLiteral("Select entire profile"), QDialogButtonBox::ActionRole);
        QPushButton* clearButton = buttons->addButton(
            QStringLiteral("Clear selection"), QDialogButtonBox::ResetRole);
        connect(selectAllButton, &QPushButton::clicked,
            m_canvas, &ProfileCanvas::selectEntireProfile);
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
