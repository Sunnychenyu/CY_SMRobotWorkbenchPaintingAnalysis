#include "ThicknessLegendWidget.h"
#include "CoatingAnalysisLanguage.h"

#include <VisualizationSDK/ScalarColorMap.h>

#include <QDialog>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    ThicknessLegendWidget::ThicknessLegendWidget(QWidget* parent)
        : QWidget(parent)
    {
        setFixedWidth(118);
        resize(width(), 360);
        setAutoFillBackground(false);
        setAttribute(Qt::WA_OpaquePaintEvent, true);
        setToolTip(QStringLiteral("Double-click the color bar to edit its range."));
    }

    void ThicknessLegendWidget::setRange(double minimumMicrometers, double maximumMicrometers)
    {
        m_minimumMicrometers = minimumMicrometers;
        m_maximumMicrometers = maximumMicrometers;
        update();
    }

    void ThicknessLegendWidget::setRelativeErrorMode(bool enabled)
    {
        m_relativeErrorMode = enabled;
        if(enabled && m_rangeEditor != nullptr) {
            m_rangeEditor->close();
        }
        update();
    }

    void ThicknessLegendWidget::setEditingEnabled(bool enabled)
    {
        m_editingEnabled = enabled;
        if(!enabled && m_rangeEditor != nullptr) {
            m_rangeEditor->close();
        }
    }

    void ThicknessLegendWidget::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        setToolTip(coatingAnalysisTranslate(
            m_languageCode,
            QStringLiteral("Double-click the color bar to edit its range.")));
        if(m_rangeEditor != nullptr) {
            m_rangeEditor->close();
        }
        update();
    }

    void ThicknessLegendWidget::mouseDoubleClickEvent(QMouseEvent* event)
    {
        const QRect barRect(18, 58, 27, qMax(80, height() - 88));
        if(event->button() == Qt::LeftButton
            && barRect.contains(event->pos())
            && m_editingEnabled
            && !m_relativeErrorMode) {
            showRangeEditor();
            event->accept();
            return;
        }
        QWidget::mouseDoubleClickEvent(event);
    }

    void ThicknessLegendWidget::showRangeEditor()
    {
        if(m_rangeEditor != nullptr) {
            m_rangeEditor->show();
            m_rangeEditor->raise();
            m_rangeEditor->activateWindow();
            return;
        }

        auto* dialog = new QDialog(this, Qt::Tool);
        m_rangeEditor = dialog;
        dialog->setAttribute(Qt::WA_DeleteOnClose, true);
        dialog->setWindowTitle(coatingAnalysisTranslate(
            m_languageCode, QStringLiteral("Thickness display range")));

        auto* minimumSpin = new QDoubleSpinBox(dialog);
        auto* maximumSpin = new QDoubleSpinBox(dialog);
        for(QDoubleSpinBox* spin : { minimumSpin, maximumSpin }) {
            spin->setDecimals(6);
            spin->setRange(0.0, 1.0e12);
            spin->setSingleStep(0.1);
            spin->setSuffix(QStringLiteral(" \u03bcm"));
        }
        minimumSpin->setValue(qMax(0.0, m_minimumMicrometers));
        maximumSpin->setValue(qMax(0.0, m_maximumMicrometers));

        auto* form = new QFormLayout;
        form->addRow(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Minimum")),
            minimumSpin);
        form->addRow(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Maximum")),
            maximumSpin);

        auto* errorLabel = new QLabel(dialog);
        errorLabel->setStyleSheet(QStringLiteral("color: #c43c3c;"));
        auto* applyButton = new QPushButton(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Apply")), dialog);
        auto* automaticButton = new QPushButton(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Restore automatic range")),
            dialog);
        auto* closeButton = new QPushButton(
            coatingAnalysisTranslate(m_languageCode, QStringLiteral("Close")), dialog);

        const auto updateValidation = [minimumSpin, maximumSpin, errorLabel, applyButton,
            languageCode = m_languageCode]() {
            const bool valid = minimumSpin->value() < maximumSpin->value();
            applyButton->setEnabled(valid);
            errorLabel->setText(valid
                ? QString()
                : coatingAnalysisTranslate(
                    languageCode,
                    QStringLiteral("Minimum must be less than maximum.")));
        };
        connect(minimumSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            dialog,
            [updateValidation](double) { updateValidation(); });
        connect(maximumSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            dialog,
            [updateValidation](double) { updateValidation(); });
        connect(applyButton, &QPushButton::clicked, dialog,
            [this, dialog, minimumSpin, maximumSpin]() {
                emit rangeChangeRequested(minimumSpin->value(), maximumSpin->value());
                dialog->close();
            });
        connect(automaticButton, &QPushButton::clicked, dialog, [this, dialog]() {
            emit automaticRangeRequested();
            dialog->close();
        });
        connect(closeButton, &QPushButton::clicked, dialog, &QDialog::close);

        auto* buttons = new QHBoxLayout;
        buttons->addWidget(automaticButton);
        buttons->addStretch(1);
        buttons->addWidget(applyButton);
        buttons->addWidget(closeButton);

        auto* layout = new QVBoxLayout(dialog);
        layout->addLayout(form);
        layout->addWidget(errorLabel);
        layout->addLayout(buttons);
        updateValidation();
        dialog->adjustSize();
        dialog->move(mapToGlobal(QPoint(-dialog->width() - 8, 58)));
        dialog->show();
    }

    void ThicknessLegendWidget::paintEvent(QPaintEvent* event)
    {
        QWidget::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        QColor background = palette().color(QPalette::Window);
        background.setAlpha(255);
        painter.fillRect(rect(), background);
        painter.setPen(palette().color(QPalette::Mid));
        painter.setBrush(background);
        painter.drawRoundedRect(rect().adjusted(1, 1, -2, -2), 6, 6);

        painter.setPen(palette().color(QPalette::Text));
        painter.drawText(QRect(8, 8, width() - 16, 38),
            Qt::AlignHCenter | Qt::AlignVCenter,
            m_relativeErrorMode
                ? coatingAnalysisTranslate(
                    m_languageCode, QStringLiteral("Relative error\n(%)"))
                : coatingAnalysisTranslate(
                    m_languageCode, QStringLiteral("Thickness\n(\u03bcm)")));

        const QRect barRect(18, 58, 27, qMax(80, height() - 88));
        const smrobot::visualization::ScalarColorMap colorMap =
            m_relativeErrorMode
                ? smrobot::visualization::ScalarColorMap({
                    { 0.000, Eigen::Vector3f(0.0f, 0.0f, 0.5f) },
                    { 0.125, Eigen::Vector3f(0.0f, 0.0f, 1.0f) },
                    { 0.250, Eigen::Vector3f(0.0f, 0.5f, 1.0f) },
                    { 0.375, Eigen::Vector3f(0.0f, 1.0f, 1.0f) },
                    { 0.500, Eigen::Vector3f(0.5f, 1.0f, 0.5f) },
                    { 0.625, Eigen::Vector3f(1.0f, 1.0f, 0.0f) },
                    { 0.750, Eigen::Vector3f(1.0f, 0.5f, 0.0f) },
                    { 0.875, Eigen::Vector3f(1.0f, 0.0f, 0.0f) },
                    { 1.000, Eigen::Vector3f(0.5f, 0.0f, 0.0f) }
                })
                : smrobot::visualization::ScalarColorMap::heatMap();

        for(int y = barRect.top(); y <= barRect.bottom(); ++y) {
            const double normalized = barRect.height() > 1
                ? static_cast<double>(barRect.bottom() - y) /
                    static_cast<double>(barRect.height() - 1)
                : 0.0;
            const Eigen::Vector3f color = colorMap.sampleNormalized(normalized)
                .cwiseMax(0.0f)
                .cwiseMin(1.0f);
            const QColor displayColor(
                qRound(color.x() * 255.0f),
                qRound(color.y() * 255.0f),
                qRound(color.z() * 255.0f),
                255);
            painter.fillRect(
                QRect(barRect.left(), y, barRect.width(), 1),
                displayColor);
        }
        painter.setBrush(Qt::NoBrush);
        painter.setPen(palette().color(QPalette::Mid));
        painter.drawRect(barRect.adjusted(0, 0, -1, -1));

        painter.setPen(palette().color(QPalette::Text));
        const double thicknessSpan = m_maximumMicrometers - m_minimumMicrometers;
        for(const smrobot::visualization::ScalarColorStop& stop : colorMap.stops()) {
            const int tickY = barRect.bottom() -
                qRound(stop.position * static_cast<double>(barRect.height() - 1));
            const double thickness = m_minimumMicrometers + stop.position * thicknessSpan;
            painter.drawLine(barRect.right() + 2, tickY, barRect.right() + 9, tickY);
            painter.drawText(
                QRect(barRect.right() + 14, tickY - 11,
                    width() - barRect.right() - 18, 22),
                Qt::AlignLeft | Qt::AlignVCenter,
                QString::number(thickness, 'f', 1));
        }
    }
}
