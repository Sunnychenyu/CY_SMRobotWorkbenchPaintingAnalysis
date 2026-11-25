#pragma once

#include "ThicknessPickSampling.h"

#include <QDialog>

class QDoubleSpinBox;
class QComboBox;
class QLabel;
class QPushButton;

namespace robot_qt_viewer
{
    class ThicknessPickSectionDialog : public QDialog
    {
    public:
        explicit ThicknessPickSectionDialog(
            const std::vector<ThicknessPickSectionSegment>& segments,
            QWidget* parent = nullptr);

        std::vector<Eigen::Vector2d> selectionPolygon() const;
        double annotationSpacingMeters() const;
        ThicknessSectionSchematicOptions schematicOptions() const;
        const ThicknessPickReportSection& selectedSection() const;
        void setLanguageCode(const QString& languageCode);
        void accept() override;

    private:
        class SectionCanvas;
        void updateSelectionStatus();

        SectionCanvas* m_canvas = nullptr;
        QLabel* m_description = nullptr;
        QLabel* m_spacingLabel = nullptr;
        QLabel* m_multiplierLabel = nullptr;
        QLabel* m_multiplierExplanation = nullptr;
        QComboBox* m_multiplierMode = nullptr;
        QDoubleSpinBox* m_multiplier = nullptr;
        ThicknessPickReportSection m_selectedSection;
        QLabel* m_selectionStatus = nullptr;
        QDoubleSpinBox* m_spacing = nullptr;
        QPushButton* m_finishButton = nullptr;
        QPushButton* m_fitButton = nullptr;
        QPushButton* m_clearButton = nullptr;
        QPushButton* m_exportButton = nullptr;
        QPushButton* m_cancelButton = nullptr;
        bool m_chinese = false;
    };
}
