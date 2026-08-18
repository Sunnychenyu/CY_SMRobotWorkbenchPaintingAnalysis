#include "PaintingAnalysisDialogService.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QPushButton>

namespace robot_qt_viewer
{
    QString PaintingAnalysisDialogService::selectModelFile(QWidget* parent)
    {
        return QFileDialog::getOpenFileName(
            parent,
            QStringLiteral("Open Coating Analysis Model"),
            QString(),
            QStringLiteral("Mesh Models (*.stl *.obj *.dae *.ply);;All Files (*.*)"));
    }

    bool PaintingAnalysisDialogService::selectModelUnitScale(
        QWidget* parent,
        const QString& modelPath,
        double& scaleToMeters)
    {
        QMessageBox dialog(parent);
        dialog.setWindowTitle(QStringLiteral("Model Unit"));
        dialog.setIcon(QMessageBox::Question);
        dialog.setText(QStringLiteral("Select the coordinate unit of %1.")
            .arg(QFileInfo(modelPath).fileName()));
        dialog.setInformativeText(QStringLiteral(
            "STL and OBJ files usually do not store unit metadata. "
            "Millimeters converts model coordinates to meters; meters keeps them unchanged."));

        QPushButton* millimeterButton = dialog.addButton(
            QStringLiteral("Millimeters (mm)"),
            QMessageBox::AcceptRole);
        QPushButton* meterButton = dialog.addButton(
            QStringLiteral("Meters (m)"),
            QMessageBox::AcceptRole);
        dialog.addButton(QMessageBox::Cancel);
        dialog.setDefaultButton(millimeterButton);
        dialog.exec();

        if(dialog.clickedButton() == millimeterButton) {
            scaleToMeters = 0.001;
            return true;
        }
        if(dialog.clickedButton() == meterButton) {
            scaleToMeters = 1.0;
            return true;
        }
        return false;
    }

    QString PaintingAnalysisDialogService::selectTrajectoryFile(QWidget* parent)
    {
        return QFileDialog::getOpenFileName(
            parent,
            QStringLiteral("Open Spray Trajectory"),
            QString(),
            QStringLiteral("Legacy Matrix Trajectory (*.txt);;All Files (*.*)"));
    }
}
