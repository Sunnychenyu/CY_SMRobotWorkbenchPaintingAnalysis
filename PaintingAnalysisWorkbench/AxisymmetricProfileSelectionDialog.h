#pragma once

#include <SprayThicknessPredictionOpenGL/AxisymmetricProfileReduction.h>

#include <QDialog>

class QWidget;

namespace robot_qt_viewer
{
    class AxisymmetricProfileSelectionDialog : public QDialog
    {
        Q_OBJECT

    public:
        explicit AxisymmetricProfileSelectionDialog(
            const spraythickness::opengl::AxisymmetricProfileSlice& slice,
            QWidget* parent = nullptr);

        spraythickness::opengl::AxisymmetricProfileSelection selection() const;

    private:
        class ProfileCanvas;

        ProfileCanvas* m_canvas = nullptr;
    };
}
