#pragma once

#include <QString>

class QWidget;

namespace robot_qt_viewer
{
    class PaintingAnalysisDialogService
    {
    public:
        static QString selectModelFile(QWidget* parent);
    };
}
