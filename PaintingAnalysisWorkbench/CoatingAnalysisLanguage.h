#pragma once

#include <QString>

namespace robot_qt_viewer
{
    // Converts coating-analysis UI text without introducing a dependency on
    // the application's global translation resources.
    QString coatingAnalysisTranslate(
        const QString& languageCode,
        const QString& text);
}
