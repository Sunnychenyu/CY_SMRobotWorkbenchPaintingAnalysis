#pragma once

#include "CoatingAnalysisSession.h"

#include <QObject>
#include <QPoint>
#include <QString>

namespace robot_qt_viewer
{
    class CoatingAnalysisPanel;
    class RobotQtViewerDocumentContext;
    struct RobotQtViewerEvent;

    class CoatingAnalysisModuleController : public QObject
    {
        Q_OBJECT

    public:
        CoatingAnalysisModuleController(
            CoatingAnalysisPanel& panel,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);

        void activate();
        void deactivate();
        void handleEvent(const RobotQtViewerEvent& event);
        void handleSurfaceScalarHover(
            const QString& objectId,
            double valueMeters,
            const QPoint& viewportPosition,
            bool hit);

    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
        void thicknessToolTipRequested(
            const QString& text,
            const QPoint& viewportPosition,
            bool visible);
        void thicknessLegendChanged(
            bool visible,
            double minimumMicrometers,
            double maximumMicrometers);

    private:
        void openModel();
        void predictThickness();
        void setShowThickness(bool enabled);
        void clearSession();
        void applyOverlayAfterReload();
        void refreshViewModel();
        void publishStateChanged();

        CoatingAnalysisPanel& m_panel;
        RobotQtViewerDocumentContext& m_context;
        CoatingAnalysisSession m_session;
        QString m_status = QStringLiteral("Open a mesh model to begin.");
        bool m_active = false;
        bool m_hasCurrentThickness = false;
        double m_currentThicknessMeters = 0.0;
    };
}
