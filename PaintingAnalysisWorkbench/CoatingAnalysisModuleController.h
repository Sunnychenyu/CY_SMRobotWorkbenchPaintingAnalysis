#pragma once

#include "CoatingAnalysisSession.h"
#include "PaintingAnalysisMeshAdapter.h"

#include <QHash>
#include <QObject>
#include <QPoint>
#include <QString>

#include <Eigen/Core>

#include <cstdint>
#include <chrono>
#include <memory>
#include <vector>

namespace spraytrajectory
{
    struct SprayPathPoint;
}

namespace robot_qt_viewer
{
    class CoatingAnalysisPanel;
    class CoatingAnalysisTreePanel;
    class CoatingAnalysisInfoPanel;
    class CoatingAnalysisVisibilityBar;
    class RobotQtViewerDocumentContext;
    class ThicknessPredictionJobController;
    struct RobotQtViewerEvent;

    class CoatingAnalysisModuleController : public QObject
    {
        Q_OBJECT

    public:
        CoatingAnalysisModuleController(
            CoatingAnalysisPanel& panel,
            CoatingAnalysisTreePanel& treePanel,
            CoatingAnalysisInfoPanel& infoPanel,
            CoatingAnalysisVisibilityBar& visibilityBar,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);
        ~CoatingAnalysisModuleController() override;

        void activate();
        void deactivate();
        void handleEvent(const RobotQtViewerEvent& event);
        void handleSurfaceScalarHover(
            const QString& objectId,
            double valueMeters,
            double worldX,
            double worldY,
            double worldZ,
            const QPoint& viewportPosition,
            bool hit);
        void handleRotationSurfacePicked(
            const QString& objectId,
            std::uint32_t triangleIndex,
            double hitX,
            double hitY,
            double hitZ,
            double normalX,
            double normalY,
            double normalZ);

        // Waypoint inspection source for the virtualized tree and details dialog.
        const std::vector<spraytrajectory::SprayPathPoint>& waypoints() const;
        const spraytrajectory::SprayPathPoint& waypointAt(std::size_t index) const;
        double waypointDurationAt(std::size_t index) const;

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
        struct AxisymmetricProfileState;
        bool loadModel(const QString& path, double scaleToMeters);
        void openModelFromDialog();
        bool loadTrajectory(const QString& path);
        void openTrajectoryFromDialog();
        void predictThickness();
        void previewLocalInputs();
        void selectAxisymmetricProfileRegion();
        void cancelPrediction();
        void handlePredictionProgress(double progress, const QString& message);
        void handlePredictionFinished(const spraythickness::ThicknessPredictionResult& prediction);
        void handlePredictionFailed(const QString& message);
        void setShowModel(bool enabled);
        void setShowSprayPoints(bool enabled);
        void setShowThickness(bool enabled);
        void setThicknessPickEnabled(bool enabled);
        void setLocalDebugVisibility(
            bool cylindricalSurface,
            bool rotationAxis,
            bool localSector,
            bool sprayPoints);
        void selectWorkpiece(const QString& objectId);
        void selectWorkpieceFromTree(const QString& objectId);
        void ensureWorkpieceSelection();
        void updateSelectedModelInfo();
        void clearModelVisibilityOverrides();
        void applyModelVisibilityOverrides();
        void clearSession();
        void applyVisualizationState();
        void submitTrajectoryPreview();
        void updateTrajectoryPreviewVisibility();
        void applyOverlayAfterReload();
        void refreshViewModel();
        void publishStateChanged();
        bool ensurePreviewWorkpieceLoaded();
        bool hasEffectiveRotationAxis() const;
        Eigen::Vector3d effectiveRotationAxisOrigin() const;
        Eigen::Vector3d effectiveRotationAxisDirection() const;
        QString rotationAxisSource() const;
        void applyLocalDebugVisibility();
        bool ensureAxisymmetricProfileSlice();
        void clearAxisymmetricProfileSelection();
        bool rebuildAxisymmetricProfileReduction();
        void updateAxisymmetricProfileDebugState();

        void handleWaypointInfoRequested(int index);
        void handleModelVisibilityToggleRequested(const QString& objectId);
        void handleModelSetAsWorkpiece(const QString& objectId);
        void handleThicknessClearRequested();

        CoatingAnalysisPanel& m_panel;
        CoatingAnalysisTreePanel& m_treePanel;
        CoatingAnalysisInfoPanel& m_infoPanel;
        CoatingAnalysisVisibilityBar& m_visibilityBar;
        RobotQtViewerDocumentContext& m_context;
        CoatingAnalysisSession m_session;
        std::unique_ptr<ThicknessPredictionJobController> m_predictionJob;
        QString m_status = QStringLiteral("Load a model and trajectory to begin.");
        QString m_predictionObjectId;
        double m_predictionProgress = 0.0;
        bool m_active = false;
        bool m_hasCurrentThickness = false;
        double m_currentThicknessMeters = 0.0;
        bool m_hasRotationAxis = false;
        Eigen::Vector3d m_rotationAxisOrigin = Eigen::Vector3d::Zero();
        Eigen::Vector3d m_rotationAxisDirection = Eigen::Vector3d::UnitZ();
        sprayworkpiece::WorkpieceModel m_rotationPreviewWorkpiece;
        std::vector<std::size_t> m_rotationSurfaceTriangleIndices;
        std::size_t m_rotationSeedTriangleIndex = 0;
        double m_rotationFitElapsedMilliseconds = 0.0;
        bool m_hasLocalPreview = false;
        QString m_localPreviewDetails;
        std::unique_ptr<AxisymmetricProfileState> m_axisymmetricProfile;
        bool m_showCylindricalSurface = true;
        bool m_showRotationAxis = true;
        bool m_showLocalSector = true;
        bool m_showLocalSprayPoints = true;
        std::chrono::steady_clock::time_point m_predictionStartedAt{};
        bool m_predictionTimerActive = false;
        std::vector<QString> m_modelVisibilityOverrideIds;
        QHash<QString, bool> m_modelVisibility;
    };
}
