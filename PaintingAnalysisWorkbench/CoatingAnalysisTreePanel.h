#pragma once

#include "CoatingAnalysisViewModel.h"

#include <QWidget>

#include <vector>

class QModelIndex;
class QTreeView;

namespace spraytrajectory
{
    struct SprayPathPoint;
}

namespace robot_qt_viewer
{
    class CoatingAnalysisTreeModel;

    // Top half of the left coating panel: the analysis tree (Trajectory /
    // Waypoints / Models / Thickness) with a right-click context menu.
    class CoatingAnalysisTreePanel : public QWidget
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisTreePanel(QWidget* parent = nullptr);
        void setLanguageCode(const QString& languageCode);

        void applyTreeView(const CoatingAnalysisTreeView& view);
        // Not owned: forwards to the model so the waypoint branch can be
        // virtualized against the controller's cached waypoint vector.
        void setWaypoints(const std::vector<spraytrajectory::SprayPathPoint>* waypoints);

        QTreeView* treeView() const { return m_treeView; }

    signals:
        void waypointInfoRequested(int index);
        void modelVisibilityToggleRequested(const QString& objectId);
        void modelSetAsWorkpiece(const QString& objectId);
        void thicknessClearRequested();

    private:
        void showContextMenu(const QPoint& position);

        CoatingAnalysisTreeModel* m_model = nullptr;
        QTreeView* m_treeView = nullptr;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
