#pragma once

#include "CoatingAnalysisViewModel.h"

#include <SprayTrajectoryCore/SprayTrajectory.h>

#include <QAbstractItemModel>
#include <QHash>

#include <vector>

namespace robot_qt_viewer
{
    enum class CoatingAnalysisNodeKind
    {
        Root,
        Trajectory,
        TrajectoryInfo,
        Waypoints,
        Waypoint,
        Models,
        Model,
        Thickness,
        ThicknessInfo
    };

    // Analysis tree backing the coating workbench left panel. Four top-level
    // branches: Trajectory, Waypoints, Models, Thickness. The waypoint branch is
    // virtualized (canFetchMore / fetchMore) so a large trajectory never floods
    // the view.
    class CoatingAnalysisTreeModel : public QAbstractItemModel
    {
        Q_OBJECT

    public:
        explicit CoatingAnalysisTreeModel(QObject* parent = nullptr);
        void setLanguageCode(const QString& languageCode);

        void setTrajectory(
            const QString& name,
            const CoatingAnalysisTrajectoryInfo& info);
        void setWorkpieces(
            const QVector<CoatingAnalysisWorkpieceItem>& workpieces,
            const QString& selectedWorkpieceId);
        void setModelVisibility(const QHash<QString, bool>& visibility);
        void setThickness(
            bool hasThickness,
            const spraythickness::ThicknessMetrics& metrics);
        // Not owned: points to the controller's cached waypoint vector. The
        // controller must refresh it after every trajectory load.
        void setWaypoints(const std::vector<spraytrajectory::SprayPathPoint>* waypoints);

        CoatingAnalysisNodeKind nodeKind(const QModelIndex& index) const;
        int waypointIndex(const QModelIndex& index) const;
        QString workpieceId(const QModelIndex& index) const;

        // QAbstractItemModel
        QModelIndex index(int row, int column, const QModelIndex& parent) const override;
        QModelIndex parent(const QModelIndex& index) const override;
        int rowCount(const QModelIndex& parent) const override;
        int columnCount(const QModelIndex& parent) const override;
        QVariant data(const QModelIndex& index, int role) const override;
        bool hasChildren(const QModelIndex& parent) const override;
        bool canFetchMore(const QModelIndex& parent) const override;
        void fetchMore(const QModelIndex& parent) override;

    private:
        enum TopLevelRow
        {
            TrajectoryRow = 0,
            WaypointsRow,
            ModelsRow,
            ThicknessRow,
            TopLevelRowCount
        };

        static constexpr int kWaypointBatchSize = 500;
        static constexpr quintptr kIdBase = 1000000;

        bool hasTrajectory() const { return !m_trajectoryName.isEmpty(); }
        bool hasWaypoints() const { return m_waypoints != nullptr && !m_waypoints->empty(); }
        bool hasModels() const { return !m_workpieces.isEmpty(); }

        QString waypointText(int index) const;
        QString trajectoryInfoText(int row) const;
        QString thicknessInfoText(int row) const;

        QString m_trajectoryName;
        CoatingAnalysisTrajectoryInfo m_trajectoryInfo;
        QVector<CoatingAnalysisWorkpieceItem> m_workpieces;
        QString m_selectedWorkpieceId;
        QHash<QString, bool> m_modelVisibility;
        bool m_hasThickness = false;
        spraythickness::ThicknessMetrics m_thicknessMetrics;
        const std::vector<spraytrajectory::SprayPathPoint>* m_waypoints = nullptr;
        int m_loadedWaypointCount = 0;
        QString m_languageCode{ QStringLiteral("en") };
    };
}
