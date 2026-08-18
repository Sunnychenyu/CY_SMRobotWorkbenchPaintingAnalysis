#include "CoatingAnalysisTreeModel.h"

#include <QColor>
#include <QFont>

#include <algorithm>
#include <cmath>

namespace robot_qt_viewer
{
    namespace
    {
        QString formatKeyValue(const QString& key, const QString& value)
        {
            return key + QStringLiteral(" : ") + value;
        }
    }

    CoatingAnalysisTreeModel::CoatingAnalysisTreeModel(QObject* parent)
        : QAbstractItemModel(parent)
    {
    }

    void CoatingAnalysisTreeModel::setTrajectory(
        const QString& name,
        const CoatingAnalysisTrajectoryInfo& info)
    {
        const bool changed = name != m_trajectoryName
            || info.pointCount != m_trajectoryInfo.pointCount
            || info.durationSeconds != m_trajectoryInfo.durationSeconds
            || info.pathLengthMeters != m_trajectoryInfo.pathLengthMeters
            || info.averageSpeedMetersPerSecond != m_trajectoryInfo.averageSpeedMetersPerSecond;
        if(!changed) {
            return;
        }
        beginResetModel();
        m_trajectoryName = name;
        m_trajectoryInfo = info;
        m_loadedWaypointCount = 0;
        endResetModel();
    }

    void CoatingAnalysisTreeModel::setWorkpieces(
        const QVector<CoatingAnalysisWorkpieceItem>& workpieces,
        const QString& selectedWorkpieceId)
    {
        bool changed = m_selectedWorkpieceId != selectedWorkpieceId
            || workpieces.size() != m_workpieces.size();
        if(!changed) {
            for(int i = 0; i < workpieces.size(); ++i) {
                if(workpieces[i].id != m_workpieces[i].id ||
                    workpieces[i].name != m_workpieces[i].name) {
                    changed = true;
                    break;
                }
            }
        }
        if(!changed) {
            return;
        }
        beginResetModel();
        m_workpieces = workpieces;
        m_selectedWorkpieceId = selectedWorkpieceId;
        endResetModel();
    }

    void CoatingAnalysisTreeModel::setModelVisibility(const QHash<QString, bool>& visibility)
    {
        bool changed = visibility.size() != m_modelVisibility.size();
        if(!changed) {
            for(auto it = visibility.constBegin(); it != visibility.constEnd(); ++it) {
                const auto found = m_modelVisibility.constFind(it.key());
                if(found == m_modelVisibility.constEnd() || found.value() != it.value()) {
                    changed = true;
                    break;
                }
            }
        }
        if(!changed) {
            return;
        }
        m_modelVisibility = visibility;
        if(!m_workpieces.isEmpty()) {
            const QModelIndex parent = index(ModelsRow, 0, QModelIndex());
            emit dataChanged(
                index(0, 0, parent),
                index(m_workpieces.size() - 1, 0, parent),
                { Qt::ForegroundRole });
        }
    }

    void CoatingAnalysisTreeModel::setThickness(
        bool hasThickness,
        const spraythickness::ThicknessMetrics& metrics)
    {
        if(hasThickness == m_hasThickness &&
            (!hasThickness ||
                (metrics.minThickness == m_thicknessMetrics.minThickness &&
                    metrics.maxThickness == m_thicknessMetrics.maxThickness &&
                    metrics.averageThickness == m_thicknessMetrics.averageThickness &&
                    metrics.coverageRatio == m_thicknessMetrics.coverageRatio))) {
            return;
        }
        beginResetModel();
        m_hasThickness = hasThickness;
        m_thicknessMetrics = metrics;
        endResetModel();
    }

    void CoatingAnalysisTreeModel::setWaypoints(
        const std::vector<spraytrajectory::SprayPathPoint>* waypoints)
    {
        beginResetModel();
        m_waypoints = waypoints;
        m_loadedWaypointCount = 0;
        endResetModel();
    }

    CoatingAnalysisNodeKind CoatingAnalysisTreeModel::nodeKind(const QModelIndex& index) const
    {
        if(!index.isValid()) {
            return CoatingAnalysisNodeKind::Root;
        }
        const quintptr id = index.internalId();
        if(id < kIdBase) {
            switch(index.row()) {
            case TrajectoryRow:
                return CoatingAnalysisNodeKind::Trajectory;
            case WaypointsRow:
                return CoatingAnalysisNodeKind::Waypoints;
            case ModelsRow:
                return CoatingAnalysisNodeKind::Models;
            case ThicknessRow:
            default:
                return CoatingAnalysisNodeKind::Thickness;
            }
        }
        const int topLevel = static_cast<int>(id / kIdBase) - 1;
        switch(topLevel) {
        case TrajectoryRow:
            return CoatingAnalysisNodeKind::TrajectoryInfo;
        case WaypointsRow:
            return CoatingAnalysisNodeKind::Waypoint;
        case ModelsRow:
            return CoatingAnalysisNodeKind::Model;
        case ThicknessRow:
        default:
            return CoatingAnalysisNodeKind::ThicknessInfo;
        }
    }

    int CoatingAnalysisTreeModel::waypointIndex(const QModelIndex& index) const
    {
        if(nodeKind(index) != CoatingAnalysisNodeKind::Waypoint) {
            return -1;
        }
        return static_cast<int>(index.internalId() % kIdBase);
    }

    QString CoatingAnalysisTreeModel::workpieceId(const QModelIndex& index) const
    {
        if(nodeKind(index) != CoatingAnalysisNodeKind::Model) {
            return QString();
        }
        const int row = static_cast<int>(index.internalId() % kIdBase);
        return row >= 0 && row < m_workpieces.size()
            ? m_workpieces[row].id
            : QString();
    }

    QModelIndex CoatingAnalysisTreeModel::index(int row, int column, const QModelIndex& parent) const
    {
        if(row < 0 || column < 0) {
            return QModelIndex();
        }
        if(!parent.isValid()) {
            return row < TopLevelRowCount
                ? createIndex(row, column, static_cast<quintptr>(row))
                : QModelIndex();
        }
        const quintptr parentId = parent.internalId();
        const int topLevel = parentId < kIdBase
            ? parent.row()
            : static_cast<int>(parentId / kIdBase) - 1;
        return createIndex(row, column,
            static_cast<quintptr>((topLevel + 1) * kIdBase + row));
    }

    QModelIndex CoatingAnalysisTreeModel::parent(const QModelIndex& index) const
    {
        if(!index.isValid()) {
            return QModelIndex();
        }
        const quintptr id = index.internalId();
        if(id < kIdBase) {
            return QModelIndex();
        }
        const int topLevel = static_cast<int>(id / kIdBase) - 1;
        return topLevel >= 0 && topLevel < TopLevelRowCount
            ? createIndex(topLevel, 0, static_cast<quintptr>(topLevel))
            : QModelIndex();
    }

    int CoatingAnalysisTreeModel::rowCount(const QModelIndex& parent) const
    {
        if(!parent.isValid()) {
            return TopLevelRowCount;
        }
        const quintptr id = parent.internalId();
        if(id >= kIdBase) {
            return 0;
        }
        switch(parent.row()) {
        case TrajectoryRow:
            return hasTrajectory() ? 4 : 0;
        case WaypointsRow:
            return hasWaypoints() ? m_loadedWaypointCount : 0;
        case ModelsRow:
            return m_workpieces.size();
        case ThicknessRow:
            return m_hasThickness ? 6 : 0;
        default:
            return 0;
        }
    }

    int CoatingAnalysisTreeModel::columnCount(const QModelIndex& parent) const
    {
        Q_UNUSED(parent);
        return 1;
    }

    QVariant CoatingAnalysisTreeModel::data(const QModelIndex& index, int role) const
    {
        if(!index.isValid()) {
            return QVariant();
        }
        const CoatingAnalysisNodeKind kind = nodeKind(index);
        const quintptr id = index.internalId();
        const int row = static_cast<int>(id % kIdBase);

        if(role == Qt::DisplayRole) {
            switch(kind) {
            case CoatingAnalysisNodeKind::Trajectory:
                return hasTrajectory()
                    ? QVariant(QStringLiteral("Trajectory  |  %1").arg(m_trajectoryName))
                    : QVariant(QStringLiteral("Trajectory"));
            case CoatingAnalysisNodeKind::Waypoints:
                return hasWaypoints()
                    ? QVariant(QStringLiteral("Waypoints  |  %1").arg(m_waypoints->size()))
                    : QVariant(QStringLiteral("Waypoints"));
            case CoatingAnalysisNodeKind::Models:
                return QVariant(QStringLiteral("Models  |  %1").arg(m_workpieces.size()));
            case CoatingAnalysisNodeKind::Thickness:
                return m_hasThickness
                    ? QVariant(QStringLiteral("Thickness"))
                    : QVariant(QStringLiteral("Thickness"));
            case CoatingAnalysisNodeKind::TrajectoryInfo:
                return QVariant(trajectoryInfoText(row));
            case CoatingAnalysisNodeKind::Waypoint:
                return QVariant(waypointText(row));
            case CoatingAnalysisNodeKind::Model:
                return row >= 0 && row < m_workpieces.size()
                    ? QVariant(m_workpieces[row].name.isEmpty()
                            ? m_workpieces[row].id
                            : m_workpieces[row].name)
                    : QVariant();
            case CoatingAnalysisNodeKind::ThicknessInfo:
                return QVariant(thicknessInfoText(row));
            default:
                return QVariant();
            }
        }
        if(role == Qt::ForegroundRole && kind == CoatingAnalysisNodeKind::Model) {
            if(row >= 0 && row < m_workpieces.size()) {
                const QString id = m_workpieces[row].id;
                const bool visible = m_modelVisibility.contains(id)
                    ? m_modelVisibility.value(id)
                    : true;
                if(!visible) {
                    return QVariant(QColor(150, 150, 150));
                }
            }
        }
        if(role == Qt::FontRole && kind == CoatingAnalysisNodeKind::Model) {
            if(row >= 0 && row < m_workpieces.size() &&
                m_workpieces[row].id == m_selectedWorkpieceId) {
                QFont font;
                font.setBold(true);
                return QVariant(font);
            }
        }
        return QVariant();
    }

    bool CoatingAnalysisTreeModel::hasChildren(const QModelIndex& parent) const
    {
        if(!parent.isValid()) {
            return true;
        }
        const quintptr id = parent.internalId();
        if(id >= kIdBase) {
            return false;
        }
        switch(parent.row()) {
        case TrajectoryRow:
            return hasTrajectory();
        case WaypointsRow:
            return hasWaypoints();
        case ModelsRow:
            return hasModels();
        case ThicknessRow:
            return m_hasThickness;
        default:
            return false;
        }
    }

    bool CoatingAnalysisTreeModel::canFetchMore(const QModelIndex& parent) const
    {
        if(!parent.isValid() || parent.internalId() >= kIdBase) {
            return false;
        }
        if(parent.row() != WaypointsRow || !hasWaypoints()) {
            return false;
        }
        return m_loadedWaypointCount < static_cast<int>(m_waypoints->size());
    }

    void CoatingAnalysisTreeModel::fetchMore(const QModelIndex& parent)
    {
        if(!canFetchMore(parent)) {
            return;
        }
        const int total = static_cast<int>(m_waypoints->size());
        const int newCount = std::min(total, m_loadedWaypointCount + kWaypointBatchSize);
        beginInsertRows(parent, m_loadedWaypointCount, newCount - 1);
        m_loadedWaypointCount = newCount;
        endInsertRows();
    }

    QString CoatingAnalysisTreeModel::waypointText(int index) const
    {
        if(m_waypoints == nullptr || index < 0 ||
            index >= static_cast<int>(m_waypoints->size())) {
            return QString();
        }
        const spraytrajectory::SprayPathPoint& point = (*m_waypoints)[index];
        return QStringLiteral("#%1    t=%2 s    %3")
            .arg(index)
            .arg(point.time, 0, 'f', 2)
            .arg(point.sprayEnabled ? QStringLiteral("spray") : QStringLiteral("stop"));
    }

    QString CoatingAnalysisTreeModel::trajectoryInfoText(int row) const
    {
        switch(row) {
        case 0:
            return formatKeyValue(QStringLiteral("Points"),
                QString::number(static_cast<qulonglong>(m_trajectoryInfo.pointCount)));
        case 1:
            return formatKeyValue(QStringLiteral("Duration"),
                QStringLiteral("%1 s").arg(m_trajectoryInfo.durationSeconds, 0, 'f', 3));
        case 2:
            return formatKeyValue(QStringLiteral("Path length"),
                QStringLiteral("%1 mm").arg(m_trajectoryInfo.pathLengthMeters * 1000.0, 0, 'f', 2));
        case 3:
            return formatKeyValue(QStringLiteral("Avg speed"),
                QStringLiteral("%1 mm/s")
                    .arg(m_trajectoryInfo.averageSpeedMetersPerSecond * 1000.0, 0, 'f', 2));
        default:
            return QString();
        }
    }

    QString CoatingAnalysisTreeModel::thicknessInfoText(int row) const
    {
        switch(row) {
        case 0:
            return formatKeyValue(QStringLiteral("Min"),
                QStringLiteral("%1 um").arg(m_thicknessMetrics.minThickness * 1.0e6, 0, 'f', 1));
        case 1:
            return formatKeyValue(QStringLiteral("Max"),
                QStringLiteral("%1 um").arg(m_thicknessMetrics.maxThickness * 1.0e6, 0, 'f', 1));
        case 2:
            return formatKeyValue(QStringLiteral("Average"),
                QStringLiteral("%1 um").arg(m_thicknessMetrics.averageThickness * 1.0e6, 0, 'f', 1));
        case 3:
            return formatKeyValue(QStringLiteral("Coverage"),
                QStringLiteral("%1%").arg(m_thicknessMetrics.coverageRatio * 100.0, 0, 'f', 1));
        case 4:
            return formatKeyValue(QStringLiteral("Under-coated"),
                QStringLiteral("%1%").arg(m_thicknessMetrics.underCoatedRatio * 100.0, 0, 'f', 1));
        case 5:
            return formatKeyValue(QStringLiteral("Over-coated"),
                QStringLiteral("%1%").arg(m_thicknessMetrics.overCoatedRatio * 100.0, 0, 'f', 1));
        default:
            return QString();
        }
    }
}
