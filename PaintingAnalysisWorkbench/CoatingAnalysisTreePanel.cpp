#include "CoatingAnalysisTreePanel.h"

#include "CoatingAnalysisTreeModel.h"
#include "CoatingAnalysisLanguage.h"

#include <QAction>
#include <QMenu>
#include <QTreeView>
#include <QVBoxLayout>

namespace robot_qt_viewer
{
    CoatingAnalysisTreePanel::CoatingAnalysisTreePanel(QWidget* parent)
        : QWidget(parent)
    {
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);

        m_model = new CoatingAnalysisTreeModel(this);
        m_treeView = new QTreeView(this);
        m_treeView->setModel(m_model);
        m_treeView->setHeaderHidden(true);
        m_treeView->setUniformRowHeights(true);
        m_treeView->setContextMenuPolicy(Qt::CustomContextMenu);
        m_treeView->expandToDepth(0);
        layout->addWidget(m_treeView);

        connect(m_treeView, &QTreeView::customContextMenuRequested,
            this, &CoatingAnalysisTreePanel::showContextMenu);
    }

    void CoatingAnalysisTreePanel::setLanguageCode(const QString& languageCode)
    {
        m_languageCode = languageCode.toLower().startsWith(QStringLiteral("zh"))
            ? QStringLiteral("zh-CN") : QStringLiteral("en");
        m_model->setLanguageCode(languageCode);
    }

    void CoatingAnalysisTreePanel::applyTreeView(const CoatingAnalysisTreeView& view)
    {
        m_model->setTrajectory(view.trajectoryName, view.trajectoryInfo);
        m_model->setWorkpieces(view.workpieces, view.selectedWorkpieceId);
        m_model->setModelVisibility(view.modelVisibility);
        m_model->setThickness(view.hasThickness, view.thicknessMetrics);
    }

    void CoatingAnalysisTreePanel::setWaypoints(
        const std::vector<spraytrajectory::SprayPathPoint>* waypoints)
    {
        m_model->setWaypoints(waypoints);
    }

    void CoatingAnalysisTreePanel::showContextMenu(const QPoint& position)
    {
        const QModelIndex index = m_treeView->indexAt(position);
        if(!index.isValid()) {
            return;
        }
        const CoatingAnalysisNodeKind kind = m_model->nodeKind(index);

        QMenu menu(this);
        QAction* action = nullptr;
        switch(kind) {
        case CoatingAnalysisNodeKind::Trajectory:
        case CoatingAnalysisNodeKind::Waypoints: {
            if(!m_model->hasChildren(index)) {
                return;
            }
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Delete Trajectory")));
            connect(action, &QAction::triggered,
                this, &CoatingAnalysisTreePanel::trajectoryDeleteRequested);
            break;
        }
        case CoatingAnalysisNodeKind::Waypoint: {
            const int waypointIndex = m_model->waypointIndex(index);
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Waypoint Info...")));
            connect(action, &QAction::triggered, this, [this, waypointIndex]() {
                if(waypointIndex >= 0) {
                    emit waypointInfoRequested(waypointIndex);
                }
            });
            break;
        }
        case CoatingAnalysisNodeKind::Model: {
            const QString objectId = m_model->workpieceId(index);
            if(objectId.isEmpty()) {
                return;
            }
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Set as Prediction Workpiece")));
            connect(action, &QAction::triggered, this, [this, objectId]() {
                emit modelSetAsWorkpiece(objectId);
            });
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Show / Hide Model")));
            connect(action, &QAction::triggered, this, [this, objectId]() {
                emit modelVisibilityToggleRequested(objectId);
            });
            menu.addSeparator();
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Delete Model")));
            connect(action, &QAction::triggered, this, [this, objectId]() {
                emit modelDeleteRequested(objectId);
            });
            break;
        }
        case CoatingAnalysisNodeKind::Thickness: {
            action = menu.addAction(coatingAnalysisTranslate(
                m_languageCode, QStringLiteral("Clear Result")));
            connect(action, &QAction::triggered, this, &CoatingAnalysisTreePanel::thicknessClearRequested);
            break;
        }
        default:
            return;
        }
        if(action != nullptr) {
            menu.exec(m_treeView->viewport()->mapToGlobal(position));
        }
    }
}
