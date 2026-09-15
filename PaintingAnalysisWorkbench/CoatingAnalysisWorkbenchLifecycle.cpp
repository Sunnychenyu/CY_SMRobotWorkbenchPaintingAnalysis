#include "CoatingAnalysisWorkbenchLifecycle.h"

#include "CoatingAnalysisModuleController.h"

namespace robot_qt_viewer
{
    bool registerPaintingAnalysisWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog)
    {
        const QString packageId = QStringLiteral("smrobot.workbench.painting-analysis");
        if(!catalog.registerPackage(makeRobotQtViewerWorkbenchPackage(
               packageId, QStringLiteral("Painting Analysis"))) ||
            !catalog.registerWorkbench(makeRobotQtViewerWorkbench(
               packageId,
               RobotQtViewerWorkbenchKind::CoatingAnalysis,
               QStringLiteral("coatingAnalysisWorkbench"),
               70,
               { QStringLiteral("smrobot.feature.coating-analysis") },
               { robotQtViewerWorkbenchId(RobotQtViewerWorkbenchKind::Browse) }))) {
            return false;
        }
        return catalog.registerFeature(makeRobotQtViewerWorkbenchFeature(
            QStringLiteral("smrobot.feature.coating-analysis"),
            QStringLiteral("Coating Analysis"),
            packageId,
            { robotQtViewerWorkbenchId(RobotQtViewerWorkbenchKind::CoatingAnalysis) }));
    }

    CoatingAnalysisWorkbenchLifecycle::CoatingAnalysisWorkbenchLifecycle(
        CoatingAnalysisModuleController& controller)
        : m_controller(controller)
    {
    }

    RobotQtViewerWorkbenchTransitionResult
    CoatingAnalysisWorkbenchLifecycle::prepareDeactivate(
        const RobotQtViewerWorkbenchTransitionContext&)
    {
        return workbenchTransitionSucceeded();
    }

    RobotQtViewerWorkbenchTransitionResult CoatingAnalysisWorkbenchLifecycle::deactivate(
        const RobotQtViewerWorkbenchTransitionContext&)
    {
        m_controller.deactivate();
        return workbenchTransitionSucceeded();
    }

    RobotQtViewerWorkbenchTransitionResult CoatingAnalysisWorkbenchLifecycle::activate(
        const RobotQtViewerWorkbenchActivationContext&)
    {
        m_controller.activate();
        return workbenchTransitionSucceeded();
    }

    void CoatingAnalysisWorkbenchLifecycle::releaseProject(
        const RobotQtViewerWorkbenchProjectReleaseContext&) noexcept
    {
        m_controller.deactivate();
    }

    void CoatingAnalysisWorkbenchLifecycle::shutdown(
        const RobotQtViewerWorkbenchShutdownContext&) noexcept
    {
        m_controller.deactivate();
    }

    CoatingAnalysisWorkbenchLanguageParticipant::
    CoatingAnalysisWorkbenchLanguageParticipant(
        CoatingAnalysisModuleController& controller)
        : m_controller(controller)
    {
    }

    void CoatingAnalysisWorkbenchLanguageParticipant::retranslateUi(
        const RobotQtViewerLocalizationService& localization) noexcept
    {
        m_controller.setLanguageCode(localization.currentLanguageId());
    }
}
