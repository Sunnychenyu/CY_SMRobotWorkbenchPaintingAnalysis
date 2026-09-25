#include "CoatingAnalysisWorkbenchLifecycle.h"

#include "CoatingAnalysisModuleController.h"

namespace robot_qt_viewer
{
    bool registerPaintingAnalysisWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog)
    {
        const QString packageId = QStringLiteral("smrobot.workbench.painting-analysis");
#if SMROBOT_PAINTING_ANALYSIS_HAS_WORKBENCH_LIFECYCLE
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
#else
        RobotQtViewerWorkbenchPackageDesc package;
        package.id = packageId;
        package.displayName = QStringLiteral("Painting Analysis");
        if(!catalog.registerPackage(package)) {
            return false;
        }

        RobotQtViewerWorkbenchModeDesc mode;
        mode.packageId = packageId;
        mode.descriptor = robotQtViewerWorkbenchDescriptor(
            RobotQtViewerWorkbenchKind::CoatingAnalysis);
        return catalog.registerMode(mode);
#endif
    }

#if SMROBOT_PAINTING_ANALYSIS_HAS_WORKBENCH_LIFECYCLE
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
#endif
}
