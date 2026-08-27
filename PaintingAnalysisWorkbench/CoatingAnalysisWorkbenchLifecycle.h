#pragma once

#include "RobotQtViewerLocalization.h"
#include "RobotQtViewerWorkbenchLifecycle.h"
#include "RobotQtViewerWorkbenchPackageRegistry.h"

namespace robot_qt_viewer
{
    class CoatingAnalysisModuleController;

    class CoatingAnalysisWorkbenchLifecycle final : public IRobotQtViewerWorkbenchLifecycle
    {
    public:
        explicit CoatingAnalysisWorkbenchLifecycle(
            CoatingAnalysisModuleController& controller);

        RobotQtViewerWorkbenchTransitionResult prepareDeactivate(
            const RobotQtViewerWorkbenchTransitionContext& context) override;
        RobotQtViewerWorkbenchTransitionResult deactivate(
            const RobotQtViewerWorkbenchTransitionContext& context) override;
        RobotQtViewerWorkbenchTransitionResult activate(
            const RobotQtViewerWorkbenchActivationContext& context) override;
        void releaseProject(
            const RobotQtViewerWorkbenchProjectReleaseContext& context) noexcept override;
        void shutdown(
            const RobotQtViewerWorkbenchShutdownContext& context) noexcept override;

    private:
        CoatingAnalysisModuleController& m_controller;
    };

    class CoatingAnalysisWorkbenchLanguageParticipant final
        : public IRobotQtViewerLanguageParticipant
    {
    public:
        explicit CoatingAnalysisWorkbenchLanguageParticipant(
            CoatingAnalysisModuleController& controller);
        void retranslateUi(
            const RobotQtViewerLocalizationService& localization) noexcept override;

    private:
        CoatingAnalysisModuleController& m_controller;
    };

    bool registerPaintingAnalysisWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog);
}
