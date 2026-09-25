#pragma once

#include "RobotQtViewerWorkbenchPackageRegistry.h"

#if __has_include("RobotQtViewerLocalization.h") && \
    __has_include("RobotQtViewerWorkbenchLifecycle.h")
#include "RobotQtViewerLocalization.h"
#include "RobotQtViewerWorkbenchLifecycle.h"
#define SMROBOT_PAINTING_ANALYSIS_HAS_WORKBENCH_LIFECYCLE 1
#else
#define SMROBOT_PAINTING_ANALYSIS_HAS_WORKBENCH_LIFECYCLE 0
#endif

namespace robot_qt_viewer
{
    class CoatingAnalysisModuleController;

#if SMROBOT_PAINTING_ANALYSIS_HAS_WORKBENCH_LIFECYCLE
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
#endif

    bool registerPaintingAnalysisWorkbenchContribution(
        RobotQtViewerWorkbenchPackageRegistry& catalog);
}
