#pragma once
#include "RobotQtViewerViewportServices.h"
#include <MotionPlanningCore/MotionPlanning.h>

namespace robot_qt_viewer
{
    struct OnlineSceneMotionSample
    {
        Eigen::Isometry3d gun = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        std::vector<double> robotJoints;
        double tableJoint = 0.0;
        bool sprayEnabled = false;
    };
    // Transient execution; never writes the project document or its calibration.
    class OnlineSceneMotion
    {
    public:
        bool configure(const simulation_project::ProjectDocument& document,
            RobotQtViewerViewportServices& services, const QString& attachmentId, const QString& frameId,
            const QString& targetId, const QString& tableRobotId, const QString& tableJoint,
            bool useTrajectory, bool usePlannedRpm, double manualRpm, QString& error);
        OnlineSceneMotionSample sample(double time) const;
        void apply(double time);
        void restore(bool restoreRobotJoints = true);
        double duration() const;
        double nextBoundary(double time) const;
        double maximumRpm() const { return m_maximumRpm; }
        bool followsTrajectory() const { return m_useTrajectory; }
        QString robotId() const { return m_robotId; }
    private:
        RobotQtViewerViewportServices* m_services = nullptr;
        motion_planning::StoredMotionPlan m_plan;
        RobotQtViewerViewportServices::RobotForwardKinematics m_robotFk, m_targetFk;
        std::vector<double> m_robotInitialJoints;
        std::vector<double> m_rotationAtBoundary;
        std::vector<std::string> m_jointNames;
        QString m_robotId, m_tableRobotId, m_tableJointName;
        double m_initialTableJoint = 0.0, m_manualRpm = 0.0, m_maximumRpm = 0.0;
        bool m_useTrajectory = false, m_usePlannedRpm = false;
    };
}
