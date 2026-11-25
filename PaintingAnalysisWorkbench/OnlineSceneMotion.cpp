#include "OnlineSceneMotion.h"
#include "ConfiguredTcpSelection.h"
#include <RotationBodyTrajectoryPlanning/Persistence/PublishedTrajectoryPlanContract.h>
#include <RotationBodyTrajectoryPlanning/Persistence/PublishedTrajectoryPlanStore.h>
#include <SimulationProject/ProjectDocument.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_qt_viewer
{
    namespace
    {
        constexpr double radiansPerRpmSecond = 6.2831853071795864769 / 60.0;
    }
    bool OnlineSceneMotion::configure(const simulation_project::ProjectDocument& document,
        RobotQtViewerViewportServices& services, const QString& attachmentId, const QString& frameId,
        const QString& targetId, const QString& tableRobotId, const QString& tableJoint,
        bool useTrajectory, bool usePlannedRpm, double manualRpm, QString& error)
    {
        m_services=nullptr;
        const auto fail=[&](const char* message) { error=QString::fromLatin1(message); return false; };
        const auto tcp=configuredTcpSelection(document, configuredTcpSelectionId(attachmentId,frameId));
        if(!tcp) return fail("The configured nozzle TCP is unavailable. Confirm again.");
        if(tableRobotId.isEmpty() || tableJoint.isEmpty() || tableRobotId==tcp->robotId)
            return fail("Select the positioner's rotary table joint, separate from the spray robot.");
        const auto axes=services.robotMotionJoints(tableRobotId);
        const auto axis=std::find_if(axes.begin(),axes.end(),[&](const auto& a) { return a.first==tableJoint.toStdString(); });
        if(axis==axes.end() || (axis->second!="continuous" && axis->second!="revolute"))
            return fail("The selected positioner joint is not rotary.");
        m_robotId=tcp->robotId; m_tableRobotId=tableRobotId; m_tableJointName=tableJoint;
        m_jointNames.clear(); m_robotInitialJoints.clear(); m_plan={}; m_rotationAtBoundary.clear();
        for(const auto& joint:services.robotMotionJoints(m_robotId)) {
            bool ok=false;
            const double value=services.robotJointValue(m_robotId,QString::fromStdString(joint.first),&ok);
            if(!ok) return fail("Cannot read the spray robot joints.");
            m_jointNames.push_back(joint.first); m_robotInitialJoints.push_back(value);
        }
        const auto targetAttachment=std::find_if(document.mountedAttachments.begin(),document.mountedAttachments.end(),
            [&](const auto& a) { return a.sourceObjectId==targetId.toStdString() && a.enabled; });
        if(targetAttachment==document.mountedAttachments.end())
            return fail("Associate or bind the target workpiece to the positioner before starting scene rotation or offline simulation.");
        const auto targetMount=std::find_if(document.robotMounts.begin(),document.robotMounts.end(),
            [&](const auto& a) { return a.id==targetAttachment->mountFrameId; });
        if(targetMount==document.robotMounts.end() || targetMount->robotId!=tableRobotId.toStdString())
            return fail("The target is mounted on a different device. Select its positioner.");
        bool ok=false;
        Eigen::Isometry3d initialTarget;
        m_initialTableJoint=services.robotJointValue(tableRobotId,tableJoint,&ok);
        if(!ok || !services.sceneObjectSurfaceTransform(targetId,initialTarget))
            return fail("Cannot read the positioner or target pose.");
        m_robotFk=services.robotFrameForwardKinematics(tcp->robotId,m_jointNames,tcp->linkName,tcp->linkToFrame);
        const auto mountLinkFk=services.robotFrameForwardKinematics(tableRobotId,{tableJoint.toStdString()},
            QString::fromStdString(targetMount->linkName),Eigen::Isometry3d::Identity());
        if(!m_robotFk || !mountLinkFk) return fail("The robot or positioner kinematics are unavailable.");
        const auto initialMountLink=mountLinkFk({m_initialTableJoint});
        const auto positiveMountLink=mountLinkFk({m_initialTableJoint+0.01});
        const auto negativeMountLink=mountLinkFk({m_initialTableJoint-0.01});
        if(!initialMountLink.matrix().allFinite() || !positiveMountLink.matrix().allFinite()
            || !negativeMountLink.matrix().allFinite())
            return fail("The robot or positioner kinematics are unavailable.");
        // FK captures the existing assembly. A base/other branch cannot drive the target.
        if((initialMountLink.matrix()-positiveMountLink.matrix()).norm()<1.0e-8
            && (initialMountLink.matrix()-negativeMountLink.matrix()).norm()<1.0e-8)
            return fail("The target mount is not driven by the selected positioner joint. Associate or bind it to the rotating link.");
        const Eigen::Isometry3d linkToTarget=initialMountLink.inverse()*initialTarget;
        m_targetFk=services.robotFrameForwardKinematics(tableRobotId,{tableJoint.toStdString()},
            QString::fromStdString(targetMount->linkName),linkToTarget);
        if(!m_targetFk) return fail("The robot or positioner kinematics are unavailable.");
        m_useTrajectory=useTrajectory; m_usePlannedRpm=usePlannedRpm; m_manualRpm=manualRpm;
        std::string publishedPayload;
        for(const auto& extension:document.extensions)
            if(extension.key==smrobot::spray::rotationbody::kPublishedTrajectoryPlanExtensionKey) publishedPayload=extension.serializedPayload;
        if(useTrajectory) {
            bool found=false;
            for(const auto& plan:motion_planning::MotionPlanningProjectStore::plans(document)) {
                if(plan.id!="rotation_body_group_"+m_robotId.toStdString()) continue;
                m_plan=plan; found=true; break;
            }
            if(!found || m_plan.trajectory.points.size()<2 || m_plan.processIntervals.size()!=m_plan.trajectory.points.size())
                return fail("Solve and save the calibrated whole trajectory group in Trajectory IK first.");
            if(m_plan.targetObjectId!=targetId.toStdString() || m_plan.tcpAttachmentId!=attachmentId.toStdString()
                || m_plan.tcpFrameId!=frameId.toStdString() || m_plan.jointNames!=m_jointNames
                || m_plan.sourceSignature!=sprayExecutionSignature(document,m_robotId,tcp->selectionId(),publishedPayload))
                return fail("Trajectory, calibration, robot or nozzle TCP changed. Solve the whole group again.");
            double previous=0.0, angle=0.0;
            m_rotationAtBoundary.reserve(m_plan.processIntervals.size());
            for(std::size_t i=0;i<m_plan.processIntervals.size();++i) {
                const auto& interval=m_plan.processIntervals[i]; const auto& point=m_plan.trajectory.points[i];
                if(!std::isfinite(interval.endTime) || !std::isfinite(interval.positionerRpm)
                    || point.q.size()!=m_jointNames.size() || interval.endTime!=point.time
                    || (i && interval.endTime<=previous) || (!i && interval.endTime!=0.0))
                    return fail("The stored trajectory has invalid execution timing. Solve it again.");
                if(!std::all_of(point.q.begin(),point.q.end(),[](double v){return std::isfinite(v);}))
                    return fail("The stored trajectory contains invalid joints.");
                angle+=(interval.endTime-previous)*interval.positionerRpm*radiansPerRpmSecond;
                m_rotationAtBoundary.push_back(angle); previous=interval.endTime;
            }
        } else if(usePlannedRpm) {
            const auto published=smrobot::spray::rotationbody::PublishedTrajectoryPlanStore::read(publishedPayload);
            if(!published || published->group.passes.empty()) return fail("Publish a rotation-body trajectory to read its positioner speed.");
            m_manualRpm=published->group.passes.front().trajectory.parameters.positionerRpm;
        }
        m_maximumRpm=std::abs(m_manualRpm);
        if(useTrajectory && usePlannedRpm) for(const auto& v:m_plan.processIntervals)
            m_maximumRpm=std::max(m_maximumRpm,std::abs(v.positionerRpm));
        m_services=&services; error.clear(); return true;
    }
    double OnlineSceneMotion::duration() const
    { return m_useTrajectory ? m_plan.trajectory.points.back().time : std::numeric_limits<double>::infinity(); }
    double OnlineSceneMotion::nextBoundary(double time) const
    {
        const auto it=std::upper_bound(m_plan.trajectory.points.begin(),m_plan.trajectory.points.end(),time+1.0e-10,
            [](double t,const auto& point){return t<point.time;});
        return it==m_plan.trajectory.points.end() ? duration() : it->time;
    }
    OnlineSceneMotionSample OnlineSceneMotion::sample(double time) const
    {
        OnlineSceneMotionSample result; time=std::clamp(time,0.0,duration());
        result.robotJoints=m_robotInitialJoints;
        result.sprayEnabled=!m_useTrajectory;
        double angle=time*m_manualRpm*radiansPerRpmSecond;
        if(m_useTrajectory) {
            auto it=std::lower_bound(m_plan.trajectory.points.begin()+1,m_plan.trajectory.points.end(),time,
                [](const auto& point,double t){return point.time<t;});
            if(it==m_plan.trajectory.points.end()) --it;
            const auto index=static_cast<std::size_t>(it-m_plan.trajectory.points.begin());
            const auto& left=*(it-1);
            const double alpha=std::clamp((time-left.time)/(it->time-left.time),0.0,1.0);
            for(std::size_t j=0;j<result.robotJoints.size();++j)
                result.robotJoints[j]=left.q[j]+alpha*(it->q[j]-left.q[j]);
            result.sprayEnabled=m_plan.processIntervals[index].sprayEnabled;
            if(m_usePlannedRpm) angle=m_rotationAtBoundary[index-1]+(time-left.time)
                *m_plan.processIntervals[index].positionerRpm*radiansPerRpmSecond;
        }
        result.tableJoint=m_initialTableJoint+angle;
        result.gun=m_robotFk(result.robotJoints);
        result.target=m_targetFk({result.tableJoint});
        return result;
    }
    void OnlineSceneMotion::apply(double time)
    {
        if(!m_services) return;
        const auto value=sample(time);
        if(m_useTrajectory) for(std::size_t i=0;i<m_jointNames.size();++i)
            m_services->setRobotJointValue(m_robotId,QString::fromStdString(m_jointNames[i]),value.robotJoints[i]);
        m_services->setRobotJointValue(m_tableRobotId,m_tableJointName,value.tableJoint);
    }
    void OnlineSceneMotion::restore(bool restoreRobotJoints)
    {
        if(!m_services) return;
        if(restoreRobotJoints) {
            if(m_useTrajectory) for(std::size_t i=0;i<m_jointNames.size();++i)
                m_services->setRobotJointValue(m_robotId,QString::fromStdString(m_jointNames[i]),m_robotInitialJoints[i]);
            m_services->setRobotJointValue(m_tableRobotId,m_tableJointName,m_initialTableJoint);
        }
        m_services=nullptr;
    }
}
