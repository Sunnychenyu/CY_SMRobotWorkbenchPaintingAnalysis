#pragma once

#include <RotationBodyTrajectoryPlanning/TrajectoryPlanning/TrajectoryTypes.h>
#include <SprayThicknessPrediction/LocalDepositionCorrection.h>

namespace robot_qt_viewer
{
    // Use the final published labels, including manual region edits.
    std::shared_ptr<const spraythickness::LocalDepositionCorrectionGeometry>
        makePublishedToothTopCorrectionGeometry(
            const smrobot::spray::rotationbody::PublishedTrajectoryPlan& plan,
            const Eigen::Isometry3d& planningFromPrediction);
}
