#include "LocalDepositionCorrectionAdapter.h"

#include <algorithm>
#include <stdexcept>

namespace robot_qt_viewer
{
    std::shared_ptr<const spraythickness::LocalDepositionCorrectionGeometry>
        makePublishedToothTopCorrectionGeometry(
            const smrobot::spray::rotationbody::PublishedTrajectoryPlan& plan,
            const Eigen::Isometry3d& planningFromPrediction)
    {
        if(!plan.section || !plan.regions || !plan.regions->matches(*plan.section)) {
            throw std::invalid_argument("The saved section and region labels do not match.");
        }
        std::vector<std::uint8_t> toothTop;
        toothTop.reserve(plan.regions->segmentLabels.size());
        for(const auto label : plan.regions->segmentLabels) {
            toothTop.push_back(label == smrobot::spray::rotationbody::RegionLabel::ToothTop ? 1 : 0);
        }
        const bool radialSection = std::all_of(plan.section->pointsYz.begin(),
            plan.section->pointsYz.end(), [](const Eigen::Vector2d& point) {
                return point.x() >= -1.0e-10;
            });
        return spraythickness::makeLocalDepositionCorrectionGeometry(
            plan.section->pointsYz, toothTop, plan.section->closed,
            planningFromPrediction, radialSection);
    }
}
