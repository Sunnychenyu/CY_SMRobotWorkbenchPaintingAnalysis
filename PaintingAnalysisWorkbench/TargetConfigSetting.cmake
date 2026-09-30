cmake_minimum_required(VERSION 3.20)

set(${TARGET_NAME}_RequiredLibsPublic
    SMRobotWorkbenchCommon::WorkbenchCommon
    SMRobotPlatform::AssetCore
    SMRobotPlatform::VisualizationSDK
    SMRobotSpray::SprayThicknessPrediction
    SMRobotSpray::SprayTrajectoryCore
)

set(${TARGET_NAME}_RequiredLibsPrivate
    Qt5::Gui
    Qt5::Widgets
    Common::CustomLog
    Common::GLRuntime
    SMRobotPlatform::SimulationProject
    SMRobotSpray::SprayThicknessPredictionOpenGL
    SMRobotSpray::RotationBodyTrajectoryOptimization
)
