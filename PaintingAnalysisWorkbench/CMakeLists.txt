cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME PaintingAnalysisWorkbench)
set(CMAKE_AUTOMOC ON)

set(SOURCES
    AlgorithmReproductionJobController.cpp
    AlgorithmReproductionJobController.h
    CoatingAnalysisLanguage.cpp
    CoatingAnalysisLanguage.h
    CoatingAnalysisInfoPanel.cpp
    CoatingAnalysisInfoPanel.h
    AxisymmetricProfileSelectionDialog.cpp
    AxisymmetricProfileSelectionDialog.h
    CoatingAnalysisModuleController.cpp
    CoatingAnalysisModuleController.h
    CoatingAnalysisWorkbenchLifecycle.cpp
    CoatingAnalysisWorkbenchLifecycle.h
    CoatingAnalysisPanel.cpp
    CoatingAnalysisPanel.h
    CoatingAnalysisSession.h
    CoatingAnalysisTreeModel.cpp
    CoatingAnalysisTreeModel.h
    CoatingAnalysisTreePanel.cpp
    CoatingAnalysisTreePanel.h
    CoatingAnalysisViewModel.h
    CoatingAnalysisVisibilityBar.cpp
    CoatingAnalysisVisibilityBar.h
    SimulationExperiment.cpp
    SimulationExperiment.h
    CoatingAnalysisWaypointDialog.cpp
    CoatingAnalysisWaypointDialog.h
    DepositionCurveWidget.cpp
    DepositionCurveWidget.h
    PaintingAnalysisDialogService.cpp
    PaintingAnalysisDialogService.h
    PaintingAnalysisMeshAdapter.cpp
    PaintingAnalysisMeshAdapter.h
    PublishedReproductionAdapter.cpp
    PublishedReproductionAdapter.h
    PublishedReproductionDisplayAdapter.cpp
    PublishedReproductionDisplayAdapter.h
    ThicknessPredictionJobController.cpp
    ThicknessPredictionJobController.h
    ThicknessUniformityAnalysis.cpp
    ThicknessUniformityAnalysis.h
    ThicknessLegendWidget.cpp
    ThicknessLegendWidget.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(SMRobotWorkbenchPaintingAnalysis::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        SMRobotWorkbenchCommon::WorkbenchCommon
        SMRobotPlatform::AssetCore
        SMRobotPlatform::VisualizationSDK
        SMRobotSpray::SprayThicknessPrediction
        SMRobotSpray::SprayTrajectoryCore
        SMRobotSpray::RotationBodyTrajectoryPlanning
    PRIVATE
        Qt5::Gui
        Qt5::Widgets
        Common::CustomLog
        Common::GLRuntime
        SMRobotPlatform::SimulationProject
        SMRobotSpray::SprayThicknessPredictionOpenGL
        SMRobotSpray::RotationBodyTrajectoryOptimization
        nlohmann_json::nlohmann_json
)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(${TARGET_NAME} PRIVATE /utf-8)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    FOLDER SMRobotWorkbenchPaintingAnalysis
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
    SMROBOT_WORKBENCH_REGISTRAR_HEADER "CoatingAnalysisWorkbenchLifecycle.h"
    SMROBOT_WORKBENCH_REGISTRAR_FUNCTION "robot_qt_viewer::registerPaintingAnalysisWorkbenchContribution"
    EXPORT_PROPERTIES "SMROBOT_WORKBENCH_REGISTRAR_HEADER;SMROBOT_WORKBENCH_REGISTRAR_FUNCTION"
)

if(BuildTests)
    add_subdirectory(
        regression/PaintingAnalysisMeshAdapterTest
        ${PROJECT_BINARY_DIR}/tests/painting_analysis_mesh_adapter)
    add_subdirectory(
        regression/ThicknessUniformityAnalysisTest
        ${PROJECT_BINARY_DIR}/tests/thickness_uniformity_analysis)
endif()

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        SMRobotWorkbenchCommon::WorkbenchCommon
        SMRobotPlatform::AssetCore
        SMRobotPlatform::VisualizationSDK
        SMRobotSpray::SprayThicknessPrediction
        SMRobotSpray::SprayTrajectoryCore
    )
    set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(REMOVE_RECURSE ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})
    file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME})
    foreach(header_ IN LISTS SOURCES)
        if(header_ MATCHES "\\.h$")
            configure_file(
                ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
                ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME}/${header_}
                COPYONLY)
        endif()
    endforeach()
    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )
endif()
