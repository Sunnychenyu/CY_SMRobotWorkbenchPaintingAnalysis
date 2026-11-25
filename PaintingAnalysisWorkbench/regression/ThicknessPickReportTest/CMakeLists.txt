cmake_minimum_required(VERSION 3.20)

add_executable(ThicknessPickReportTest main.cpp DynamicSurfaceReportTests.cpp SectionSamplingTests.cpp SectionDialogTests.cpp SectionSchematicTests.cpp SectionReportReproduction.cpp)
target_link_libraries(ThicknessPickReportTest
    PRIVATE SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench Qt5::Core Qt5::Widgets)
target_compile_features(ThicknessPickReportTest PRIVATE cxx_std_17)
set_target_properties(ThicknessPickReportTest PROPERTIES
    FOLDER tests/SMRobotWorkbenchPaintingAnalysis)
add_test(NAME ThicknessPickReportTest COMMAND $<TARGET_FILE:ThicknessPickReportTest>)
