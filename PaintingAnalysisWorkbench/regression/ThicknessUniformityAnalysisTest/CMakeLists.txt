cmake_minimum_required(VERSION 3.20)

add_executable(ThicknessUniformityAnalysisTest main.cpp)
target_link_libraries(ThicknessUniformityAnalysisTest
    PRIVATE
        SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench
)
target_compile_features(ThicknessUniformityAnalysisTest PRIVATE cxx_std_17)
set_target_properties(ThicknessUniformityAnalysisTest PROPERTIES
    FOLDER tests/SMRobotWorkbenchPaintingAnalysis
)
add_test(
    NAME ThicknessUniformityAnalysisTest
    COMMAND $<TARGET_FILE:ThicknessUniformityAnalysisTest>
)
