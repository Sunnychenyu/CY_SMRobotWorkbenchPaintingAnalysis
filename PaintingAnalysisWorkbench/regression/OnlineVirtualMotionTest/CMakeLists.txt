cmake_minimum_required(VERSION 3.20)

add_executable(OnlineVirtualMotionTest
    main.cpp
    ../../OnlineVirtualMotion.cpp)
target_link_libraries(OnlineVirtualMotionTest
    PRIVATE SMRobotWorkbenchPaintingAnalysis::PaintingAnalysisWorkbench)
target_compile_features(OnlineVirtualMotionTest PRIVATE cxx_std_17)
set_target_properties(OnlineVirtualMotionTest PROPERTIES
    FOLDER tests/SMRobotWorkbenchPaintingAnalysis)
add_test(NAME OnlineVirtualMotionTest
    COMMAND $<TARGET_FILE:OnlineVirtualMotionTest>)
