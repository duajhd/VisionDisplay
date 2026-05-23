cmake_minimum_required(VERSION 3.16)

project(VisionDisplayLib VERSION 0.9.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(QT_QML_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/qml)

option(ENABLE_ASAN "Enable AddressSanitizer" OFF)

set(VISIONDISPLAY_VCPKG_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet used by VisionDisplay dependencies")
set(_visiondisplay_vcpkg_prefix_candidates
    "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/${VISIONDISPLAY_VCPKG_TRIPLET}"
    "${CMAKE_BINARY_DIR}/vcpkg_installed/${VISIONDISPLAY_VCPKG_TRIPLET}"
)
foreach(_visiondisplay_vcpkg_prefix IN LISTS _visiondisplay_vcpkg_prefix_candidates)
    if(EXISTS "${_visiondisplay_vcpkg_prefix}/share/opencv4/OpenCVConfig.cmake")
        list(PREPEND CMAKE_PREFIX_PATH "${_visiondisplay_vcpkg_prefix}")
        if(NOT OpenCV_DIR)
            set(OpenCV_DIR "${_visiondisplay_vcpkg_prefix}/share/opencv4" CACHE PATH "OpenCV package config directory")
        endif()
        break()
    endif()
endforeach()
unset(_visiondisplay_vcpkg_prefix)
unset(_visiondisplay_vcpkg_prefix_candidates)

function(enable_msvc_asan target_name)
    if(ENABLE_ASAN AND MSVC AND TARGET ${target_name})
        target_compile_options(${target_name} PRIVATE /fsanitize=address)
        target_link_options(${target_name} PRIVATE /fsanitize=address)
    endif()
endfunction()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Qt6 REQUIRED COMPONENTS Quick Concurrent)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

qt_standard_project_setup(REQUIRES 6.8)

set(VISIONTOOLS_BUILD_EXAMPLES OFF CACHE BOOL "Build VisionToolsLib examples" FORCE)
add_subdirectory(VisionToolsLib)
enable_msvc_asan(VisionTools)

qt_add_library(VisionDisplay SHARED)

qt_add_qml_module(VisionDisplay
    URI VisionDisplay
    VERSION 1.0
    SOURCES
        include/VisionDisplay/VisionDisplay_global.h
        include/VisionDisplay/CoordinateMapper.h
        include/VisionDisplay/FrameData.h
        include/VisionDisplay/GraphicObject.h
        include/VisionDisplay/GraphicManager.h
        include/VisionDisplay/OverlayData.h
        include/VisionDisplay/RoiObject.h
        include/VisionDisplay/RoiManager.h
        include/VisionDisplay/VisionDisplayTypes.h
        include/VisionDisplay/VisionDisplayItem.h
        src/CoordinateMapper.cpp
        src/GraphicObject.cpp
        src/GraphicManager.cpp
        src/OverlayData.cpp
        src/RoiObject.cpp
        src/RoiManager.cpp
        src/VisionDisplayItem.cpp
        src/Plugin.cpp
)

target_compile_definitions(VisionDisplay
    PRIVATE VISIONDISPLAY_LIBRARY
)

target_include_directories(VisionDisplay
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/VisionDisplay
)

target_link_libraries(VisionDisplay
    PUBLIC Qt6::Quick
)

enable_msvc_asan(VisionDisplay)
if(TARGET VisionDisplayplugin)
    enable_msvc_asan(VisionDisplayplugin)
endif()

add_library(ShapeMatchEval STATIC
    src/shape_match/core/ShapeMatchTypes.h
    src/shape_match/core/ShapeMatchTypes.cpp
    src/shape_match/core/ShapeTemplateModel.h
    src/shape_match/core/ShapeTemplateModel.cpp
    src/shape_match/core/EdgeImageData.h
    src/shape_match/core/EdgeImageData.cpp
    src/shape_match/coarse/CoarseMatchConfig.h
    src/shape_match/coarse/CoarseCandidate.h
    src/shape_match/coarse/CoarseMatchReport.h
    src/shape_match/coarse/ImagePyramid.h
    src/shape_match/coarse/ImagePyramid.cpp
    src/shape_match/coarse/TemplatePyramid.h
    src/shape_match/coarse/TemplatePyramid.cpp
    src/shape_match/coarse/DistanceFieldBuilder.h
    src/shape_match/coarse/DistanceFieldBuilder.cpp
    src/shape_match/coarse/RoiDistanceField.h
    src/shape_match/coarse/RoiDistanceField.cpp
    src/shape_match/coarse/RoiDistanceFieldBuilder.h
    src/shape_match/coarse/RoiDistanceFieldBuilder.cpp
    src/shape_match/coarse/CandidateRegionGenerator.h
    src/shape_match/coarse/CandidateRegionGenerator.cpp
    src/shape_match/coarse/TemplatePointSoA.h
    src/shape_match/coarse/TemplatePointSoA.cpp
    src/shape_match/coarse/RotatedTemplateCache.h
    src/shape_match/coarse/RotatedTemplateCache.cpp
    src/shape_match/coarse/CandidateBudgetPolicy.h
    src/shape_match/coarse/CandidateBudgetPolicy.cpp
    src/shape_match/coarse/ParallelCandidateScorer.h
    src/shape_match/coarse/ParallelCandidateScorer.cpp
    src/shape_match/coarse/FastPoseScorer.h
    src/shape_match/coarse/FastPoseScorer.cpp
    src/shape_match/coarse/TopKCandidateBuffer.h
    src/shape_match/coarse/TopKCandidateBuffer.cpp
    src/shape_match/coarse/SpatialDiversityTopKBuffer.h
    src/shape_match/coarse/SpatialDiversityTopKBuffer.cpp
    src/shape_match/coarse/CoarsePoseGenerator.h
    src/shape_match/coarse/CoarsePoseGenerator.cpp
    src/shape_match/coarse/CoarseShapeMatcher.h
    src/shape_match/coarse/CoarseShapeMatcher.cpp
    src/shape_match/coarse/CoarseMatchReportWriter.h
    src/shape_match/coarse/CoarseMatchReportWriter.cpp
    src/shape_match/coarse/CoarseMatchProfiler.h
    src/shape_match/coarse/CoarseMatchProfiler.cpp
    src/shape_match/coarse/MissedTargetDiagnostic.h
    src/shape_match/coarse/MissedTargetDiagnostic.cpp
    src/shape_match/coarse/MissedTargetStageTrace.h
    src/shape_match/coarse/MissedTargetStageTrace.cpp
    src/shape_match/coarse/VotingConfig.h
    src/shape_match/coarse/SparseAccumulator.h
    src/shape_match/coarse/SparseAccumulator.cpp
    src/shape_match/coarse/RTTable.h
    src/shape_match/coarse/RTTable.cpp
    src/shape_match/coarse/ImageEdgeSampler.h
    src/shape_match/coarse/ImageEdgeSampler.cpp
    src/shape_match/coarse/VotingDebugReport.h
    src/shape_match/coarse/OrientationVotingCandidateGenerator.h
    src/shape_match/coarse/OrientationVotingCandidateGenerator.cpp
    src/shape_match/coarse/CoarseMatchDebugOverlay.h
    src/shape_match/coarse/CoarseMatchDebugOverlay.cpp
    src/shape_match/evaluation/ShapeMatchEvalConfig.h
    src/shape_match/evaluation/MatchEvaluator.h
    src/shape_match/evaluation/MatchEvaluator.cpp
    src/shape_match/evaluation/FinalCandidateNms.h
    src/shape_match/evaluation/FinalCandidateNms.cpp
    src/shape_match/evaluation/CandidateRanker.h
    src/shape_match/evaluation/CandidateRanker.cpp
    src/shape_match/evaluation/PoseErrorEvaluator.h
    src/shape_match/evaluation/PoseErrorEvaluator.cpp
    src/shape_match/evaluation/MultiTargetEvaluator.h
    src/shape_match/evaluation/MultiTargetEvaluator.cpp
    src/shape_match/io/MatchReportWriter.h
    src/shape_match/io/MatchReportWriter.cpp
    src/shape_match/io/VisionProGroundTruthReader.h
    src/shape_match/io/VisionProGroundTruthReader.cpp
    src/shape_match/overlay/ShapeMatchOverlayData.h
    src/shape_match/overlay/MatchDebugOverlayAdapter.h
    src/shape_match/overlay/MatchDebugOverlayAdapter.cpp
    src/shape_match/tests/ShapeMatchEvalSelfTest.h
    src/shape_match/tests/ShapeMatchEvalSelfTest.cpp
)

target_include_directories(ShapeMatchEval
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ShapeMatchEval
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        ${OpenCV_LIBS}
)

enable_msvc_asan(ShapeMatchEval)

add_library(ShapeMatchVisionDisplayOverlay STATIC
    src/shape_match/overlay/VisionDisplayOverlayAdapter.h
    src/shape_match/overlay/VisionDisplayOverlayAdapter.cpp
)

target_include_directories(ShapeMatchVisionDisplayOverlay
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ShapeMatchVisionDisplayOverlay
    PUBLIC
        ShapeMatchEval
        VisionDisplay
)

enable_msvc_asan(ShapeMatchVisionDisplayOverlay)

add_executable(ShapeMatchEvalSelfTest
    src/shape_match/tests/ShapeMatchEvalSelfTestMain.cpp
)

target_link_libraries(ShapeMatchEvalSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(ShapeMatchEvalSelfTest)

add_executable(CoarseShapeMatcherSelfTest
    src/shape_match/tests/CoarseShapeMatcherSelfTestMain.cpp
    src/shape_match/tests/CoarseShapeMatcherSelfTest.h
    src/shape_match/tests/CoarseShapeMatcherSelfTest.cpp
)

target_link_libraries(CoarseShapeMatcherSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(CoarseShapeMatcherSelfTest)

add_executable(FastPoseScorerSelfTest
    src/shape_match/tests/FastPoseScorerSelfTestMain.cpp
    src/shape_match/tests/FastPoseScorerSelfTest.h
    src/shape_match/tests/FastPoseScorerSelfTest.cpp
)

target_link_libraries(FastPoseScorerSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(FastPoseScorerSelfTest)

add_executable(RecallSpeedTuningTest
    src/shape_match/tests/RecallSpeedTuningTestMain.cpp
    src/shape_match/tests/RecallSpeedTuningTest.h
    src/shape_match/tests/RecallSpeedTuningTest.cpp
)

target_link_libraries(RecallSpeedTuningTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(RecallSpeedTuningTest)

add_executable(OrientationVotingSelfTest
    src/shape_match/tests/OrientationVotingSelfTestMain.cpp
    src/shape_match/tests/OrientationVotingSelfTest.h
    src/shape_match/tests/OrientationVotingSelfTest.cpp
)

target_link_libraries(OrientationVotingSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(OrientationVotingSelfTest)

add_executable(MissedTargetStageTraceSelfTest
    src/shape_match/tests/MissedTargetStageTraceSelfTest.cpp
)

target_link_libraries(MissedTargetStageTraceSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(MissedTargetStageTraceSelfTest)

add_executable(VisionProGroundTruthReaderSelfTest
    src/shape_match/tests/VisionProGroundTruthReaderSelfTest.cpp
)

target_link_libraries(VisionProGroundTruthReaderSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(VisionProGroundTruthReaderSelfTest)

add_executable(FinalRankerOptimizationSelfTest
    src/shape_match/tests/FinalRankerOptimizationSelfTest.cpp
)

target_link_libraries(FinalRankerOptimizationSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(FinalRankerOptimizationSelfTest)

add_executable(LargeImageRoiDistanceFieldSelfTest
    src/shape_match/tests/LargeImageRoiDistanceFieldSelfTest.cpp
)

target_link_libraries(LargeImageRoiDistanceFieldSelfTest
    PRIVATE ShapeMatchEval
)

enable_msvc_asan(LargeImageRoiDistanceFieldSelfTest)

set_target_properties(VisionDisplay PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME VisionDisplay
)

qt_add_executable(appVisionDisplay
    main.cpp
    IntegratedDemoController.h
    IntegratedDemoController.cpp
    ToolResultDisplayAdapter.h
    ToolResultDisplayAdapter.cpp
    ShapeModelOverlayBuilder.h
    ShapeModelOverlayBuilder.cpp
    ShapeModelDebugViewModel.h
    ShapeModelDebugViewModel.cpp
)

qt_add_qml_module(appVisionDisplay
    URI VisionDisplayApp
    VERSION 1.0
    QML_FILES
        Main.qml
        ShapeModelDebugPage.qml
)

target_link_libraries(appVisionDisplay
    PRIVATE
        Qt6::Quick
        Qt6::Concurrent
        VisionDisplay
        VisionTools
        ShapeMatchEval
        ShapeMatchVisionDisplayOverlay
)

enable_msvc_asan(appVisionDisplay)

set_target_properties(appVisionDisplay PROPERTIES
    WIN32_EXECUTABLE TRUE
)

add_custom_command(TARGET appVisionDisplay POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:appVisionDisplay>
)

install(TARGETS VisionDisplay
    EXPORT VisionDisplayTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

if(TARGET VisionDisplayplugin)
    install(TARGETS VisionDisplayplugin
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/qml/VisionDisplay
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/qml/VisionDisplay
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

install(DIRECTORY include/VisionDisplay
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT VisionDisplayTargets
    NAMESPACE VisionDisplay::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/VisionDisplay
)

configure_package_config_file(
    cmake/VisionDisplayConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/VisionDisplayConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/VisionDisplay
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/VisionDisplayConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/VisionDisplayConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/VisionDisplayConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/VisionDisplay
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/qml/VisionDisplay
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/qml
    OPTIONAL
)
