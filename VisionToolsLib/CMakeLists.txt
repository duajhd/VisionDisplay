cmake_minimum_required(VERSION 3.16)

project(VisionToolsLib VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC OFF)

option(VISIONTOOLS_BUILD_EXAMPLES "Build VisionToolsLib examples" ON)
option(VISIONTOOLS_BUILD_TESTS "Build VisionToolsLib tests" ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui)
find_package(OpenCV REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
set(GFLAGS_USE_TARGET_NAMESPACE FALSE CACHE BOOL "Use non-namespaced gflags CMake targets" FORCE)
find_package(gflags CONFIG REQUIRED)
if(NOT TARGET gflags::gflags)
    get_filename_component(_visiontools_gflags_prefix "${gflags_DIR}/../.." ABSOLUTE)
    add_library(gflags::gflags INTERFACE IMPORTED)
    set_target_properties(gflags::gflags PROPERTIES
        INTERFACE_LINK_LIBRARIES "$<$<CONFIG:Debug>:${_visiontools_gflags_prefix}/debug/lib/gflags_debug.lib>;$<$<NOT:$<CONFIG:Debug>>:${_visiontools_gflags_prefix}/lib/gflags.lib>;shlwapi.lib"
    )
    unset(_visiontools_gflags_prefix)
endif()
find_package(Ceres CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_library(VisionTools SHARED
    include/VisionTools/VisionTools_global.h
    include/VisionTools/ImageView.h
    include/VisionTools/Backend/OpenCvBackend.h
    include/VisionTools/Backend/CeresBackend.h
    include/VisionTools/Backend/DependencySelfTest.h
    include/VisionTools/CaliperTypes.h
    include/VisionTools/CaliperTool.h
    include/VisionTools/EdgePoint.h
    include/VisionTools/LineTypes.h
    include/VisionTools/LineFitter.h
    include/VisionTools/FindLineTool.h
    include/VisionTools/CircleTypes.h
    include/VisionTools/CircleFitter.h
    include/VisionTools/FindCircleTool.h
    include/VisionTools/RectTypes.h
    include/VisionTools/FindRectTool.h
    include/VisionTools/EllipseTypes.h
    include/VisionTools/EllipseFitter.h
    include/VisionTools/FindEllipseTool.h
    include/VisionTools/Matching/MatchTypes.h
    include/VisionTools/Matching/ImagePyramid.h
    include/VisionTools/Matching/GrayTemplate.h
    include/VisionTools/Matching/GrayMatcher.h
    include/VisionTools/Matching/ShapeTemplateModel.h
    include/VisionTools/Matching/ShapeModelBuilder.h
    src/Backend/OpenCvBackend.cpp
    src/Backend/CeresBackend.cpp
    src/Backend/DependencySelfTest.cpp
    src/ImageView.cpp
    src/CaliperTool.cpp
    src/EdgePoint.cpp
    src/LineFitter.cpp
    src/FindLineTool.cpp
    src/CircleFitter.cpp
    src/FindCircleTool.cpp
    src/FindRectTool.cpp
    src/EllipseFitter.cpp
    src/FindEllipseTool.cpp
    src/Matching/ImagePyramid.cpp
    src/Matching/GrayMatcher.cpp
    src/Matching/ShapeTemplateModel.cpp
    src/Matching/ShapeModelBuilder.cpp
)

target_compile_definitions(VisionTools
    PRIVATE VISIONTOOLS_LIBRARY
)

target_include_directories(VisionTools
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(VisionTools
    PUBLIC
        Qt6::Core
        Qt6::Gui
        Eigen3::Eigen
    PRIVATE
        ${OpenCV_LIBS}
        Ceres::ceres
        nlohmann_json::nlohmann_json
        spdlog::spdlog
)

if(VISIONTOOLS_BUILD_EXAMPLES)
    add_subdirectory(examples/CaliperDemo)
    add_subdirectory(examples/FindLineDemo)
    add_subdirectory(examples/FindCircleDemo)
    add_subdirectory(examples/FindRectDemo)
    add_subdirectory(examples/FindEllipseDemo)
endif()

if(VISIONTOOLS_BUILD_TESTS)
    add_executable(CaliperExpectedSelectionTest
        tests/CaliperExpectedSelectionTest.cpp
    )
    target_link_libraries(CaliperExpectedSelectionTest
        PRIVATE VisionTools
    )

    add_executable(FindCircleStabilityTest
        tests/FindCircleStabilityTest.cpp
    )
    target_link_libraries(FindCircleStabilityTest
        PRIVATE VisionTools
    )
endif()
