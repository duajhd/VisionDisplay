cmake_minimum_required(VERSION 3.16)

project(VisionToolsLib VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC OFF)

option(VISIONTOOLS_BUILD_EXAMPLES "Build VisionToolsLib examples" ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui)

add_library(VisionTools SHARED
    include/VisionTools/VisionTools_global.h
    include/VisionTools/ImageView.h
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
)

target_compile_definitions(VisionTools
    PRIVATE VISIONTOOLS_LIBRARY
)

target_include_directories(VisionTools
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(VisionTools
    PUBLIC
        Qt6::Core
        Qt6::Gui
)

if(VISIONTOOLS_BUILD_EXAMPLES)
    add_subdirectory(examples/CaliperDemo)
    add_subdirectory(examples/FindLineDemo)
    add_subdirectory(examples/FindCircleDemo)
    add_subdirectory(examples/FindRectDemo)
    add_subdirectory(examples/FindEllipseDemo)
endif()
