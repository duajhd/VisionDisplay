cmake_minimum_required(VERSION 3.16)

project(VisionDisplayLib VERSION 0.9.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(QT_QML_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/qml)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Qt6 REQUIRED COMPONENTS Quick)

qt_standard_project_setup(REQUIRES 6.8)

set(VISIONTOOLS_BUILD_EXAMPLES OFF CACHE BOOL "Build VisionToolsLib examples" FORCE)
add_subdirectory(VisionToolsLib)

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
        include/VisionDisplay/RoiObject.h
        include/VisionDisplay/RoiManager.h
        include/VisionDisplay/VisionDisplayTypes.h
        include/VisionDisplay/VisionDisplayItem.h
        src/CoordinateMapper.cpp
        src/GraphicObject.cpp
        src/GraphicManager.cpp
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
)

qt_add_qml_module(appVisionDisplay
    URI VisionDisplayApp
    VERSION 1.0
    QML_FILES
        Main.qml
)

target_link_libraries(appVisionDisplay
    PRIVATE
        Qt6::Quick
        VisionDisplay
        VisionTools
)

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
