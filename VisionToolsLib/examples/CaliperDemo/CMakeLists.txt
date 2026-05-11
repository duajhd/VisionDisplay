add_executable(CaliperDemo
    main.cpp
)

target_link_libraries(CaliperDemo
    PRIVATE
        VisionTools
        Qt6::Core
        Qt6::Gui
)

add_custom_command(TARGET CaliperDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:CaliperDemo>
)
