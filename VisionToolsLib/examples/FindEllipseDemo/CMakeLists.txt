qt_add_executable(FindEllipseDemo
    main.cpp
)

target_link_libraries(FindEllipseDemo
    PRIVATE
        VisionTools
)

add_custom_command(TARGET FindEllipseDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:FindEllipseDemo>
)
