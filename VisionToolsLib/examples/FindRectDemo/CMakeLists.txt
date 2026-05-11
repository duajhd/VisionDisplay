qt_add_executable(FindRectDemo
    main.cpp
)

target_link_libraries(FindRectDemo
    PRIVATE
        VisionTools
)

add_custom_command(TARGET FindRectDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:FindRectDemo>
)
