add_executable(FindCircleDemo
    main.cpp
)

target_link_libraries(FindCircleDemo
    PRIVATE
        VisionTools
        Qt6::Core
        Qt6::Gui
)

add_custom_command(TARGET FindCircleDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:FindCircleDemo>
)
