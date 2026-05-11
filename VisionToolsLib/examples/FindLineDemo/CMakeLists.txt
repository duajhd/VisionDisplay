add_executable(FindLineDemo
    main.cpp
)

target_link_libraries(FindLineDemo
    PRIVATE
        VisionTools
        Qt6::Core
        Qt6::Gui
)

add_custom_command(TARGET FindLineDemo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:VisionTools>
        $<TARGET_FILE_DIR:FindLineDemo>
)
