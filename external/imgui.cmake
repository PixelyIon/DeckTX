set(IMGUI_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/imgui)

add_library(imgui STATIC)

target_sources(imgui PRIVATE
    ${IMGUI_SOURCE_DIR}/imgui.cpp
    ${IMGUI_SOURCE_DIR}/imgui_demo.cpp
    ${IMGUI_SOURCE_DIR}/imgui_draw.cpp
    ${IMGUI_SOURCE_DIR}/imgui_widgets.cpp
    ${IMGUI_SOURCE_DIR}/imgui_tables.cpp
    ${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp
)

target_include_directories(imgui PUBLIC ${IMGUI_SOURCE_DIR} PRIVATE ${IMGUI_SOURCE_DIR}/backends)

target_link_libraries(imgui PUBLIC SDL3::SDL3)