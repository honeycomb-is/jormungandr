cmake_minimum_required(VERSION 3.20)
project(Honeycomb LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find packages
find_package(glfw3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)

# ImGui sources
set(IMGUI_DIR packages/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_metal.mm
)

# Application sources
add_executable(Honeycomb
    src/main.mm
    src/Honeycomb.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(Honeycomb PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    src
)

# Provide absolute path to assets for runtime resource loading
target_compile_definitions(Honeycomb PRIVATE HONEYCOMB_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets")

target_compile_options(Honeycomb PRIVATE
    $<$<COMPILE_LANGUAGE:OBJCXX>:-Wno-arc-bridge-casts-disallowed-in-nonarc>
)

target_link_libraries(Honeycomb
    glfw
    "-framework Metal"
    "-framework MetalKit"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
    "-framework QuartzCore"
)