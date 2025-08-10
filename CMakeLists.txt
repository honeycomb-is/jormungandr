cmake_minimum_required(VERSION 3.20)
project(Honeycomb LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ImGui paths
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/src/app_metal.mm
    ${CMAKE_SOURCE_DIR}/src/honeycomb/chem/csv_loader.cpp
    ${CMAKE_SOURCE_DIR}/src/honeycomb/chem/periodic_table_tensor.cpp
    ${CMAKE_SOURCE_DIR}/src/honeycomb/chem/reaction.cpp
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
    ${IMGUI_DIR}/backends/imgui_impl_metal.mm
    ${IMGUI_DIR}/backends/imgui_impl_osx.mm
)

add_executable(Honeycomb ${SOURCES})

target_include_directories(Honeycomb PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMGUI_DIR}/misc/cpp
)

# Disable gamepad support on macOS backend to avoid linking GameController framework
target_compile_definitions(Honeycomb PRIVATE IMGUI_IMPL_OSX_DISABLE_GAMEPAD)

# Enable higher warnings for clarity
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
    target_compile_options(Honeycomb PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(Honeycomb PRIVATE /W4 /permissive-)
endif()

# Link macOS frameworks for Metal + Cocoa
if(APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(METAL_LIBRARY Metal)
    find_library(METALKIT_LIBRARY MetalKit)
    find_library(QUARTZCORE_LIBRARY QuartzCore)
    find_library(GAMECONTROLLER_LIBRARY GameController)
    target_link_libraries(Honeycomb PRIVATE ${COCOA_LIBRARY} ${METAL_LIBRARY} ${METALKIT_LIBRARY} ${QUARTZCORE_LIBRARY} ${GAMECONTROLLER_LIBRARY})
endif()

# Copy runtime assets next to the built binary so relative paths work
add_custom_command(TARGET Honeycomb POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:Honeycomb>/assets)
