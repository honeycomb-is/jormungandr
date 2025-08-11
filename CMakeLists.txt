cmake_minimum_required(VERSION 3.20)
project(Honeycomb LANGUAGES CXX OBJCXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to Debug for development
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Development vs Production optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "🚀 PRODUCTION BUILD - Optimizing for size and performance")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -flto -ffunction-sections -fdata-sections")
    set(CMAKE_OBJCXX_FLAGS_RELEASE "-O3 -DNDEBUG -flto -ffunction-sections -fdata-sections") 
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto -Wl,-dead_strip -Wl,-x")
else()
    message(STATUS "🛠️  DEVELOPMENT BUILD - Fast compilation, debug symbols")
endif()

# Find packages
find_package(glfw3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)

# ImGui sources
set(IMGUI_DIR packages/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_metal.mm
)

# Include imgui_demo.cpp only in development builds
if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND IMGUI_SOURCES ${IMGUI_DIR}/imgui_demo.cpp)
    message(STATUS "📝 Including ImGui demo (development build)")
endif()

# Application sources
add_executable(Honeycomb
    src/main.mm
    src/Honeycomb.cpp
    src/engine/math/Mat4.hpp
    src/engine/ecs/Components.hpp
    src/engine/gfx/MeshFactory.hpp
    src/engine/scene/Scene.hpp
    src/engine/scene/SceneBuilder.hpp
    src/engine/render/RenderPrimitives.hpp
    src/engine/species/Entities.hpp
    ${IMGUI_SOURCES}
)

target_include_directories(Honeycomb PRIVATE
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    src
)

# Build-type specific definitions
target_compile_definitions(Honeycomb PRIVATE 
    # Debug: use workspace assets dir; Release: we will prefer bundle Resources via NSBundle
    HONEYCOMB_ASSETS_DIR="${CMAKE_SOURCE_DIR}/assets"
    $<$<CONFIG:Release>:IMGUI_DISABLE_DEMO_WINDOWS>
    $<$<CONFIG:Release>:IMGUI_DISABLE_DEBUG_TOOLS>
)

target_compile_options(Honeycomb PRIVATE
    $<$<COMPILE_LANGUAGE:OBJCXX>:-Wno-arc-bridge-casts-disallowed-in-nonarc>
    $<$<CONFIG:Release>:-Os>              # Optimize for size in release
    $<$<CONFIG:Release>:-fvisibility=hidden>  # Hide symbols in release
)

target_link_libraries(Honeycomb
    glfw
    "-framework Metal"
    "-framework MetalKit"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
    "-framework QuartzCore"
    "-framework Foundation"
)

# Post-build optimization for release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET Honeycomb POST_BUILD
        COMMAND strip -x "$<TARGET_FILE:Honeycomb>"
        COMMENT "🔧 Stripping symbols for production build"
    )

    # Produce a simple .app bundle layout into dist for distribution using a template Info.plist
    add_custom_command(TARGET Honeycomb POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/dist
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/dist/Honeycomb.app/Contents/MacOS
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/dist/Honeycomb.app/Contents/Resources
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:Honeycomb> ${CMAKE_BINARY_DIR}/dist/Honeycomb.app/Contents/MacOS/
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/dist/Honeycomb.app/Contents/Resources/assets
        COMMAND ${CMAKE_COMMAND} -DINPUT=${CMAKE_SOURCE_DIR}/cmake/Info.plist.in -DOUTPUT=${CMAKE_BINARY_DIR}/dist/Honeycomb.app/Contents/Info.plist -DBUNDLE_NAME=Honeycomb -DBUNDLE_ID=com.honeycomb.app -DBUNDLE_VERSION=1.0 -DMIN_SYS_VER=10.15 -P ${CMAKE_SOURCE_DIR}/cmake/ConfigurePlist.cmake
        COMMENT "📦 Creating dist/Honeycomb.app with embedded assets"
    )
endif()

# Print build info
add_custom_command(TARGET Honeycomb POST_BUILD
    COMMAND echo "📦 Built: $<TARGET_FILE:Honeycomb>"
    COMMAND ls -lh "$<TARGET_FILE:Honeycomb>"
    VERBATIM
)