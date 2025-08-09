cmake_minimum_required(VERSION 3.20)
project(Honeycomb LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(Honeycomb
    src/main.cpp
)

target_include_directories(Honeycomb PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Install or copy assets for runtime
set(ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")
set(ASSETS_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/assets")
file(MAKE_DIRECTORY "${ASSETS_OUT_DIR}")
set(PERIODIC_TABLE_SOURCE "${ASSETS_DIR}/periodic_table.csv")
set(PERIODIC_TABLE_DEST   "${ASSETS_OUT_DIR}/elements.csv")
add_custom_command(TARGET Honeycomb POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${PERIODIC_TABLE_SOURCE}" "${PERIODIC_TABLE_DEST}")

# Enable higher warnings for clarity
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
    target_compile_options(Honeycomb PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(Honeycomb PRIVATE /W4 /permissive-)
endif()
