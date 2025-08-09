cmake_minimum_required(VERSION 3.20)
project(Honeycomb LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(Honeycomb
    src/main.cpp
)

target_include_directories(Honeycomb PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Enable higher warnings for clarity
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
    target_compile_options(Honeycomb PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(Honeycomb PRIVATE /W4 /permissive-)
endif()
