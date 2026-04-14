cmake_minimum_required(VERSION 3.20)

project(satellite-detumble LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)

FetchContent_MakeAvailable(raylib)

add_executable(satellite-detumble
    src/main.cpp
    src/simulation.cpp
    src/ui.cpp
)

target_include_directories(satellite-detumble PRIVATE include/satellite-detumble)

target_link_libraries(rotation PRIVATE raylib)
target_link_libraries(satellite-detumble PRIVATE raylib)

if(MSVC)
    target_compile_options(rotation PRIVATE /W4)
else()
    target_compile_options(rotation PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(MSVC)
    target_compile_options(satellite-detumble PRIVATE /W4)
else()
    target_compile_options(satellite-detumble PRIVATE -Wall -Wextra -Wpedantic)
endif()