cmake_minimum_required(VERSION 3.16)
project(drone-show LANGUAGES CXX)

# ---- C++ Standard ----
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Default Build Type ----
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Release or Debug" FORCE)
endif()

# ---- Warnings ----
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Include Directories ----
include_directories(${CMAKE_SOURCE_DIR}/include)

# ---- Source Files ----
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# ---- Main Executable ----
add_executable(drone-show ${SOURCES})

# ---- Visualization Backend Options ----
option(ENABLE_POLYSCOPE "Enable Polyscope visualization" ON)
option(ENABLE_OPENGL "Enable raw OpenGL visualization" OFF)

# ----------------------------------------------------------------------------
# Polyscope FetchContent
# ----------------------------------------------------------------------------
if(ENABLE_POLYSCOPE)
    message(STATUS "Fetching Polyscope...")

    include(FetchContent)
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG v2.3.0  # Stable recent release
    )
    FetchContent_MakeAvailable(polyscope)

    target_link_libraries(drone-show PRIVATE polyscope)
    target_compile_definitions(drone-show PRIVATE ENABLE_POLYSCOPE)
endif()

# ----------------------------------------------------------------------------
# Optional OpenGL Renderer (later)
# ----------------------------------------------------------------------------
if(ENABLE_OPENGL)
    message(STATUS "OpenGL renderer ENABLED")

    find_package(OpenGL REQUIRED)
    find_package(glfw3 CONFIG REQUIRED)

    target_link_libraries(drone-show PRIVATE OpenGL::GL glfw)
    target_compile_definitions(drone-show PRIVATE ENABLE_OPENGL)
endif()

# ---- IDE Organization ----
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "Source" FILES ${SOURCES})

# ---- MSVC Runtime Fix ----
if(MSVC)
    set_target_properties(drone-show PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()
