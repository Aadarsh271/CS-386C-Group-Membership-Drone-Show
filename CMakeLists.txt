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
        GIT_TAG v2.3.0
    )
    FetchContent_MakeAvailable(polyscope)

    target_link_libraries(drone-show PRIVATE polyscope)
    target_compile_definitions(drone-show PRIVATE ENABLE_POLYSCOPE)
endif()
# ----------------------------------------------------------------------------
# stb_image FetchContent
# ----------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
)
FetchContent_MakeAvailable(stb)

# stb is header-only; just add the include directory
target_include_directories(drone-show PRIVATE ${stb_SOURCE_DIR})

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

# ============================================================================ #
#                            UNIT TEST EXECUTABLE                              #
# ============================================================================ #

# Collect only the core logic (NO Polyscope/OpenGL)
file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/network/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/membership/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/control/*.cpp" 
    "${CMAKE_SOURCE_DIR}/src/util/*.cpp"
)

set(TEST_MAIN
    ${CMAKE_SOURCE_DIR}/tests/MembershipReconfigHarness.cpp
)
add_executable(membership-tests
    ${TEST_SOURCES}
    ${TEST_MAIN}
)

add_executable(formation-tests
    ${TEST_SOURCES}
    ${CMAKE_SOURCE_DIR}/tests/FormationAssignmentHarness.cpp
)

target_include_directories(membership-tests PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(formation-tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_include_directories(formation-tests PRIVATE ${stb_SOURCE_DIR})

# ---- vcpkg dependencies needed for tests ----
find_package(glm CONFIG REQUIRED)
target_link_libraries(membership-tests PRIVATE glm::glm)
target_link_libraries(formation-tests PRIVATE glm::glm)

# Optional: organize in IDE
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "TestSources" FILES ${TEST_SOURCES})
source_group(TREE ${CMAKE_SOURCE_DIR}/tests PREFIX "Tests" FILES ${TEST_MAIN})
