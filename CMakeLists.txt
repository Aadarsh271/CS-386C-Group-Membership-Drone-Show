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
#                                 AUTO TESTS                                   #
# ============================================================================ #

# Collect core logic files (NO visualization / OpenGL)
file(GLOB_RECURSE TEST_SOURCES
    "${CMAKE_SOURCE_DIR}/src/core/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/network/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/membership/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/control/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/util/*.cpp"
)

# Collect every .cpp test file in tests/
file(GLOB TEST_FILES "${CMAKE_SOURCE_DIR}/tests/*.cpp")

# Ensure glm is available
find_package(glm CONFIG REQUIRED)

foreach(test_src ${TEST_FILES})

    # Extract the test name (filename without extension)
    get_filename_component(test_name ${test_src} NAME_WE)

    # Make an executable for this test
    add_executable(${test_name}
        ${TEST_SOURCES}
        ${test_src}
    )

    # Include dirs
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${stb_SOURCE_DIR}                # allow image-based tests
    )

    # Link dependencies
    target_link_libraries(${test_name} PRIVATE glm::glm)

    # Put all test binaries in the "Tests" folder in IDEs
    set_target_properties(${test_name} PROPERTIES FOLDER "Tests")

endforeach()

# Organize tests and sources in IDE
source_group(TREE ${CMAKE_SOURCE_DIR}/tests PREFIX "Tests" FILES ${TEST_FILES})
source_group(TREE ${CMAKE_SOURCE_DIR}/src PREFIX "TestSources" FILES ${TEST_SOURCES})
