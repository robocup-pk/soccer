cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
cmake_policy(SET CMP0072 NEW) # Suppress warnings related to opengl

project(robocup-pk)

# Ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Cmake build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

# Detect build platform (pi or pc)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64"))
    add_compile_definitions(BUILD_ON_PI)
else()
    add_compile_definitions(BUILD_ON_PC)
endif()

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR 
        "In-source builds are not allowed!\n"
        "Please create a separate build directory and run cmake from there:\n"
        "  mkdir build\n"
        "  cd build\n"
        "  cmake ..\n"
        "Current source dir: ${CMAKE_SOURCE_DIR}\n"
        "Current binary dir: ${CMAKE_BINARY_DIR}")
endif()

# To set the paths relative to build directory
add_compile_definitions(CMAKE_BUILD_DIR="${CMAKE_CURRENT_BINARY_DIR}")

# Tests
option(ENABLE_TESTS "Build unit tests" ON)
if (ENABLE_TESTS)
  enable_testing()
endif()

# Libraries
add_subdirectory(libs)

# Applications
add_subdirectory(apps)

# Test executables
add_executable(test_original_waypoints test_original_waypoints.cpp)
target_link_libraries(test_original_waypoints
    ctrl_lib
    utils_lib
    cfg_lib
)

# SSL Software - Remote Control
# docker pull robocupssl/ssl-remote-control
# docker run -p 8084:8084 robocupssl/ssl-remote-control
#
# 
#
#