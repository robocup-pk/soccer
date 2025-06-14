cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0072 NEW) # Suppress warnings related to opengl

project(robocup-pk)

# Ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Tests
option(ENABLE_TESTS "Build unit tests" ON)
if (ENABLE_TESTS)
  enable_testing()
endif()

# Libraries
add_subdirectory(libs)

# Applications
add_subdirectory(apps)

# SSL Software - Remote Control
# docker pull robocupssl/ssl-remote-control
# docker run -p 8084:8084 robocupssl/ssl-remote-control
#
# 
#
#