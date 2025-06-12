cmake_minimum_required(VERSION 3.10)

project(robocup-pk)

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