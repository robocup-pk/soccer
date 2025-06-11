cmake_minimum_required(VERSION 3.10)

project(robocup-pk)

enable_testing()

# Libraries
add_subdirectory(libs)

# Applications
add_subdirectory(apps)

# Tests
option(ENABLE_TESTS "Build unit tests" ON)
if (ENABLE_TESTS)
endif()