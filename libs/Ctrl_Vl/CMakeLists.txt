cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0072 NEW)

project(Demo-t)
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(Velocity ${SOURCES} ${HEADERS})
target_include_directories(Velocity PUBLIC include)

add_subdirectory(Demo_app)