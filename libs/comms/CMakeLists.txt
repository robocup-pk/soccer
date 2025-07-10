cmake_minimum_required(VERSION 3.10)
project(comms_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# collect .cpp
file(GLOB_RECURSE COMM_SOURCES src/*.cpp)

# collect .proto
file(GLOB_RECURSE PROTO_FILES proto/*.proto)
if (PROTO_FILES)
  message(STATUS "Generating protobuf for ${PROTO_FILES}")
  protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    PROTOS ${PROTO_FILES}
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto
  )
endif()

add_library(comms
  ${COMM_SOURCES}
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(comms PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}       # for generated .pb.h
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(comms PUBLIC
  ${Protobuf_LIBRARIES}
  Threads::Threads
)

# examples
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
  add_subdirectory(examples)
endif()