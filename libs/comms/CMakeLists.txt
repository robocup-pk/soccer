cmake_minimum_required(VERSION 3.10)
project(comms_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# 1) collect all .cpp sources under src/
file(GLOB_RECURSE COMM_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 2) find every .proto under proto/ and its subdirectories
file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

# count how many we found
list(LENGTH PROTO_FILES PROTO_COUNT)
message(STATUS "Found ${PROTO_COUNT} .proto files")
if(PROTO_COUNT GREATER 0)
  message(STATUS "Proto files: ${PROTO_FILES}")
endif()

# 3) only call protobuf_generate_cpp() when PROTO_COUNT > 0
if(PROTO_COUNT GREATER 0)
  # Generate protobuf files manually to preserve directory structure
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
  
  foreach(PROTO_FILE ${PROTO_FILES})
    # Get relative path from proto directory
    file(RELATIVE_PATH REL_PROTO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/proto" "${PROTO_FILE}")
    get_filename_component(PROTO_NAME "${REL_PROTO_PATH}" NAME_WE)
    get_filename_component(PROTO_DIR "${REL_PROTO_PATH}" DIRECTORY)
    
    # Set output paths
    if("${PROTO_DIR}" STREQUAL "")
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
    else()
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.h")
      # Create directory if it doesn't exist
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}")
    endif()
    
    # Add custom command to generate protobuf files
    add_custom_command(
      OUTPUT ${OUTPUT_CC} ${OUTPUT_H}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
           --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
           ${PROTO_FILE}
      DEPENDS ${PROTO_FILE}
      COMMENT "Generating protobuf files for ${REL_PROTO_PATH}"
      VERBATIM
    )
    
    list(APPEND PROTO_SRCS ${OUTPUT_CC})
    list(APPEND PROTO_HDRS ${OUTPUT_H})
  endforeach()
else()
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
endif()

# 4) build the library, including any generated .cc/.h
add_library(comms
  ${COMM_SOURCES}
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(comms PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}"
)

target_link_libraries(comms PUBLIC
  ${Protobuf_LIBRARIES}
  Threads::Threads
)

# 5) only descend into examples/ if it actually exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
  add_subdirectory(examples)
else()
  message(STATUS "No examples/ directory, skipping.")
endif()