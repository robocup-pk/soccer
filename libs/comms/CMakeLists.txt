# Find locally installed protobuf
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# Verify protobuf was found
if(NOT Protobuf_FOUND)
  message(FATAL_ERROR "Protobuf not found! Make sure it's installed locally.")
endif()

message(STATUS "Using Protobuf version: ${Protobuf_VERSION}")
message(STATUS "Protobuf compiler: ${Protobuf_PROTOC_EXECUTABLE}")
message(STATUS "Protobuf libraries: ${Protobuf_LIBRARIES}")

# 1) Collect all .cpp sources under src/
file(GLOB_RECURSE COMM_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# 2) Find every .proto under proto/ and its subdirectories
file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

# Count how many we found
list(LENGTH PROTO_FILES PROTO_COUNT)
message(STATUS "Found ${PROTO_COUNT} .proto files")

if(PROTO_COUNT GREATER 0)
  message(STATUS "Generating protobuf for: ${PROTO_FILES}")
  
  # Generate protobuf files manually to preserve directory structure
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
  
  foreach(PROTO_FILE ${PROTO_FILES})
    # Get relative path from proto directory
    file(RELATIVE_PATH REL_PROTO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/proto" "${PROTO_FILE}")
    get_filename_component(PROTO_NAME "${REL_PROTO_PATH}" NAME_WE)
    get_filename_component(PROTO_DIR "${REL_PROTO_PATH}" DIRECTORY)
    
    # Set output paths (preserve directory structure)
    if("${PROTO_DIR}" STREQUAL "")
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
    else()
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.h")
      # Create directory if it doesn't exist
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}")
    endif()
    
    # Add custom command to generate protobuf files using local protoc
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
  message(STATUS "No .proto files found, skipping protobuf generation")
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
endif()

# 4) Build the library, including any generated .cc/.h
add_library(comms
  ${COMM_SOURCES}
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

# Include directories - make sure to include protobuf headers
target_include_directories(comms PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}"        # for generated .pb.h files
  ${Protobuf_INCLUDE_DIRS}             # local protobuf headers
)

# Link libraries - use local protobuf
target_link_libraries(comms PUBLIC
  ${Protobuf_LIBRARIES}               # local protobuf libraries
  Threads::Threads
)

# Set C++ standard if needed for protobuf compatibility
if(NOT CMAKE_CXX_STANDARD)
  set_property(TARGET comms PROPERTY CXX_STANDARD 17)
endif()

# 5) Only descend into examples/ if it actually exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
  message(STATUS "Adding examples subdirectory")
  add_subdirectory(examples)
else()
  message(STATUS "No examples/ directory found, skipping.")
endif()