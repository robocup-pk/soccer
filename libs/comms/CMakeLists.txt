# Find locally installed protobuf
find_package(Protobuf REQUIRED)
find_package(absl QUIET)


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

# Generate proto sources if any
if(PROTO_FILES)
  set(PROTO_SRCS "")
  set(PROTO_HDRS "")
  
  foreach(PROTO_FILE ${PROTO_FILES})
    file(RELATIVE_PATH REL_PROTO_PATH "${CMAKE_CURRENT_SOURCE_DIR}/proto" "${PROTO_FILE}")
    get_filename_component(PROTO_NAME "${REL_PROTO_PATH}" NAME_WE)
    get_filename_component(PROTO_DIR "${REL_PROTO_PATH}" DIRECTORY)

    if("${PROTO_DIR}" STREQUAL "")
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H  "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
    else()
      set(OUTPUT_CC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.cc")
      set(OUTPUT_H  "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}/${PROTO_NAME}.pb.h")
      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR}")
    endif()

    add_custom_command(
      OUTPUT ${OUTPUT_CC} ${OUTPUT_H}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
           --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
           ${PROTO_FILE}
      DEPENDS ${PROTO_FILE}
      COMMENT "Generating protobuf for ${REL_PROTO_PATH}"
      VERBATIM
    )
    list(APPEND PROTO_SRCS ${OUTPUT_CC})
    list(APPEND PROTO_HDRS ${OUTPUT_H})
  endforeach()
endif()

# 4) Build the comms_lib library
add_library(comms_lib
  ${COMM_SOURCES}
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

# Include directories
target_include_directories(comms_lib PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_BINARY_DIR}"       # for generated .pb.h
  ${Protobuf_INCLUDE_DIRS}
)

# Link libraries
if(absl_FOUND)
    message(STATUS "Using Abseil libraries with Protobuf")
    target_link_libraries(comms_lib PUBLIC
        ${Protobuf_LIBRARIES}  
        Threads::Threads
        absl::log
        absl::log_internal_check_op
        absl::strings
        absl::base
    )
else()
    message(STATUS "Abseil not found - linking Protobuf without Abseil")
    target_link_libraries(comms_lib PUBLIC
        ${Protobuf_LIBRARIES}  
        Threads::Threads
    )
endif()

# 5) Examples (if any)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()