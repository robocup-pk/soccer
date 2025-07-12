cmake_minimum_required(VERSION 3.10)
project(comms_examples)

set(CMAKE_CXX_STANDARD 17)

# Add the comms library

# Include directories
include_directories(../include)

# File globbing for example source files
file(GLOB EXAMPLES "*.cpp")

# Build each example executable
foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXENAME ${EXAMPLE} NAME_WE)
    add_executable(${EXENAME} ${EXAMPLE})
    target_link_libraries(${EXENAME} comms_lib)
endforeach()

# Build test pair - for testing the communication
add_custom_target(test_comms
    COMMAND echo "Starting receiver in background..."
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/receiver_example &
    COMMAND echo "Starting sender..."
    COMMAND sleep 1
    COMMAND timeout 10 ${CMAKE_CURRENT_BINARY_DIR}/sender_example || true
    COMMAND pkill -f receiver_example || true
    DEPENDS sender_example receiver_example
    COMMENT "Test UDP communication (runs for 10 seconds)"
)

# Build test raw communication
add_custom_target(test_raw_comms
    COMMAND echo "Starting raw receiver in background..."
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/raw_receiver_example &
    COMMAND echo "Starting sender..."
    COMMAND sleep 1
    COMMAND timeout 5 ${CMAKE_CURRENT_BINARY_DIR}/sender_example || true
    COMMAND pkill -f raw_receiver_example || true
    DEPENDS sender_example raw_receiver_example
    COMMENT "Test raw UDP communication (runs for 5 seconds)"
)
