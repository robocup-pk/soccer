# macOS Serial Port Implementation
# This provides a LibSerial-compatible interface for macOS

set(HEADERS
    include/libserial/SerialPort.h
)

set(SOURCES
    src/SerialPort.cpp
)

add_library(macos_serial_lib ${HEADERS} ${SOURCES})

target_include_directories(macos_serial_lib PUBLIC include)

# Link with required macOS frameworks
if(APPLE)
    find_library(COREFOUNDATION_LIBRARY CoreFoundation)
    find_library(IOKIT_LIBRARY IOKit)
    target_link_libraries(macos_serial_lib PUBLIC ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
endif()