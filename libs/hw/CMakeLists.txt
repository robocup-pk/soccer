file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(hw_lib ${SOURCES} ${HEADERS})
target_link_libraries(hw_lib PUBLIC libserial_static)
target_include_directories(hw_lib PUBLIC include)

# add_subdirectory(test)