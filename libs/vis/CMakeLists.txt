file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(vis_lib ${SOURCES} ${HEADERS})
target_link_libraries(vis_lib PRIVATE glad glfw)
target_include_directories(vis_lib PUBLIC include)

add_subdirectory(demo)
add_subdirectory(test)