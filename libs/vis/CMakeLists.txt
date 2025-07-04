file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(vis_lib ${SOURCES} ${HEADERS})
target_link_libraries(vis_lib PUBLIC glad glfw glm utils_lib cfg_lib state_lib kin_lib)
target_include_directories(vis_lib PUBLIC include)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

add_subdirectory(demo)
add_subdirectory(test)