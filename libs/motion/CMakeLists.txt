file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(motion_lib ${SOURCES} ${HEADERS})
target_link_libraries(motion_lib PRIVATE glm utils_lib cfg_lib algos_lib vis_lib)
target_include_directories(motion_lib PUBLIC include ../vis/include)