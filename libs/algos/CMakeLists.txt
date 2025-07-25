set(SOURCES src/Algos.cpp src/Astar.cpp src/RRT.cpp src/RRTX.cpp)
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(algos_lib ${SOURCES} ${HEADERS})
target_link_libraries(algos_lib PRIVATE utils_lib state_lib cfg_lib vis_lib Eigen3::Eigen)
target_include_directories(algos_lib PUBLIC include)

# add_subdirectory(demo)
add_subdirectory(test)
