# Explicitly list sources to exclude RRT files that have global static state
set(SOURCES
    src/AStar.cpp
    src/Grid.cpp
    # src/RRT.cpp - EXCLUDED due to global static state causing on-off behavior
    # src/RRTX.cpp - EXCLUDED due to global static state causing on-off behavior
)

file(GLOB_RECURSE HEADERS "include/*.h")

add_library(algos_lib ${SOURCES} ${HEADERS})
target_link_libraries(algos_lib PRIVATE utils_lib state_lib cfg_lib vis_lib kin_lib)
target_include_directories(algos_lib PUBLIC include)

# add_subdirectory(demo)
add_subdirectory(test)
add_subdirectory(demo)