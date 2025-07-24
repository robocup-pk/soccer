file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(cfg_lib ${SOURCES} ${HEADERS})
target_link_libraries(cfg_lib PUBLIC utils_lib vis_lib glm glad Eigen3::Eigen)
target_include_directories (cfg_lib PUBLIC include)

# add_subdirectory(test)