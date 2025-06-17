file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(algos_lib ${SOURCES} ${HEADERS})
# target_link_libraries(algos_lib PRIVATE utils_lib)
target_include_directories(algos_lib PUBLIC include)

# add_subdirectory(test)