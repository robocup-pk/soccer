file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(state_lib ${SOURCES} ${HEADERS})
target_link_libraries(state_lib PUBLIC cfg_lib rob_lib)
target_include_directories(state_lib PUBLIC include)

# add_subdirectory(test)