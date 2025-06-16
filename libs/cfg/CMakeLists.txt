file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(cfg_lib ${SOURCES} ${HEADERS})
# target_link_libraries(cfg_lib PRIVA)
target_include_directories(cfg_lib PUBLIC include)

# add_subdirectory(test)