file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(ctrl_lib ${SOURCES} ${HEADERS})
target_link_libraries(ctrl_lib PUBLIC cfg_lib utils_lib kin_lib)
target_include_directories(ctrl_lib PUBLIC include)

add_subdirectory(test)