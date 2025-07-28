file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(ref_lib ${SOURCES} ${HEADERS})
target_link_libraries(ref_lib PUBLIC utils_lib state_lib cfg_lib vis_lib kin_lib)
target_include_directories(ref_lib PUBLIC include)
