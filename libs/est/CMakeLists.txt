file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(est_lib ${SOURCES} ${HEADERS})
target_link_libraries(est_lib PUBLIC Eigen3::Eigen hw_lib kin_lib utils_lib hw_lib)
target_include_directories(est_lib PUBLIC include)

add_subdirectory(test)