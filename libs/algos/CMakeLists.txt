file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(algos_lib ${SOURCES} ${HEADERS})
<<<<<<< HEAD
<<<<<<< HEAD
target_link_libraries(algos_lib PRIVATE utils_lib state_lib cfg_lib vis_lib kin_lib)
=======
target_link_libraries(algos_lib PRIVATE utils_lib state_lib cfg_lib vis_lib Eigen3::Eigen)
>>>>>>> 71e078e1 (Temp Changes)
=======
target_link_libraries(algos_lib PRIVATE utils_lib state_lib cfg_lib vis_lib)
>>>>>>> f58fc428 (Adding rrtx)
target_include_directories(algos_lib PUBLIC include)

# add_subdirectory(demo)
add_subdirectory(test)
add_subdirectory(demo)