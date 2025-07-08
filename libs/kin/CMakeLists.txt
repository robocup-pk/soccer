# Recursively search for header and source files in the include and src directories
file(GLOB_RECURSE HEADERS "include/*.h")
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create a library named kin_lib using the headers and sources found
add_library(kin_lib ${HEADERS} ${SOURCES})

# Specify the include directory for the kin_lib target
target_include_directories(kin_lib PUBLIC include)
target_link_libraries(kin_lib PUBLIC Eigen3::Eigen state_lib vis_lib)

# Add test subdirectory
add_subdirectory(test)