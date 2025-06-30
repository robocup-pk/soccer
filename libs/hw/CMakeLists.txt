# Recursively search for header and source files in the include and src directories
file(GLOB_RECURSE HEADERS "include/*.h")
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create a library named hw_lib using the headers and sources found
add_library(hw_lib ${HEADERS} ${SOURCES})

# Specify the include directory for the hw_lib target
target_include_directories(hw_lib PUBLIC include)
target_link_libraries(hw_lib PUBLIC Eigen3::Eigen kin_lib)

# Add test subdirectory
add_subdirectory(test)