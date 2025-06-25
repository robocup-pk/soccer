file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(utils_lib ${SOURCES} ${HEADERS})
target_include_directories(utils_lib PUBLIC include)
target_link_libraries(utils_lib PUBLIC Eigen3::Eigen)

add_subdirectory(test)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")