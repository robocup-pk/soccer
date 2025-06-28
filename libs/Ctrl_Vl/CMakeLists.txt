file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h")

add_library(Velocity ${SOURCES} ${HEADERS})
target_include_directories(Velocity PUBLIC include)

# add_subdirectory(test)