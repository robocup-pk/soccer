set(SOURCES
    src/RobotManager.cpp
    src/RobotManagerActions.cpp
)

file(GLOB_RECURSE HEADERS "include/*.h")

add_library(rob_lib ${SOURCES} ${HEADERS})
target_link_libraries(rob_lib PUBLIC cfg_lib utils_lib kin_lib hw_lib est_lib ctrl_lib)
target_include_directories(rob_lib PUBLIC include)

add_subdirectory(demo)
# add_subdirectory(test)