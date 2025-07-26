# Recursively search for header and source files in the include and src directories
set(HEADERS
    include/ActionExecutor.h
    include/DribbleExecutor.h
    include/KickExecutor.h
    include/Dribble.h
    include/Kick.h
    include/Kinematics.h
    include/RobotModel.h
)

set(SOURCES
    src/ActionExecutor.cpp
    src/DribbleExecutor.cpp
    src/KickExecutor.cpp
    src/Dribble.cpp
    src/Kick.cpp
    src/Kinematics.cpp
    src/RobotModel.cpp
)

# Create a library named kin_lib using the headers and sources found
add_library(kin_lib ${HEADERS} ${SOURCES})

# Specify the include directory for the kin_lib target
target_include_directories(kin_lib PUBLIC include)
target_link_libraries(kin_lib PUBLIC Eigen3::Eigen state_lib vis_lib)

# Add test subdirectory
add_subdirectory(test)

# Add demo subdirectory
add_subdirectory(demo)