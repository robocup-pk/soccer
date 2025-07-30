set(SOURCES
    src/AccelerationEnvelope.cpp
    src/BangBangTrajectory.cpp
    src/BangBangTrajectory3D.cpp
    src/MotionController.cpp
    src/Trajectory3D.cpp
    src/TrajectoryManager.cpp
    src/TrajectoryMergeLogic.cpp
    src/M_TrajectoryPlanner.cpp
    src/M_TrajectoryController.cpp
)

file(GLOB_RECURSE HEADERS "include/*.h")

# Find Python for acceleration envelope integration
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_library(ctrl_lib ${SOURCES} ${HEADERS})
target_link_libraries(ctrl_lib PUBLIC cfg_lib utils_lib kin_lib Python3::Python)
target_include_directories(ctrl_lib PUBLIC include ${Python3_INCLUDE_DIRS})

add_subdirectory(test)