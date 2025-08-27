set(SOURCES
    src/MotionController.cpp
    src/BSplineTrajectory.cpp
    src/BSplineTrajectoryManager.cpp
    src/UniformBSplineTrajectoryPlanner.cpp
    src/BezierTrajectoryPlanner.cpp
    src/DBRRTTrajectoryPlanner.cpp
    src/BangBangTrajectoryPlanner.cpp
)

file(GLOB_RECURSE HEADERS "include/*.h")

# Find Python for acceleration envelope integration
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_library(ctrl_lib ${SOURCES} ${HEADERS})
target_link_libraries(ctrl_lib PUBLIC cfg_lib utils_lib kin_lib Python3::Python)
target_include_directories(ctrl_lib PUBLIC include ${Python3_INCLUDE_DIRS})