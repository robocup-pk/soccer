set(SOURCES
    src/PlanarCurveSegment.cpp
    src/PlanarCurve.cpp
    src/AdvanceMotionPlanner.cpp
    src/TrajectoryTracker.cpp
    src/BangBangTrajectory1D.cpp
    src/BangBangTrajectory2D.cpp
    src/BangBangTrajectory1DOrient.cpp
    src/BangBangTrajectory2DAsync.cpp
    src/BangBangTrajectoryFactory.cpp
)

file(GLOB_RECURSE HEADERS "include/*.h")

# Find Python for acceleration envelope integration
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_library(ctrl_lib ${SOURCES} ${HEADERS})
target_link_libraries(ctrl_lib PUBLIC cfg_lib utils_lib kin_lib Python3::Python)
target_include_directories(ctrl_lib PUBLIC include ${Python3_INCLUDE_DIRS})