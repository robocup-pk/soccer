# Trapezoidal trajectory system files
set(TRAPEZOIDAL_SOURCES
    trapezoidal_trajectory/src/MotionController.cpp
    trapezoidal_trajectory/src/TrajectoryManager.cpp
    trapezoidal_trajectory/src/TrajectoryMergerLogic.cpp
    trapezoidal_trajectory/src/TrapezoidalTrajectoryVi3D.cpp
    trapezoidal_trajectory/src/Trajectory3D.cpp
)

# BSpline trajectory system files
set(BSPLINE_SOURCES
    BSpline_trajectory/src/UniformBSplineTrajectoryPlanner.cpp
)

# Advanced trajectory system files (BangBang and path planning)
set(ADVANCED_SOURCES
    advanced_trajectory/src/PlanarCurveSegment.cpp
    advanced_trajectory/src/PlanarCurve.cpp
    advanced_trajectory/src/PlanarCurveFactory.cpp
    advanced_trajectory/src/DestinationForTimedPositionCalc.cpp
    advanced_trajectory/src/AdvanceMotionPlanner.cpp
    advanced_trajectory/src/TrajectoryTracker.cpp
    advanced_trajectory/src/TrajPath.cpp
    advanced_trajectory/src/PathFinder.cpp
    advanced_trajectory/src/PathCollisionChecker.cpp
    advanced_trajectory/src/ObstacleCollisionChecker.cpp
    advanced_trajectory/src/PathResultAcceptor.cpp
    advanced_trajectory/src/TrajectoryGenerator.cpp
    advanced_trajectory/src/BangBangTrajectory1D.cpp
    advanced_trajectory/src/BangBangTrajectory2D.cpp
    advanced_trajectory/src/BangBangTrajectory1DOrient.cpp
    advanced_trajectory/src/BangBangTrajectory2DAsync.cpp
    advanced_trajectory/src/BangBangTrajectoryFactory.cpp
    advanced_trajectory/src/TrajectoryXyw.cpp
    advanced_trajectory/src/ReplanningController.cpp
)

# Collect all headers from all systems
file(GLOB_RECURSE TRAPEZOIDAL_HEADERS "trapezoidal_trajectory/include/*.h")
file(GLOB_RECURSE BSPLINE_HEADERS "BSpline_trajectory/include/*.h")
file(GLOB_RECURSE ADVANCED_HEADERS "advanced_trajectory/include/*.h")

# Find Python for acceleration envelope integration
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_library(ctrl_lib ${TRAPEZOIDAL_SOURCES} ${BSPLINE_SOURCES} ${ADVANCED_SOURCES} 
            ${TRAPEZOIDAL_HEADERS} ${BSPLINE_HEADERS} ${ADVANCED_HEADERS})
target_link_libraries(ctrl_lib PUBLIC cfg_lib utils_lib kin_lib Python3::Python)
target_include_directories(ctrl_lib PUBLIC 
    trapezoidal_trajectory/include 
    BSpline_trajectory/include
    advanced_trajectory/include 
    ../kin/include
    ${Python3_INCLUDE_DIRS}
)