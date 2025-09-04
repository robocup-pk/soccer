<<<<<<< HEAD
add_executable(demo_rrtx DemoRRTX.cpp)
target_link_libraries(demo_rrtx PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)
=======
# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
<<<<<<< HEAD
>>>>>>> 2cdab88a (Temp Changes)

<<<<<<< HEAD
add_executable(demo_astar DemoAstar.cpp)
target_link_libraries(demo_astar PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)
=======
add_executable(demo_rrt_kick DemoRRTKick.cpp)
add_executable(basic_demo demo.cpp)
add_executable(demo_with_logging demo_with_logging.cpp)
add_executable(demo_kalman_logging demo_kalman_logging.cpp)
add_executable(performance_testing_demo performance_testing_demo.cpp)
# Link required libraries
target_link_libraries(demo_rrt_kick 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)
<<<<<<< HEAD
<<<<<<< HEAD
add_executable(demo_opponent_dribbling DemoOpponentDribbling.cpp)
target_link_libraries(demo_opponent_dribbling
    algos_lib
    state_lib
    vis_lib
    kin_lib
    rob_lib
    cfg_lib
    utils_lib
    Eigen3::Eigen
)
>>>>>>> 7f1fd930 (Add RRT opponent dribbling demo with new dribble util (#4))
=======
=======
add_executable(basic_demo demo.cpp)
# Link required libraries

<<<<<<< HEAD
>>>>>>> 97084d4f (Added Smooth uniform BSpline Trajectory Planner)

<<<<<<< HEAD
>>>>>>> 55c9ca2b (Kick, Dribble, Testing, Demos)
=======
=======
add_executable(simulate_log_file_demo simulate_log_file_demo.cpp)
>>>>>>> 01c08237 (Simulate BSpline on real data)
target_link_libraries(basic_demo 
=======
target_link_libraries(demo_with_logging 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)
target_link_libraries(simulate_log_file_demo 
>>>>>>> 376f91f0 (Improve BSpline Corner turns)
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
)
<<<<<<< HEAD
>>>>>>> 2cdab88a (Temp Changes)
=======

add_executable(demo_stable_waypoints demo_stable_waypoints.cpp)
target_link_libraries(demo_stable_waypoints 
=======
)
target_link_libraries(simulate_log_file_demo 
>>>>>>> 01c08237 (Simulate BSpline on real data)
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
<<<<<<< HEAD
)

add_executable(demo_trajectory_comparison demo_trajectory_comparison.cpp)
target_link_libraries(demo_trajectory_comparison 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)
<<<<<<< HEAD
>>>>>>> 924fc1b6 (BSpline Trajactory for Path Planning)
=======

add_executable(demo_virtual_robot_bspline demo_virtual_robot_bspline.cpp)
target_link_libraries(demo_virtual_robot_bspline 
=======
)
target_link_libraries(demo_kalman_logging 
>>>>>>> 95c268e7 (Debug Kalmans Filter)
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    est_lib
<<<<<<< HEAD
    Eigen3::Eigen
)

add_executable(demo_bspline_simple demo_bspline_simple.cpp)
target_link_libraries(demo_bspline_simple 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_friction demo_bspline_friction.cpp)
target_link_libraries(demo_bspline_friction 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_virtual_robot_fixed demo_virtual_robot_fixed.cpp)
target_link_libraries(demo_virtual_robot_fixed 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_replay_sensor_data demo_replay_sensor_data.cpp)
target_link_libraries(demo_replay_sensor_data 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_robust demo_bspline_robust.cpp)
target_link_libraries(demo_bspline_robust 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_simple_fixed demo_bspline_simple_fixed.cpp)
target_link_libraries(demo_bspline_simple_fixed 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_replanning_comparison demo_bspline_replanning_comparison.cpp)
target_link_libraries(demo_bspline_replanning_comparison 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    ctrl_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_replanning_with_state_estimation demo_bspline_replanning_with_state_estimation.cpp)
target_link_libraries(demo_bspline_replanning_with_state_estimation 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    ctrl_lib
    est_lib
    Eigen3::Eigen
)

add_executable(demo_rrt_bspline_visualization demo_rrt_bspline_visualization.cpp)
target_link_libraries(demo_rrt_bspline_visualization 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    ctrl_lib
    Eigen3::Eigen
)

add_executable(demo_bspline_corner_improved demo_bspline_corner_improved.cpp)
target_link_libraries(demo_bspline_corner_improved 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    ctrl_lib
    Eigen3::Eigen
)
>>>>>>> 19ac42f6 (BSpline Debugging & Testing)
=======
)
>>>>>>> 97084d4f (Added Smooth uniform BSpline Trajectory Planner)
=======
)
>>>>>>> 01c08237 (Simulate BSpline on real data)
=======
    hw_lib
    Eigen3::Eigen
)
<<<<<<< HEAD
>>>>>>> 95c268e7 (Debug Kalmans Filter)
=======
target_link_libraries(performance_testing_demo 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)
>>>>>>> 73c525b1 (Testing Trajectory Planner with Multiple Robot & Tracking error)
