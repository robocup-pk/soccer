# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

add_executable(demo_rrt_kick DemoRRTKick.cpp)
add_executable(basic_demo demo.cpp)
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

target_link_libraries(basic_demo 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)

add_executable(demo_stable_waypoints demo_stable_waypoints.cpp)
target_link_libraries(demo_stable_waypoints 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
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

add_executable(demo_virtual_robot_bspline demo_virtual_robot_bspline.cpp)
target_link_libraries(demo_virtual_robot_bspline 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    est_lib
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