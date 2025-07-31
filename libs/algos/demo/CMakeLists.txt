<<<<<<< HEAD
add_executable(demo_rrtx DemoRRTX.cpp)
target_link_libraries(demo_rrtx PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)
=======
# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
>>>>>>> 2cdab88a (Temp Changes)

<<<<<<< HEAD
add_executable(demo_astar DemoAstar.cpp)
target_link_libraries(demo_astar PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)
=======
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

<<<<<<< HEAD
>>>>>>> 55c9ca2b (Kick, Dribble, Testing, Demos)
=======
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
<<<<<<< HEAD
>>>>>>> 2cdab88a (Temp Changes)
=======

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
>>>>>>> 924fc1b6 (BSpline Trajactory for Path Planning)
