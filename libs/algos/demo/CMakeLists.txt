add_executable(demo_rrtx DemoRRTX.cpp)
target_link_libraries(demo_rrtx PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)

<<<<<<< HEAD
add_executable(demo_astar DemoAstar.cpp)
target_link_libraries(demo_astar PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)
=======
add_executable(demo_rrt_kick DemoRRTKick.cpp)

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

>>>>>>> 55c9ca2b (Kick, Dribble, Testing, Demos)
