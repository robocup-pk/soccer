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