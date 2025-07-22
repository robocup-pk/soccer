# Demo executables for algos library

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