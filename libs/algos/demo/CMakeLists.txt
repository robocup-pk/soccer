# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

add_executable(demo_with_logging demo_with_logging.cpp)
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

# Multi-robot demo for testing trajectory planning with multiple agents
add_executable(multi_robot_demo multi_robot_demo.cpp)
target_link_libraries(multi_robot_demo 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib
    ctrl_lib
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)