# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
add_executable(basic_demo demo.cpp)
# Link required libraries

add_executable(simulate_log_file_demo simulate_log_file_demo.cpp)
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
target_link_libraries(simulate_log_file_demo 
    algos_lib 
    state_lib 
    vis_lib 
    kin_lib 
    rob_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)