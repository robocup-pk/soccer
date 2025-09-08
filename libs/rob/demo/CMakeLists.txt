add_executable(demo_ball_physics_and_kicking DemoBallPhysicsAndDribbling.cpp)
target_link_libraries(demo_ball_physics_and_kicking 
    rob_lib 
    kin_lib 
    state_lib 
    vis_lib 
    cfg_lib 
    utils_lib
    Eigen3::Eigen
)