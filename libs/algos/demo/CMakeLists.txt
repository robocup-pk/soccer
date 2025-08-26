add_executable(demo_rrtx DemoRRTX.cpp)
target_link_libraries(demo_rrtx PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)

add_executable(demo_astar DemoAstar.cpp)
target_link_libraries(demo_astar PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)