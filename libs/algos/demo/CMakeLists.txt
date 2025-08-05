add_executable(demo_rrtx DemoRRTX.cpp)
target_link_libraries(demo_rrtx PRIVATE state_lib vis_lib cfg_lib rob_lib algos_lib)