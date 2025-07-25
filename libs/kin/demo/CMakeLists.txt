set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

# Demo: KickDemo - RRT path planning with trapezoidal trajectory and kick
add_executable(kick_demo kickdemo.cpp)

# Demo: DribblingDemo - RRT path planning with dribbling functionality
#add_executable(dribbling_demo dribblingdemo.cpp)

# Link required libraries for both demos
target_link_libraries(kick_demo 
    kin_lib
    state_lib 
    vis_lib
    algos_lib
    ctrl_lib
    cfg_lib
    utils_lib
    rob_lib
    glfw
    glad
    Eigen3::Eigen
)

# target_link_libraries(dribbling_demo 
#     kin_lib
#     state_lib 
#     vis_lib
#     algos_lib
#     ctrl_lib
#     cfg_lib
#     utils_lib
#     rob_lib
#     glfw
#     glad
#     Eigen3::Eigen
# )

# Include directories for both demos
target_include_directories(kick_demo PRIVATE
    ../include
    ../../state/include
    ../../vis/include
    ../../algos/include
    ../../ctrl/include
    ../../cfg/include
    ../../utils/include
    ../../rob/include
    ../../extern/eigen
)

# target_include_directories(dribbling_demo PRIVATE
#     ../include
#     ../../state/include
#     ../../vis/include
#     ../../algos/include
#     ../../ctrl/include
#     ../../cfg/include
#     ../../utils/include
#     ../../rob/include
#     ../../extern/eigen
# )
