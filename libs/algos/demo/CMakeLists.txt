# Demo executables for algos library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

# Automatically find all .cpp files and create executables (exclude RRT demos)
file(GLOB_RECURSE DEMO_SOURCES "*.cpp")
list(REMOVE_ITEM DEMO_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/DemoRRTX.cpp
)

foreach(DEMO_SOURCE ${DEMO_SOURCES})
    get_filename_component(DEMO_NAME ${DEMO_SOURCE} NAME_WE)
    add_executable(${DEMO_NAME} ${DEMO_SOURCE})
    target_link_libraries(${DEMO_NAME}
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
endforeach()