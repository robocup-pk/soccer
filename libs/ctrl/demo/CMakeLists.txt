cmake_minimum_required(VERSION 3.10)
project(demo_robotmanager)

add_executable(demo_robotmanager demo.cpp)

target_link_libraries(demo_robotmanager
    PRIVATE
        rob_lib
        vis_lib
        cfg_lib
        state_lib
        kin_lib
        utils_lib
        Eigen3::Eigen
        glad
        glfw
        glm
)

target_include_directories(demo_robotmanager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)