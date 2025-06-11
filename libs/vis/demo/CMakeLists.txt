add_executable(gl_window DemoWindow.cpp)
target_link_libraries(gl_window PRIVATE glad glfw)

add_executable(demo_triangle DemoTriangle.cpp)
target_link_libraries(demo_triangle PRIVATE vis_lib glfw glad)