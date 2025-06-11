add_executable(gl_window DemoWindow.cpp)
target_link_libraries(gl_window PRIVATE glad glfw)

add_executable(gl_window_key DemoKeyCallback.cpp)
target_link_libraries(gl_window_key PRIVATE vis_lib glfw glad)