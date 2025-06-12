// cpp std libs
#include <iostream>

// self libs
#include "GLWindow.h"

int main(int argc, char* argv[]) {
  constexpr int width_px = 800;
  constexpr int height_px = 800;
  vis::GLWindow gl_window;

  // To draw rectangle: /libs/vis/demo/DemoShape 1
  // To draw triangle: /libs/vis/demo/DemoShape
  bool draw_rectangle;
  if (argc == 2) {
    draw_rectangle = argv[1];
  }

  if (!gl_window.Init(width_px, height_px, "RoboCup Simulator", draw_rectangle)) {
    std::cout << "[DemoKeyCallback::main] Error!" << std::endl;
    return 0;
  }

  while (gl_window.RunSimulationStep()) {
  }

  // while(1) {
  //   glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  //   glClear(GL_COLOR_BUFFER_BIT);
  //   glUseProgram(gl_window.shader_program_id);
  //   glBindVertexArray(gl_window.vao);
  //   glDrawArrays(GL_TRIANGLES, 0, 3);
  //   glfwSwapBuffers(gl_window.GetRawGLFW());
  //   glfwPollEvents();
  // }

  return 0;
}
