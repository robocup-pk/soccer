// cpp std libs
#include <iostream>

// self libs
#include "GLWindow.h"

int main(int argc, char* argv[]) {
  constexpr int width_px = 800;
  constexpr int height_px = 600;
  vis::GLWindow gl_window;

  gl_window.Init(width_px, height_px, "RoboCup Simulator");

  while (gl_window.Update()) {
  }

  return 0;
}
