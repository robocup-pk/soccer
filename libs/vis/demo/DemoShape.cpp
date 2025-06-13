// cpp std libs
#include <iostream>

// self libs
#include "GLWindow.h"

int main(int argc, char* argv[]) {
  vis::GLWindow gl_window;

  while (1) {
    // Other Steps

    // Simulation Step
    if (!gl_window.RunSimulationStep()) {
      std::cout << "[main::RunSimulationStep] Stopped" << std::endl;
      break;
    }
  }

  return 0;
}
