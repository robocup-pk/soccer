// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLWindow.h"

int main(int argc, char* argv[]) {
  vis::GLWindow gl_window;

  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  while (1) {
    // Calculate delta time
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Other Steps
    // Simulation Step
    if (!gl_window.RunSimulationStep(dt)) {
      std::cout << "[main::RunSimulationStep] Stopped" << std::endl;
      break;
    }
  }
  return 0;
}
