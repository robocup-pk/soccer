// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLWindow.h"
#include "RRT.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  vis::GLWindow gl_window;
  state::Path path;
  bool replan_required = false;

  while (1) {
    // Calculate delta time
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Other Steps
    // ...
    // ...

    // Path Planning Step
    if (replan_required) {
      replan_required = false;
      state::Waypoint start(0, 0);
      state::Waypoint goal(10, 10);
      path = algos::FindSinglePath(start, goal);
      std::cout << "Path: " << path << std::endl;
    }

    // Simulation Step
    if (!gl_window.RunSimulationStep(dt)) {
      std::cout << "[main::RunSimulationStep] Stopped" << std::endl;
      break;
    }
  }

  return 0;
}
