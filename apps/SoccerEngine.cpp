// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Collision.h"
#include "Coordinates.h"

void UpdateKinematics(std::vector<state::SoccerObject>& soccer_objects, float dt) {
  for (state::SoccerObject& soccer_object : soccer_objects) {
    soccer_object.Move(dt);
  }
}

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  while (1) {
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Other Steps
    // ...
    // ...

    state::CheckAndResolveCollisions(soccer_objects);

    // Simulation Step
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }

    // Error Checking
    // ResolveErrors();
  }

  return 0;
}
