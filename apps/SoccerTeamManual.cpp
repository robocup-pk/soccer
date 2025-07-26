#include <algorithm>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Utils.h"
#include "Coordinates.h"
#include "RobotManager.h"
#include "SoccerField.h"
#include "AutoRef.h"

int main(int argc, char* argv[]) {
  // TODO: Game State
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  if (soccer_objects.size() > 12) {
    for (int i = 0; i < soccer_objects.size() / 2; i++) {
      soccer_objects[i].position = cfg::SystemConfig::team_one_start_formation[i];
      soccer_objects[i + soccer_objects.size() / 2].position =
          cfg::SystemConfig::team_two_start_formation[i];
    }
  }

  // Simulation - OpenGL
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjectsTwoTeams(soccer_objects);
  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

  while (1) {
    float dt = util::CalculateDt();

    vis::ProcessInputTwoTeams(gl_window, soccer_objects);
    kin::UpdateKinematics(soccer_objects, dt);

    // ref::CheckCollisions(soccer_objects);
    kin::CheckAndResolveCollisions(soccer_objects);

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }
  return 0;
}