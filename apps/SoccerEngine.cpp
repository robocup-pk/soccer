// cpp std libs
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

  // Simulation - OpenGL
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);
  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

  // Robot(s)
  std::vector<rob::RobotManager> robot_managers(cfg::SystemConfig::num_robots);

  // Initializing Robot Managers Positions using SoccerObjects to avoid conflicts
  for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    robot_managers[i].SetPoseInWorldFrame(soccer_objects[i].position);
  }

  while (1) {
    float dt = util::CalculateDt();

    vis::ProcessInput(gl_window, robot_managers);

    // Update SoccerObjects using robot managers
    for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
      soccer_objects[i] = robot_managers[i];
    }

    // Step 1: Use SoccerObjects to Compute Plan
    // std::vector<std::vector<Eigen::Vector3d>> plan = algos::ComputePlan(soccer_objects);

    // Step 2: Send plan to robot managers
    // for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    //   robot_managers[i].SetPath(plan[i]);
    // }

    // Referee
    // Step: Rule checking on current soccer objects
    // 1. Kinematics (Collisions, Max speeds)
    // kin::CheckAndResolveCollisions(soccer_objects);
    ref::CheckCollisions(soccer_objects);
    ref::CheckForGoals(soccer_objects);
    // 2.
    // 3.

    // Step: Simulation
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
