// cpp std libs
#include <iostream>
#include <chrono>
#include <algorithm>
#include <vector>
#include <cmath>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Algos.h"
#include "Coordinates.h"
#include "Waypoint.h"
#include "RRT.h"
#include "RRTX.h"
#include "Algos.h"
#include "RobotManager.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  GLFWwindow* window = gl_simulation.GetRawGLFW();
  gl_simulation.InitGameObjects(soccer_objects);

  Eigen::Vector3d lastballPos = soccer_objects[0].GetPosition();  // Bot
  state::Path path;
  state::Path path1;
  static int path_index = 0;

  rob::RobotManager robot_manager;
  bool first_time = true;

  while (1) {
    // Step 1: Calculate dt
    // --- Time Calculation ---
    auto current_time = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    // Step 2: Process Input
    vis::ProcessUserInput(window, soccer_objects);

    // Step 2: Plan planning step
    algos::AlgoName algo_name = algos::AlgoName::RRTX;
    if (first_time || robot_manager.FinishedMotion()) {
      first_time = false;
      state::Path path = algos::PlanPath(algo_name, soccer_objects);
      std::cout << "Path: " << path << std::endl;
      robot_manager.SetPath(path);
    }
    soccer_objects[1].position = robot_manager.GetPoseInWorldFrame();

    // Step 5: Simulation Step
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
