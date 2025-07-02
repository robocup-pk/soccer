// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Collision.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  rob::RobotManager robot_manager;

  Eigen::Vector3d velocity_fBody(-0.2, 0.2, 1);
  robot_manager.SetBodyVelocity(velocity_fBody);
  Eigen::Vector4d ticks;

  double est_start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

  // START SIMULATION
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

    // ESTIMATED POSITION
    soccer_objects[0].position = robot_manager.GetPose();
    std::cout << "pose est: " << std::fixed << std::setprecision(3)
              << soccer_objects[0].position.transpose() << std::endl;

    // TRUE POSITION
    elapsed_time_s = util::GetCurrentTime() - est_start_time;
    soccer_objects[1].position = velocity_fBody * elapsed_time_s;
    std::cout << "pose tru: " << soccer_objects[1].position.transpose() << std::endl;

    // state::CheckAndResolveCollisions(soccer_objects);

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
