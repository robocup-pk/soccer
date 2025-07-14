// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"

int main(int argc, char* argv[]) {
  // Need two robots for this demo to run
  if (cfg::SystemConfig::num_robots != 1) {
    std::cout << "[DemoMotionController::Main] Set num_robots to 1. Exiting!" << std::endl;
    return 0;
  }

  // START SIMULATION
  // std::vector<state::SoccerObject> soccer_objects;
  // state::InitSoccerObjects(soccer_objects);
  // vis::GLSimulation gl_simulation;
  // gl_simulation.InitGameObjects(soccer_objects);

  // ROBOT
  std::vector<Eigen::Vector3d> path;
  path.push_back(Eigen::Vector3d(0, 0, 0));
  path.push_back(Eigen::Vector3d(-1, 0, 0));
  // path.push_back(Eigen::Vector3d(-1, 1, 0));
  // path.push_back(Eigen::Vector3d(0, 1, 0));
  // path.push_back(Eigen::Vector3d(0, 0, 0));

  rob::RobotManager robot_manager;
  robot_manager.SetPath(path);

  while (1) {
    // soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();

    // // Simulation
    // if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
    //   std::cout << "[main] Simulation finished" << std::endl;
    //   break;
    // }
    util::WaitMs(10);
  }

  return 0;
}
