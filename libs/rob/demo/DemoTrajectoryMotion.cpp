// cpp std libs
#include <iostream>
#include <chrono>
#include <limits>

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
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  // ROBOT
  std::cout << std::fixed << std::setprecision(2);
  rob::RobotManager robot_manager;

  std::cout << "First path\n\n";
  std::vector<Eigen::Vector3d> path;
  path.push_back(Eigen::Vector3d(0, 0, 0));
  path.push_back(Eigen::Vector3d(-1, 0, 1));
  path.push_back(Eigen::Vector3d(0, 0, 0));
  robot_manager.SetPath(path, util::GetCurrentTime());

  std::cout << "Second path\n\n";
  path.clear();
  path.push_back(Eigen::Vector3d(-0.5, 0, 0));
  path.push_back(Eigen::Vector3d(-1.5, 0, 0));
  robot_manager.SetPath(path, util::GetCurrentTime() + 6);

  std::cout << "Third path\n\n";
  path.clear();
  path.push_back(Eigen::Vector3d(-1.5, 0, 0));
  path.push_back(Eigen::Vector3d(-0.5, 0, 0));
  robot_manager.SetPath(path, util::GetCurrentTime() + 10);

  // std::cout << "Fourth path\n\n";
  // path.clear();
  // path.push_back(Eigen::Vector3d(-0.2, 0, 0));
  // path.push_back(Eigen::Vector3d(-1.2, 0, 0));
  // path.push_back(Eigen::Vector3d(-0.2, 0, 0));
  // robot_manager.SetPath(path, util::GetCurrentTime() + 12);

  double replan_time = -std::numeric_limits<double>::max();

  while (1) {
    double t = util::GetCurrentTime();
    soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();

    // if (replan_time + 9 < t) {
    //   replan_time = t;
    //   std::cout << "[DemoTrajectoryMotion::Main] Replan at " << replan_time << std::endl;
    //   robot_manager.SetPath(path);  //, t_start_s);
    //   // replan_time = 10000000;
    // }

    // Simulation
    if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
    util::WaitMs(10);
  }

  return 0;
}
