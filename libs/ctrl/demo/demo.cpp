#include <iostream>
#include <vector>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
// #include "RRTX.h"
// #include "Kick.h"
using namespace std;
int main(int argc, char* argv[]) {
  std::cout << "[Demo] Running RobotManager demo" << std::endl;

  // Initialize objects
  vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);
  // Initialize RobotManager
  rob::RobotManager robot_manager;
  // Only use RobotManager and its trajectory system
  // Set initial robot pose
  Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);  // Robot starts at origin facing up
  robot_manager.InitializePose(robot_start_pose);
  vector<Eigen::Vector3d> waypoints;
  // Choose a test case based on command line argument
  int test_case = 1;
  if (argc > 1) {
    test_case = std::atoi(argv[1]);
  }

  switch (test_case) {
    case 1: {
      // Test 1: Simple forward movement with reasonable spacing
      std::cout << "Test 1: Simple forward movement" << std::endl;
      waypoints.push_back(Eigen::Vector3d(0, 0, 0));
      waypoints.push_back(Eigen::Vector3d(-1, 0, 0));
      waypoints.push_back(Eigen::Vector3d(-1, 1, 0));
      waypoints.push_back(Eigen::Vector3d(0, 1, 0));
      waypoints.push_back(Eigen::Vector3d(0, 0, 0));
      break;
    }
    case 2: {
      // Test 2: Circular path (example: 8 points around a circle)
      std::cout << "Test 2: Circular path" << std::endl;
      double r = 1.0;
      for (int i = 0; i < 8; ++i) {
        double theta = i * M_PI / 4;
        waypoints.push_back(Eigen::Vector3d(r * cos(theta), r * sin(theta), 0));
      }
      waypoints.push_back(waypoints[0]);  // Close the loop
      break;
    }
    default: {
      std::cout << "Test 0: Simple diagonal" << std::endl;
      waypoints.push_back(Eigen::Vector3d(0, 0, 0));
      waypoints.push_back(Eigen::Vector3d(1, 1, 0));
      break;
    }
  }

  // Set the path for the robot using the new trajectory system
  // Wait for robot to leave CALIBRATING
  while (robot_manager.GetRobotState() == "CALIBRATING") {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  robot_manager.SetPath(waypoints);

  // Visualization loop (simple simulation loop)
  while (true) {
    // Run simulation step
    if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
      std::cout << "[Demo] Simulation finished" << std::endl;
      break;
    }

    // Process input and update robot state
    vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);

    // Update soccer objects with current robot pose
    soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
  }

  return 0;
}