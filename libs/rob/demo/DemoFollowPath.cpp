// cpp std libs
#include <iostream>
#include <chrono>
#include <cmath>
// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"

int main(int argc, char* argv[]) {
  // Need two robots for this demo to run
  if (cfg::SystemConfig::num_robots != 2) {
    std::cout << "[DemoMotionController::Main] Set num_robots to 2. Exiting!" << std::endl;
    return 0;
  }
  // std::cout << "Random Test 1 " << std::endl;
  //  START SIMULATION
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  std::vector<Eigen::Vector3d> path1, path2;
  path1.push_back(Eigen::Vector3d(0, 0, 0));
  path1.push_back(Eigen::Vector3d(0, 1, 0));
  path1.push_back(Eigen::Vector3d(0, 0, 0));
  path2.push_back(Eigen::Vector3d(0, 0, 0));
  path2.push_back(Eigen::Vector3d(1, 0, 0));
  path2.push_back(Eigen::Vector3d(1, 1, 0));

  // path.push_back(EigGetPoseInWorldFrameen::Vector3d(0, 0, 90 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(0, 1, 90 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(0, 1, 0 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(1, 1, 0 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(1, 1, -90 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(1, 0, -90 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(1, 0, -180 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(0, 0, -180 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(0, 0, 270 * M_PI / 180));
  // path.push_back(Eigen::Vector3d(0, 0, 360 * M_PI / 180));

  rob::RobotManager robot_manager1;
  robot_manager1.SetPath(path1);

  rob::RobotManager robot_manager2;
  robot_manager2.SetPath(path2);
  // ball
  state::SoccerObject& ball = soccer_objects[2];  // assuming index 2 is the ball

  ball.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);  // Move along +x at 1 m/s
  ball.acceleration = Eigen::Vector3d::Zero();     // No acceleration
  ball.is_attached = true;
  ball.attached_to = &soccer_objects[0];
  while (1) {
    // std::cout << "Object 2 name: " << soccer_objects[2].name << std::endl;
    soccer_objects[0].position = robot_manager1.GetPoseInWorldFrame();
    soccer_objects[1].position = robot_manager2.GetPoseInWorldFrame();

    if (ball.is_attached && ball.attached_to) {
      double angle = ball.attached_to->position[2];
      double offset = 0.1;  // distance in front of robot face
      ball.position[0] = ball.attached_to->position[0] + offset * cos(angle);
      ball.position[1] = ball.attached_to->position[1] + offset * sin(angle);

    } else {
      ball.Move(util::CalculateDt());
    }

    // simulation
    if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
