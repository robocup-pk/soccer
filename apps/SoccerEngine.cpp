// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Collision.h"
#include "MotorModel.h"
#include "Coordinates.h"
#include "Estimator.h"

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

  est::Estimator estimator;
  estimator.initialized_pose = true;
  double t_sec = 10;
  Eigen::Vector3d velocity_fBody(0.5, 0.5, 0);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // START THE MOTORS
  std::vector<hw::MotorModel> motors = std::vector<hw::MotorModel>(4);
  Eigen::Vector4d wheel_speeds_rpm =
      estimator.robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  for (int i = 0; i < 4; ++i) motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
  Eigen::Vector4d ticks;

  while (1) {
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Other Steps
    // ...
    // ...

    // Position Estimation
    for (int t = 0; t < 4; ++t) ticks[t] = motors[t].GetTicks();
    estimator.NewEncoderData(ticks);

    // ESTIMATED
    soccer_objects[0].position = estimator.GetPose();

    // TRUE POSITION
    soccer_objects[1].position = 

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
