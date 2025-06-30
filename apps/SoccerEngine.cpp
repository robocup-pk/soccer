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
#include "Utils.h"

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
  Eigen::Vector3d velocity_fBody(0.1, 0.1, 0); //0.2);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // START THE MOTORS
  std::vector<hw::MotorModel> motors = std::vector<hw::MotorModel>(4);
  Eigen::Vector4d wheel_speeds_rpm =
      estimator.robot_model->RobotVelocityToWheelSpeedsRpm(velocity_fBody);
  for (int i = 0; i < 4; ++i) motors[i].SetWheelSpeedRpm(wheel_speeds_rpm[i]);
  Eigen::Vector4d ticks;

  double est_start_time = util::GetCurrentTime();
  double elapsed_time_s = 0;

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
    std::cout << "Ticks: " << ticks[0] << " " << ticks[1] << " " << ticks[2] << " " << ticks[3]
              << std::endl;
    estimator.NewEncoderData(ticks);

    // ESTIMATED
    soccer_objects[0].position = estimator.GetPose();
    std::cout << "Set wheel speeds (rpm): " << wheel_speeds_rpm.transpose()  << std::endl;
    // std::cout << "Pos: " << soccer_objects[0].position.transpose() << std::endl;

    // TRUE POSITION
    elapsed_time_s = util::GetCurrentTime() - est_start_time;
    soccer_objects[1].position = velocity_fBody * elapsed_time_s;

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
