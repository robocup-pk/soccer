// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "HardwareManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Collision.h"
#include "Coordinates.h"
#include "Estimator.h"
#include "Utils.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  kin::RobotDescription robot_desc;

  // Square configuration with wheels at corners
  robot_desc.wheel_positions_m = {
      {0.15, 0.15},    // wheel 1: front-left
      {-0.15, 0.15},   // wheel 2: rear-left
      {-0.15, -0.15},  // wheel 3: rear-right
      {0.15, -0.15}    // wheel 4: front-right
  };

  // Wheel angles (perpendicular to radial direction for typical omniwheel setup)
  robot_desc.wheel_angles_rad = {
      -M_PI / 4,     // -45° (wheel 1)
      M_PI / 4,      // 45° (wheel 2)
      3 * M_PI / 4,  // 135° (wheel 3)
      -3 * M_PI / 4  // -135° (wheel 4)
  };

  std::shared_ptr<kin::RobotModel> robot_model = std::make_shared<kin::RobotModel>(robot_desc);

  // Start Estimator
  est::Estimator estimator(robot_model);
  estimator.initialized_pose = true;
  double t_sec = 10;
  Eigen::Vector3d velocity_fBody(-1, 1, 1);
  Eigen::Vector3d pose_true = velocity_fBody * t_sec;

  // Start Hardware (Motors, Gyro [later])
  hw::HardwareManager hardware_manager(robot_model);
  hardware_manager.SetBodyVelocity(velocity_fBody);
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
    ticks = hardware_manager.GetEncoderTicks();
    estimator.NewEncoderData(ticks);
    soccer_objects[0].position = estimator.GetPose();

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
