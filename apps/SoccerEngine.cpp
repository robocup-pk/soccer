// cpp std libs
#include <iostream>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Collision.h"
#include "Coordinates.h"

void UpdateKinematics(std::vector<state::SoccerObject>& soccer_objects, float dt) {
  for (state::SoccerObject& soccer_object : soccer_objects) {
    soccer_object.Move(dt);
  }
}

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  std::vector<state::SoccerObject> soccer_objects;

  // Ball (Name, Pos, Size, Vel, Acc, Mass)
  soccer_objects.push_back(state::SoccerObject(
      "ball",
      Eigen::Vector3d(-cfg::SystemConfig::ball_radius_ft, cfg::SystemConfig::ball_radius_ft, 0),
      Eigen::Vector2d(cfg::SystemConfig::ball_radius_ft * 2,
                      cfg::SystemConfig::ball_radius_ft * 2),
      cfg::SystemConfig::init_ball_velocity_ftps, cfg::SystemConfig::init_ball_acceleration_ftpsps,
      1));

  // Robots
  for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_ft(i * 3, 0, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_ft, cfg::SystemConfig::robot_size_ft,
                            cfg::SystemConfig::init_robot_velocity_ftps,
                            cfg::SystemConfig::init_robot_acceleration_ftpsps, 5));
  }

  vis::GLSimulation gl_simulation(soccer_objects);

  // soccer_objects[0].position[0] = 0;
  // soccer_objects[0].position[1] = 0;
  // soccer_objects[0].velocity[0] = 0;
  // soccer_objects[0].velocity[1] = 0;
  // soccer_objects[0].acceleration[0] = 0;
  // soccer_objects[0].acceleration[1] = 0;

  // soccer_objects[1].velocity[1] = -100;
  // soccer_objects[1].position[0] = 0;
  // soccer_objects[1].position[1] = 0;

  while (1) {
    // Calculate delta time
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Other Steps
    // ...
    // ...

    // Perform physical motion for all the objects
    // UpdateKinematics(soccer_objects, dt);

    state::CheckAndResolveCollisions(soccer_objects);

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
