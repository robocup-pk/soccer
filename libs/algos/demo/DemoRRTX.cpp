#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <random>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Waypoint.h"
#include "RRTX.h"
#include "AlgoConstants.h"

// Random number generator for obstacle movement
namespace {
std::mt19937 obstacle_rng(std::random_device{}());
std::uniform_real_distribution<double> speed_dist(-1, 1);  // Random velocity
std::uniform_real_distribution<double> change_vel_dist(0.0,
                                                       1.0);  // Probability to change velocity
}  // namespace

// Update obstacle velocities with random movement
void UpdateObstacleVelocities(std::vector<state::SoccerObject>& soccer_objects, float dt) {
  double field_width = vis::SoccerField::GetInstance().playing_area_width_mm / 1000.0;
  double field_height = vis::SoccerField::GetInstance().playing_area_height_mm / 1000.0;
  const double field_margin = 0.3;

  for (size_t i = 1; i < soccer_objects.size() - 1; ++i) {  // Skip robot0 and ball
    auto& obstacle = soccer_objects[i];

    // Occasionally change velocity
    if (change_vel_dist(obstacle_rng) < 0.8f * dt) {  // 80% chance per second
      obstacle.velocity = Eigen::Vector3d(speed_dist(obstacle_rng), speed_dist(obstacle_rng), 0.0);
    }

    // Simple boundary checking - reverse velocity if near edge
    if (obstacle.position.x() < -field_width / 2 + field_margin && obstacle.velocity.x() < 0) {
      obstacle.velocity.x() = -obstacle.velocity.x();
    }
    if (obstacle.position.x() > field_width / 2 - field_margin && obstacle.velocity.x() > 0) {
      obstacle.velocity.x() = -obstacle.velocity.x();
    }
    if (obstacle.position.y() < -field_height / 2 + field_margin && obstacle.velocity.y() < 0) {
      obstacle.velocity.y() = -obstacle.velocity.y();
    }
    if (obstacle.position.y() > field_height / 2 - field_margin && obstacle.velocity.y() > 0) {
      obstacle.velocity.y() = -obstacle.velocity.y();
    }
  }
}

int main() {
  // Initialize simulation
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

  // Initial positions - spread obstacles around
  state::Waypoint start(-1.5, 0.0, 0.0);
  state::Waypoint initial_goal(1.0, 0.0, 0.0);

  soccer_objects[0].position = Eigen::Vector3d(start.x, start.y, 0.0);  // Robot
  soccer_objects[1].position = Eigen::Vector3d(0.5, 0.5, 0.0);          // Robot 1
  soccer_objects[2].position = Eigen::Vector3d(-0.5, 0.5, 0.0);         // Robot 2
  soccer_objects[3].position = Eigen::Vector3d(0.0, -0.5, 0.0);         // Robot 3
  soccer_objects[soccer_objects.size() - 1].position =
      Eigen::Vector3d(initial_goal.x, initial_goal.y, 0.0);  // Ball

  // Create RRT-X planner
  algos::RRTX rrtx_planner(start, initial_goal);

  // Tracking variables
  Eigen::Vector3d last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
  Eigen::Vector3d last_robot_pos = soccer_objects[0].position;

  auto last_time = std::chrono::high_resolution_clock::now();
  auto last_obstacle_update = std::chrono::high_resolution_clock::now();
  int plan_iterations = 5;
  // Main simulation loop
  while (true) {
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Handle Input
    vis::ProcessInputMultipleObjects(gl_window, soccer_objects);

    // Update obstacle velocities (let kinematics handle movement)
    UpdateObstacleVelocities(soccer_objects, dt);

    // Get current positions
    state::Waypoint current_robot_pos(soccer_objects[0].position[0], soccer_objects[0].position[1],
                                      0.0f);
    state::Waypoint current_ball_pos(soccer_objects[soccer_objects.size() - 1].position[0],
                                     soccer_objects[soccer_objects.size() - 1].position[1], 0.0f);

    // Check if ball moved significantly
    double ball_movement =
        (soccer_objects[soccer_objects.size() - 1].position - last_ball_pos).norm();
    if (ball_movement > cfg::RRTXConstants::movement_threshold) {
      rrtx_planner = algos::RRTX(current_robot_pos, current_ball_pos);
      while (!rrtx_planner.SolutionExists()) {
        rrtx_planner.PlanStep();
      }
      last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
    }

    // Check if robot moved significantly
    if ((last_robot_pos - soccer_objects[0].position).norm() >
        cfg::RRTXConstants::movement_threshold) {
      rrtx_planner.UpdateRobotPosition(current_robot_pos);
      last_robot_pos = soccer_objects[0].position;
    }

    // Check if obstacles moved significantly (throttled)
    auto obstacle_check_time = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration<float>(obstacle_check_time - last_obstacle_update).count() > 0.0f) {
      std::vector<state::SoccerObject> obstacles;
      std::copy_if(soccer_objects.begin(), soccer_objects.end(), std::back_inserter(obstacles),
                   [](const state::SoccerObject& obj) {
                     return obj.name != "robot0" && obj.name != "ball";
                   });

      if (rrtx_planner.HasObstaclesChanged(obstacles)) {
        rrtx_planner.UpdateObstacles(obstacles);
      }
      last_obstacle_update = obstacle_check_time;
      plan_iterations++;
    }

    // Run planning steps
    for (int i = 0; i < plan_iterations; i++) {
      rrtx_planner.PlanStep();
    }

    // Update path visualization
    if (rrtx_planner.SolutionExists()) {
      state::Path path = rrtx_planner.ReconstructPath();
      if (!path.empty()) {
        gl_simulation.SetVisualizationPath(path, glm::vec3(1.0f, 0.0f, 0.0f));

        std::cout << path << std::endl;

      } else {
        gl_simulation.ClearVisualizationPath();
      }
      plan_iterations--;
      plan_iterations = std::max(plan_iterations, 1);
    } else {
      gl_simulation.ClearVisualizationPath();
      plan_iterations++;
    }

    // Update physics (this will move obstacles based on their velocities)
    kin::CheckAndResolveCollisions(soccer_objects);
    kin::UpdateKinematics(soccer_objects, dt);

    // Run simulation step
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      break;
    }
  }

  return 0;
}