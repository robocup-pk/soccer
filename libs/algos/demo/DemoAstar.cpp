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
#include "AStar.h"
#include "AlgoConstants.h"

// Random number generator for obstacle movement
namespace {
std::mt19937 obstacle_rng(std::random_device{}());
std::uniform_real_distribution<double> speed_dist(-1, 1);  // Random velocity
std::uniform_real_distribution<double> change_vel_dist(0.0,
                                                       1.0);  // Probability to change velocity
}  // namespace

void PrintPath(const state::Path& path) {
  if (path.empty()) {
    std::cout << "No path available" << std::endl;
    return;
  }

  std::cout << "Path (" << path.size() << " waypoints): ";
  for (size_t i = 0; i < path.size(); ++i) {
    std::cout << "(" << std::fixed << std::setprecision(2) << path[i].x << "," << path[i].y << ")";
    if (i < path.size() - 1) std::cout << " -> ";
  }
  std::cout << std::endl;
}

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
  soccer_objects[soccer_objects.size() - 1].position =
      Eigen::Vector3d(initial_goal.x, initial_goal.y, 0.0);  // Ball

  // Create A* planner
  algos::AStar astar_planner(0.05f);  // 5cm grid resolution

  // Tracking variables
  Eigen::Vector3d last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
  Eigen::Vector3d last_robot_pos = soccer_objects[0].position;

  auto last_time = std::chrono::high_resolution_clock::now();
  auto last_obstacle_update = std::chrono::high_resolution_clock::now();

  // Path planning frequency control
  int plan_counter = 0;
  const int PLAN_INTERVAL = 5;  // Plan every 5 frames

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

    // Extract obstacles (all objects except the robot and ball)
    std::vector<state::SoccerObject> obstacles;
    for (size_t i = 0; i < soccer_objects.size(); ++i) {
      if (soccer_objects[i].name == "robot0" || soccer_objects[i].name == "ball") continue;
      obstacles.push_back(soccer_objects[i]);
    }

    // Check if we need to replan (ball moved or it's time to plan)
    double ball_movement =
        (soccer_objects[soccer_objects.size() - 1].position - last_ball_pos).norm();
    bool ball_moved = ball_movement > cfg::AStarConstants::movement_threshold;

    // Plan at regular intervals or if the ball moved significantly
    if (plan_counter >= PLAN_INTERVAL || ball_moved) {
      // Time the path planning
      auto plan_start = std::chrono::high_resolution_clock::now();

      // Find path using A*
      state::Path path = astar_planner.findPath(current_robot_pos, current_ball_pos, obstacles);

      auto plan_end = std::chrono::high_resolution_clock::now();
      auto plan_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(plan_end - plan_start);

      std::cout << "A* planning time: " << plan_duration.count() << "ms" << std::endl;

      // Update path visualization
      if (!path.empty()) {
        gl_simulation.SetVisualizationPath(path, glm::vec3(1.0f, 0.0f, 0.0f));
        PrintPath(path);
      } else {
        gl_simulation.ClearVisualizationPath();
        std::cout << "No path found!" << std::endl;
      }

      // Reset counter and update last positions
      plan_counter = 0;
      if (ball_moved) {
        last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
      }
    }

    plan_counter++;

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