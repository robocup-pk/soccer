#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Waypoint.h"
#include "RRTX.h"

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

int main() {
  // Initialize simulation
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

  // Initial positions
  state::Waypoint start(-1.5, 0.0, 0.0);
  state::Waypoint initial_goal(1.0, 0.0, 0.0);

  soccer_objects[0].position = Eigen::Vector3d(start.x, start.y, 0.0);  // Robot
  soccer_objects[1].position = Eigen::Vector3d(0.0, 0.0, 0.0);          // Robot 1
  soccer_objects[2].position = Eigen::Vector3d(0.0, 1.0, 0.0);          // Robot 2
  soccer_objects[3].position = Eigen::Vector3d(0.0, -1.0, 0.0);         // Robot 3
  soccer_objects[soccer_objects.size() - 1].position =
      Eigen::Vector3d(initial_goal.x, initial_goal.y, 0.0);  // Ball

  // Create RRT-X planner
  algos::RRTX rrtx_planner(start, initial_goal, 0.01);

  // Tracking variables
  Eigen::Vector3d last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
  const double BALL_MOVEMENT_THRESHOLD = 0.05;  // 5cm threshold

  auto last_time = std::chrono::high_resolution_clock::now();

  // Main simulation loop
  while (true) {
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    // Handle Input
    vis::ProcessInputMultipleObjects(gl_window, soccer_objects);

    // Get current positions
    state::Waypoint current_robot_pos(soccer_objects[0].position[0], soccer_objects[0].position[1],
                                      0.0f);
    state::Waypoint current_ball_pos(soccer_objects[soccer_objects.size() - 1].position[0],
                                     soccer_objects[soccer_objects.size() - 1].position[1], 0.0f);

    // Update robot position in planner
    rrtx_planner.UpdateRobotPosition(current_robot_pos);

    // Check if ball moved significantly
    double ball_movement =
        (soccer_objects[soccer_objects.size() - 1].position - last_ball_pos).norm();
    if (ball_movement > BALL_MOVEMENT_THRESHOLD) {
      std::cout << "\n--- Ball moved to (" << current_ball_pos.x << ", " << current_ball_pos.y
                << ") ---" << std::endl;

      // Update goal in planner
      rrtx_planner.UpdateGoal(current_ball_pos);
      last_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
    }

    // Run planning steps
    std::vector<state::SoccerObject> obstacles;
    std::copy_if(
        soccer_objects.begin(), soccer_objects.end(), std::back_inserter(obstacles),
        [](const state::SoccerObject& obj) { return obj.name != "robot0" && obj.name != "ball"; });

    while (!rrtx_planner.SolutionExists()) {
      rrtx_planner.PlanStep(obstacles);
    }

    // Print path when solution exists
    if (rrtx_planner.SolutionExists()) {
      state::Path path = rrtx_planner.ReconstructPath();
      if (!path.empty()) {
        PrintPath(path);
      }
    }

    // Update physics
    kin::UpdateKinematics(soccer_objects, dt);

    // Run simulation step
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      break;
    }
  }

  return 0;
}