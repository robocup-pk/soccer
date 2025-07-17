// cpp std libs
#include <iostream>
#include <chrono>
#include <algorithm>
#include <vector>
#include <cmath>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Algos.h"
#include "Coordinates.h"
#include "Waypoint.h"
#include "RRT.h"
#include "RRTX.h"
#include "Astar.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  Eigen::Vector3d lastballPos = soccer_objects[0].GetPosition();  // Bot
  state::Path path;
  state::Path path1;
  static int path_index = 0;

  while (1) {
    // --- Time Calculation ---
    auto current_time = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    // --- Ball Movement with WASD ---
    GLFWwindow* window = gl_simulation.GetRawGLFW();
    Eigen::Vector3d& ball_velocity = soccer_objects[1].velocity;
    ball_velocity = Eigen::Vector3d::Zero();
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) ball_velocity[1] = 2.0;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) ball_velocity[1] = -2.0;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) ball_velocity[0] = -2.0;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) ball_velocity[0] = 2.0;

    // --- Bot Behavior ---
    Eigen::Vector3d& bot_pos3d = soccer_objects[0].position;
    state::Waypoint bot_pos(bot_pos3d[0], bot_pos3d[1], 0.0f);
    state::Waypoint goal_pos;

    bool bot_has_ball = (soccer_objects[1].is_attached == 1);
    goal_pos = bot_has_ball
      ? state::Waypoint(1.7, -0.09f, 0.0f)  // Go to goal if bot has ball
      : state::Waypoint(soccer_objects[1].position[0], soccer_objects[1].position[1], 0.0f); // Chase ball

    path = algos::FindSinglePath_RRTX(bot_pos, goal_pos);

    // --- Goal Tolerance Check ---
    Eigen::Vector2d bot_xy(bot_pos.x, bot_pos.y);
    Eigen::Vector2d goal_xy(goal_pos.x, goal_pos.y);
    double dist_to_goal = (bot_xy - goal_xy).norm();
    const double stop_threshold = 0.1;

    if (bot_has_ball && dist_to_goal < stop_threshold) {
      // Stop when ball reaches goal
      soccer_objects[0].velocity = Eigen::Vector3d::Zero();
    } else if (path.size() > 1) {
      state::Waypoint& target = path[1];
      Eigen::Vector2d direction(target.x - bot_pos.x, target.y - bot_pos.y);
      double norm = direction.norm();

      // --- Rotation Handling ---
      double target_angle = std::atan2(direction.y(), direction.x());
      double current_angle = bot_pos3d.z();
      double angle_diff = target_angle - current_angle;

      // Normalize angle to [-π, π]
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

      double angular_speed = 5.0;
      soccer_objects[0].velocity[2] = angle_diff * angular_speed;

      // --- Forward Movement (aligned to direction) ---
      double linear_speed = 1.0;
      if (norm > 1e-6) {
        soccer_objects[0].velocity[0] = (direction.x() / norm) * linear_speed;
        soccer_objects[0].velocity[1] = (direction.y() / norm) * linear_speed;
      } else {
        soccer_objects[0].velocity[0] = 0;
        soccer_objects[0].velocity[1] = 0;
      }
    } else {
      soccer_objects[0].velocity = Eigen::Vector3d::Zero();  // No path
    }

    // --- Sim Update ---
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
