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
#include "Astar.h"

int main(int argc, char* argv[]) {
  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);

  bool replan_required = true;
  Eigen::Vector3d lastballPos = soccer_objects[0].GetPosition();
  state::Path path;
  state::Waypoint location;
  state::Path path1;
  static int path_index = 0;
  while (1) {
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = current_time - last_time;
    float dt = std::chrono::duration<float>(duration).count();
    last_time = current_time;

    if (replan_required) {
      replan_required = false;
      state::Waypoint start =
          state::Waypoint(soccer_objects[1].position[0], soccer_objects[1].position[1], 0.0f);
      state::Waypoint goal =
          state::Waypoint(soccer_objects[0].position[0], soccer_objects[0].position[1], 0.0f);
      path = algos::Astar(start, goal);
      // std::cout << "path: " << path << ' ' << path.size() << std::endl;
      // path1 = algos::Astar(start, goal);
      // std::cout << "path1: " << path1 << std::endl;
    }
    if (abs(lastballPos[0] - soccer_objects[0].GetPosition()[0]) > 0.025 ||
        abs(lastballPos[1] - soccer_objects[0].GetPosition()[1]) > 0.025) {
      replan_required = true;
      lastballPos = soccer_objects[0].GetPosition();
      path_index = 0;
    }

    state::Waypoint robPos =
        state::Waypoint(soccer_objects[1].position[0], soccer_objects[1].position[1], 0.0f);
    // std::cout << soccer_objects[1].velocity[0] << ' ' << soccer_objects[1].velocity[1] << ' '
    //           << soccer_objects[1].velocity[2] << std::endl;
    location = path[path_index];
    state::Waypoint direction_vector = (location - robPos);
    double norm = std::sqrt(direction_vector.x * direction_vector.x +
                            direction_vector.y * direction_vector.y);
    double speed = 1.0;  // Set your desired constant speed here
    if (norm > 1e-6) {
      Eigen::Vector3d target_vel(direction_vector.x / norm * speed,
                                 direction_vector.y / norm * speed, 0.0f);
      double alpha = 0.05;
      soccer_objects[1].velocity = (1 - alpha) * soccer_objects[1].velocity + alpha * target_vel;
    } else {
      soccer_objects[1].velocity = Eigen::Vector3d(0, 0, 0);
    }
    if (sqrt((location.x - robPos.x) * (location.x - robPos.x) +
             (location.y - robPos.y) * (location.y - robPos.y)) < 0.2) {
      ++path_index;
    }

    // std::cout << soccer_objects[1].velocity[0] << ' ' << soccer_objects[1].velocity[1] << ' '
    //           << soccer_objects[1].velocity[2] << std::endl;

    // Ball control with input (WASD) using GLFW directly
    GLFWwindow* window = gl_simulation.GetRawGLFW();
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
      soccer_objects[0].velocity[1] = 2.0;  // Move up
    } else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
      soccer_objects[0].velocity[1] = -2.0;  // Move down
    } else {
      soccer_objects[0].velocity[1] = 0.0;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
      soccer_objects[0].velocity[0] = -2.0;  // Move left
    } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
      soccer_objects[0].velocity[0] = 2.0;  // Move right
    } else {
      soccer_objects[0].velocity[0] = 0.0;
    }

    kin::UpdateKinematics(soccer_objects, dt);

    kin::CheckAndResolveCollisions(soccer_objects);

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }

    // Error Checking
    // ResolveErrors();
  }

  return 0;
}
