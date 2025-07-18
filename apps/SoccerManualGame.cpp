// cpp std libs
#include <algorithm>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Utils.h"
#include "Coordinates.h"
#include "RobotManager.h"
#include "SoccerField.h"

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // Get window dimensions
    int window_width, window_height;
    glfwGetWindowSize(window, &window_width, &window_height);

    // Convert from pixel coordinates to world coordinates (meters)
    // 1. Convert pixels to millimeters
    float pixels_per_mm = util::PixelsPerMm();
    double x_mm = xpos / pixels_per_mm;
    double y_mm = ypos / pixels_per_mm;

    // 2. Convert from millimeters to meters
    double x_m = x_mm / 1000.0;
    double y_m = y_mm / 1000.0;

    // 3. Adjust coordinate system: center at field center, flip Y axis
    double field_width_m = vis::SoccerField::GetInstance().width_mm / 1000.0;
    double field_height_m = vis::SoccerField::GetInstance().height_mm / 1000.0;

    double world_x = x_m - (field_width_m / 2.0);   // Center X coordinate
    double world_y = (field_height_m / 2.0) - y_m;  // Center Y coordinate and flip Y axis

    std::cout << "[MouseClick] Pixel: (" << xpos << ", " << ypos << ")" << std::endl;
    std::cout << "[MouseClick] World: (" << world_x << ", " << world_y << ") meters" << std::endl;
  }
}

int main(int argc, char* argv[]) {
  // TODO: Game State
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);

  // Simulation - OpenGL
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjects(soccer_objects);
  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, MouseButtonCallback);

  while (1) {
    float dt = util::CalculateDt();
    vis::ProcessInput(gl_window, soccer_objects);

    // Step 1: Use SoccerObjects to Compute Plan
    // std::vector<std::vector<Eigen::Vector3d>> plan = algos::ComputePlan(soccer_objects);

    // Step 2: Send plan to robot managers
    // for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    //   robot_managers[i].SetPath(plan[i]);
    // }

    // Referee
    // Step: Rule checking on current soccer objects
    // 1. Kinematics (Collisions, Max speeds)
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    // 2.
    // 3.

    // for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    //   soccer_objects[i] = robot_managers[i];
    // }

    // Step: Simulation
    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
