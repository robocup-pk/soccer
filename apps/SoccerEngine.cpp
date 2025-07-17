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

void ProcessInput(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects,
                  rob::RobotManager& robot_manager) {
  Eigen::Vector3d velocity_fBody(0, 0, 0);

  // STEERING
  bool key_pressed = false;
  if (glfwGetKey(gl_window, GLFW_KEY_W) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.x() += 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_S) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.x() -= 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_A) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.y() += 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_D) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.y() -= 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_C) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.z() += 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_X) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.z() -= 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_K) == GLFW_PRESS) {
    if (soccer_objects[soccer_objects.size() - 1].is_attached) {
      kin::DetachBall(soccer_objects[soccer_objects.size() - 1], 0.5f);
      std::cout << "[ProcessInput] Ball detached from robot" << std::endl;
    }
  }
  if (glfwGetKey(gl_window, GLFW_KEY_J) == GLFW_PRESS) {
    if (soccer_objects[soccer_objects.size() - 1].is_attached) {
      kin::DetachBall(soccer_objects[soccer_objects.size() - 1], 0.1f);
      std::cout << "[ProcessInput] Ball detached from robot" << std::endl;
    }
  }

  if (key_pressed) {
    velocity_fBody.normalize();
  }
  // Update robot's velocity directly
  robot_manager.SetBodyVelocity(velocity_fBody);
}

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

  // Robot(s)
  rob::RobotManager robot_manager;

  while (1) {
    float dt = util::CalculateDt();
    ProcessInput(gl_window, soccer_objects, robot_manager);
    robot_manager.IntegratePhysics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);
    robot_manager.HandleCollisionFeedback(soccer_objects);
    robot_manager.UpdateKinematics(soccer_objects, dt);

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
