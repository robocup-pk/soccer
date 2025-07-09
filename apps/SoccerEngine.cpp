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
#include "Utils.h"
#include "Coordinates.h"
#include "RobotManager.h"

void ProcessInput(GLFWwindow* gl_window, rob::RobotManager& robot_manager) {
  Eigen::Vector3d velocity_fBody(0, 0, 0);

  if (glfwGetKey(gl_window, GLFW_KEY_H) == GLFW_PRESS) {
    robot_manager.GoHome();
    return;
  }

  // STEERING
  bool key_pressed = false;
  if (glfwGetKey(gl_window, GLFW_KEY_W) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.y() += 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_X) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.y() -= 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_A) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.x() -= 1;
  }
  if (glfwGetKey(gl_window, GLFW_KEY_D) == GLFW_PRESS) {
    key_pressed = true;
    velocity_fBody.x() += 1;
  }
  if (key_pressed) {
    velocity_fBody.normalize();
    robot_manager.SetBodyVelocity(velocity_fBody);
  }
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    // From pixel-frame to world-frame
    // xpos -= field.util::MmToPixels(field.width_mm);
    // double ft_per_pixel = 1 / cfg::Coordinates::px_per_mm;
    // double x = xpos * ft_per_pixel;

    std::cout << "[MouseClick] Click at: (" << xpos << ", " << ypos << ")\n";
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
    ProcessInput(gl_window, robot_manager);
    kin::UpdateKinematics(soccer_objects, dt);
    kin::CheckAndResolveCollisions(soccer_objects);

    soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }

  return 0;
}
