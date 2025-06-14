#include <iostream>

#include "GLCallback.h"
#include "GLWindow.h"

float vis::GLCallback::x_offset_robot0 = 0;
float vis::GLCallback::y_offset_robot0 = 0;
float vis::GLCallback::x_offset_robot0_worldf = 0;
float vis::GLCallback::y_offset_robot0_worldf = 0;
float vis::GLCallback::x_offset_robot1 = 0;
float vis::GLCallback::y_offset_robot1 = 0;
float vis::GLCallback::x_offset_robot1_worldf = 0;
float vis::GLCallback::y_offset_robot1_worldf = 0;

// When window is resized (this function is called), we have to resize the viewport
void vis::GLCallback::FrameBufferCallback(GLFWwindow* window, int width_px, int height_px) {
  glViewport(0, 0, width_px, height_px);
}

// When a key is pressed, this function is called
void vis::GLCallback::KeyCallback(GLFWwindow* window, int key, int scancode, int action,
                                  int mode) {
  static float move_speed = 0.005f;
  static float max_possible_speed = 0.03f;

  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    // Simulate Acceleration
    if (action != GLFW_REPEAT) {
      move_speed = 0.005f;
    } else {
      move_speed = std::min(max_possible_speed, move_speed + 0.002f);
    }

    switch (key) {
      case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;
      case GLFW_KEY_W:
        std::cout << "[vis::GLCallback] W" << std::endl;
        if (std::abs(vis::GLCallback::y_offset_robot0 - move_speed) < 1.0f) {
          vis::GLCallback::y_offset_robot0 -= move_speed;
        }
        break;
      case GLFW_KEY_X:
        std::cout << "[vis::GLCallback] X" << std::endl;
        if (std::abs(vis::GLCallback::y_offset_robot0 + move_speed) < 1.0f) {
          vis::GLCallback::y_offset_robot0 += move_speed;
        }
        break;
      case GLFW_KEY_A:
        std::cout << "[vis::GLCallback] A" << std::endl;
        if (std::abs(vis::GLCallback::x_offset_robot0 - move_speed) < 1.0f) {
          vis::GLCallback::x_offset_robot0 -= move_speed;
        }
        break;
      case GLFW_KEY_D:
        std::cout << "[vis::GLCallback] D" << std::endl;
        if (std::abs(vis::GLCallback::x_offset_robot0 + move_speed) < 1.0f) {
          vis::GLCallback::x_offset_robot0 += move_speed;
        }
        break;
      case GLFW_KEY_UP:
        std::cout << "[vis::GLCallback] Up" << std::endl;
        if (std::abs(vis::GLCallback::y_offset_robot1 - move_speed) < 1.0f) {
          vis::GLCallback::y_offset_robot1 -= move_speed;
        }
        break;
      case GLFW_KEY_DOWN:
        std::cout << "[vis::GLCallback] Down" << std::endl;
        if (std::abs(vis::GLCallback::y_offset_robot1 + move_speed) < 1.0f) {
          vis::GLCallback::y_offset_robot1 += move_speed;
        }
        break;
      case GLFW_KEY_LEFT:
        std::cout << "[vis::GLCallback] Left" << std::endl;
        if (std::abs(vis::GLCallback::x_offset_robot1 - move_speed) < 1.0f) {
          vis::GLCallback::x_offset_robot1 -= move_speed;
        }
        break;
      case GLFW_KEY_RIGHT:
        std::cout << "[vis::GLCallback] Right" << std::endl;
        if (std::abs(vis::GLCallback::x_offset_robot1 + move_speed) < 1.0f) {
          vis::GLCallback::x_offset_robot1 += move_speed;
        }
        break;
      default:
        break;
    }
  }
  vis::GLCallback::x_offset_robot0_worldf =
      vis::GLCallback::x_offset_robot0 * vis::GLConfig::window_width_px;
  vis::GLCallback::y_offset_robot0_worldf =
      vis::GLCallback::y_offset_robot0 * vis::GLConfig::window_height_px;
  vis::GLCallback::x_offset_robot1_worldf =
      vis::GLCallback::x_offset_robot1 * vis::GLConfig::window_width_px;
  vis::GLCallback::y_offset_robot1_worldf =
      vis::GLCallback::y_offset_robot1 * vis::GLConfig::window_height_px;

  // TODO: add support for other keyboard keys to make it interactive
}

void vis::GLWindow::RegisterCallbacks() {
  // Any key pressed
  glfwSetKeyCallback(window, vis::GLCallback::KeyCallback);

  // Window resized
  glfwSetFramebufferSizeCallback(window, vis::GLCallback::FrameBufferCallback);

  // TODO: register callback for joystick inputs when available
}