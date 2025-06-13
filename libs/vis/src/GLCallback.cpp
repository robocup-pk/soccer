#include <iostream>

#include "GLCallback.h"
#include "GLWindow.h"

float vis::GLCallback::x_offset = 0;
float vis::GLCallback::y_offset = 0;

// When window is resized (this function is called), we have to resize the viewport
void vis::GLCallback::FrameBufferCallback(GLFWwindow* window, int width_px, int height_px) {
  // Create square viewport
  int square_size = std::min(width_px, height_px);
  int x_offset = (width_px - square_size) / 2;
  int y_offset = (height_px - square_size) / 2;

  glViewport(x_offset, y_offset, square_size, square_size);
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
        if (std::abs(vis::GLCallback::y_offset + move_speed) < 1.0f) {
          vis::GLCallback::y_offset += move_speed;
        }
        break;
      case GLFW_KEY_X:
        std::cout << "[vis::GLCallback] X" << std::endl;
        if (std::abs(vis::GLCallback::y_offset - move_speed) < 1.0f) {
          vis::GLCallback::y_offset -= move_speed;
        }
        break;
      case GLFW_KEY_A:
        std::cout << "[vis::GLCallback] A" << std::endl;
        if (std::abs(vis::GLCallback::x_offset - move_speed) < 1.0f) {
          vis::GLCallback::x_offset -= move_speed;
        }
        break;
      case GLFW_KEY_D:
        std::cout << "[vis::GLCallback] D" << std::endl;
        if (std::abs(vis::GLCallback::x_offset + move_speed) < 1.0f) {
          vis::GLCallback::x_offset += move_speed;
        }
        break;
      default:
        break;
    }
  }
  // TODO: add support for other keyboard keys to make it interactive
}

void vis::GLWindow::RegisterCallbacks() {
  // Any key pressed
  glfwSetKeyCallback(window, vis::GLCallback::KeyCallback);

  // Window resized
  glfwSetFramebufferSizeCallback(window, vis::GLCallback::FrameBufferCallback);

  // TODO: register callback for joystick inputs when available
}