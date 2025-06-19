#include <iostream>

#include "GLCallback.h"
#include "GLSimulation.h"

float vis::GLCallback::x_offset_robot0 = 0;
float vis::GLCallback::y_offset_robot0 = 0;
float vis::GLCallback::x_offset_robot0_worldf = 0;
float vis::GLCallback::y_offset_robot0_worldf = 0;
float vis::GLCallback::x_offset_robot1 = 0;
float vis::GLCallback::y_offset_robot1 = 0;
float vis::GLCallback::x_offset_robot1_worldf = 0;
float vis::GLCallback::y_offset_robot1_worldf = 0;

bool vis::GLCallback::keys[1024] = {false};

// When window is resized (this function is called), we have to resize the viewport
void vis::GLCallback::FrameBufferCallback(GLFWwindow* window, int width_px, int height_px) {
  glViewport(0, 0, width_px, height_px);
}

// When a key is pressed, this function is called
void vis::GLCallback::KeyCallback(GLFWwindow* window, int key, int scancode, int action,
                                  int mode) {
  if (key >= 0 && key < 1024) {
    if (action == GLFW_PRESS) {
      keys[key] = true;
    } else if (action == GLFW_RELEASE) {
      keys[key] = false;
    }
  }

  // Handle immediate actions (like ESC)
  if (action == GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;
    }
  }
  // TODO: add support for other keyboard keys to make it interactive
}

void vis::GLSimulation::RegisterCallbacks() {
  // Any key pressed
  glfwSetKeyCallback(window, vis::GLCallback::KeyCallback);

  // Window resized
  glfwSetFramebufferSizeCallback(window, vis::GLCallback::FrameBufferCallback);

  // TODO: register callback for joystick inputs when available
}