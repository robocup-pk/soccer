#include <iostream>

#include "GLCallback.h"
#include "GLWindow.h"

float vis::GLCallback::x_offset = 0;
float vis::GLCallback::y_offset = 0;

// When window is resized (this function is called), we have to resize the viewport
void vis::GLCallback::FrameBufferCallback(GLFWwindow* window, int width_px, int height_px) {
  glViewport(0, 0, width_px, height_px);
}

// When a key is pressed, this function is called
void vis::GLCallback::KeyCallback(GLFWwindow* window, int key, int scancode, int action,
                                  int mode) {
  float move_speed = 0.05f;
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  } else if (key == GLFW_KEY_W) {  // && action == GLFW_PRESS) {
    std::cout << "[vis::GLCallback] W" << std::endl;
    vis::GLCallback::y_offset += move_speed;
  } else if (key == GLFW_KEY_A) {  // && action == GLFW_PRESS) {
    std::cout << "[vis::GLCallback] A" << std::endl;
    vis::GLCallback::x_offset -= move_speed;
  } else if (key == GLFW_KEY_D) {  // && action == GLFW_PRESS) {
    std::cout << "[vis::GLCallback] D" << std::endl;
    vis::GLCallback::x_offset += move_speed;
  } else if (key == GLFW_KEY_X) {  // && action == GLFW_PRESS) {
    std::cout << "[vis::GLCallback] X" << std::endl;
    vis::GLCallback::y_offset -= move_speed;
  }
  // TODO: add support for other keyboard keys to make it interactive
}

void vis::GLWindow::RegisterCallbacks() {
  // Any key pressed
  glfwSetKeyCallback(window, vis::GLCallback::KeyCallback);

  // Window resized
  glfwSetFramebufferSizeCallback(window, vis::GLCallback::FrameBufferCallback);

  // TODO: register callback for joystick inputs when available

  glfwMakeContextCurrent(window);  // TODO: what does it do?
}