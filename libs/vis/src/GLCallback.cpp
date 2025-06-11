#include "GLCallback.h"
#include "GLWindow.h"

// When window is resized (this function is called), we have to resize the viewport
void vis::GLCallback::FrameBufferCallback(GLFWwindow* window, int width_px, int height_px) {
  glViewport(0, 0, width_px, height_px);
}

// When a key is pressed, this function is called
void vis::GLCallback::KeyCallback(GLFWwindow* window, int key, int scancode, int action,
                                  int mode) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) glfwSetWindowShouldClose(window, GL_TRUE);
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