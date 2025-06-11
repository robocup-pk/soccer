#include <gtest/gtest.h>

#include "GLCallback.h"
#include "GLWindow.h"

TEST(GLWindowTest, CanCreateWindow) {
  vis::GLWindow window;
  bool success = window.Init(800, 600, "Test Window");
  EXPECT_TRUE(success);
}

TEST(GLWindowTest, WindowClosesOnEscape) {
  vis::GLWindow window;
  window.Init(800, 600, "Test Window");

  EXPECT_FALSE(glfwWindowShouldClose(window.GetRawGLFW()));

  // Simulate escape key
  vis::GLCallback::KeyCallback(window.GetRawGLFW(), GLFW_KEY_ESCAPE, 0, GLFW_PRESS, 0);

  EXPECT_TRUE(glfwWindowShouldClose(window.GetRawGLFW()));
}