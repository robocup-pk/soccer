#include <gtest/gtest.h>

#include "GLCallback.h"
#include "GLWindow.h"

TEST(GLWindowTest, CanCreateWindow) {
  vis::GLWindow window(800, 600, "Test Window");
  EXPECT_TRUE(1);
}

TEST(GLWindowTest, WindowClosesOnEscape) {
  vis::GLWindow window(800, 600, "Test Window");

  EXPECT_FALSE(glfwWindowShouldClose(window.GetRawGLFW()));

  // Simulate escape key
  vis::GLCallback::KeyCallback(window.GetRawGLFW(), GLFW_KEY_ESCAPE, 0, GLFW_PRESS, 0);

  EXPECT_TRUE(glfwWindowShouldClose(window.GetRawGLFW()));
}