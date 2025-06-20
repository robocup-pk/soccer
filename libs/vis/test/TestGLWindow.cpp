#include <gtest/gtest.h>

#include "GLCallback.h"
#include "GLSimulation.h"

TEST(GLSimulationTest, WindowClosesOnEscape) {
  // TODO: do these tests in a better way. I think github has problems showing up the window
  // vis::GLSimulation window(800, 600, "Test Window");

  // EXPECT_FALSE(glfwWindowShouldClose(window.GetRawGLFW()));

  // // Simulate escape key
  // vis::GLCallback::KeyCallback(window.GetRawGLFW(), GLFW_KEY_ESCAPE, 0, GLFW_PRESS, 0);

  // EXPECT_TRUE(glfwWindowShouldClose(window.GetRawGLFW()));
}