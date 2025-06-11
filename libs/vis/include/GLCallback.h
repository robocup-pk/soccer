#ifndef GL_CALLBACK_H
#define GL_CALLBACK_H

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

namespace vis {
class GLCallback {
 public:
  static float x_offset;
  static float y_offset;

  // Callbacks (static because 'this' ptr is not needed)
  static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mode);
  static void FrameBufferCallback(GLFWwindow* window, int width_px, int height_px);
  // TODO: add a callback for joystick later
};
}  // namespace vis

#endif  // GL_CALLBACK_H