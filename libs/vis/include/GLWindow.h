#ifndef GL_WINDOW_H
#define GL_WINDOW_H

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

namespace vis {

class GLWindow {
 public:
  bool Init(int width_px, int height_px, const char* window_title);
  void RegisterCallbacks();
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  void SetScreenColor();
  bool Update();

  ~GLWindow();

 private:
  GLFWwindow* window;
  int width_px, height_px;
};

}  // namespace vis

#endif  // GL_WINDOW_H