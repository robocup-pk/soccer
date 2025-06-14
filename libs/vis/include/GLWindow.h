#ifndef GL_WINDOW_H
#define GL_WINDOW_H

// std
#include <vector>

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

// self
#include "GameObject.h"
#include "GLConfig.h"

namespace vis {

class GLWindow {
 public:
  GLWindow(int width_px = GLConfig::window_width_px, int height_px = GLConfig::window_height_px,
           const char* window_title = GLConfig::window_title);
  void RegisterCallbacks();
  void InitGameObjects();
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(float dt);
  void Render(float dt);
  bool Update();

  ~GLWindow();

 private:
  GLFWwindow* window;
  std::vector<GameObject> game_objects;
  SpriteRenderer renderer;
};

}  // namespace vis

#endif  // GL_WINDOW_H