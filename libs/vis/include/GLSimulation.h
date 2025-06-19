#ifndef GL_WINDOW_H
#define GL_WINDOW_H

// std
#include <map>

// extern libs
#include <glad/glad.h>  // ORDER MATTERS (bw glad and glfw)
#include <GLFW/glfw3.h>

// self
#include "GameObject.h"
#include "GLConfig.h"

namespace vis {

class GLSimulation {
 public:
  GLSimulation(std::vector<state::SoccerObject>& soccer_objects,
               int width_px = cfg::Coordinates::window_width_px,
               int height_px = cfg::Coordinates::window_height_px,
               const char* window_title = GLConfig::window_title);
  void RegisterCallbacks();
  void InitGameObjects(std::vector<state::SoccerObject>& soccer_objects);
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(std::vector<state::SoccerObject>& soccer_objects, float dt);
  void Render(float dt);
  bool Update();
  void ProcessInput(float dt);
  void UpdateGameObject(const state::SoccerObject& soccer_object);

  // Outside Access
  std::map<std::string, GameObject>& GetGameObjects();

  ~GLSimulation();

 private:
  GLFWwindow* window;
  std::map<std::string, GameObject> game_objects;
  SpriteRenderer renderer;
};

}  // namespace vis

#endif  // GL_WINDOW_H