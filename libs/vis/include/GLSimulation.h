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
#include "SoccerField.h"
#include "RobotManager.h"

namespace vis {

void ProcessInput(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects);
void ProcessInput(GLFWwindow* gl_window, std::vector<rob::RobotManager>& robot_managers);

class GLSimulation {
 public:
  GLSimulation();
  void RegisterCallbacks();
  void InitGameObjects(std::vector<state::SoccerObject>& soccer_objects);
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(std::vector<state::SoccerObject>& soccer_objects, float dt);
  void Render(float dt);
  bool Update();
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