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
void ProcessInputTwoTeams(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects);

/* Process Input Helpers*/
void FindAndUpdateSelectedPlayer(std::vector<state::SoccerObject>& soccer_objects);
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

extern int team_one_selected_player;
extern int team_two_selected_player;

class GLSimulation {
 public:
  GLSimulation();
  void RegisterCallbacks();
  void InitGameObjects(std::vector<state::SoccerObject>& soccer_objects);
  void InitGameObjectsTwoTeams(std::vector<state::SoccerObject>& soccer_objects);
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(std::vector<state::SoccerObject>& soccer_objects, float dt);

  void Render(float dt);
  bool Update();
  void UpdateGameObject(const state::SoccerObject& soccer_object);

  // Outside Access
  std::map<std::string, GameObject>& GetGameObjects();

  static bool RobotAreaPressed(double robot_center_x, double robot_center_y,
                               double mouse_click_left_pos_x, double mouse_click_left_pos_y);

  ~GLSimulation();

 private:
  GLFWwindow* window;
  std::map<std::string, GameObject> game_objects;
  SpriteRenderer renderer;
};

}  // namespace vis

#endif  // GL_WINDOW_H