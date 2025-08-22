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
#include "PathRenderer.h"

namespace vis {

void ProcessInput(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects);
void ProcessInput(GLFWwindow* gl_window, std::vector<rob::RobotManager>& robot_managers);
void ProcessInputTwoTeams(GLFWwindow* gl_window, std::vector<state::SoccerObject>& soccer_objects);
void ProcessInputMultipleObjects(GLFWwindow* gl_window,
                                 std::vector<state::SoccerObject>& soccer_objects);

/* Process Input Helpers*/
void FindAndUpdateSelectedPlayer(std::vector<state::SoccerObject>& soccer_objects);
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

extern int team_one_selected_player;
extern int team_two_selected_player;
extern int button_pressed;
extern Eigen::Vector3d last_ball_kick_pos;
extern bool last_kick_valid;  // to determine whether last_ball_kick_pos is null or not
inline bool k_key_prev_was_pressed = false;
inline bool l_key_prev_was_pressed = false;
inline bool j_key_prev_was_pressed = false;
inline bool i_key_prev_was_pressed = false;

inline Eigen::Vector3d button_one_pos_m = Eigen::Vector3d(-1.5, -1.3, 0);
inline Eigen::Vector3d button_two_pos_m = Eigen::Vector3d(-1, -1.3, 0);
inline Eigen::Vector3d button_three_pos_m = Eigen::Vector3d(-0.5, -1.3, 0);
inline Eigen::Vector3d button_four_pos_m = Eigen::Vector3d(0, -1.3, 0);

inline Eigen::Vector3d button_size_m = Eigen::Vector3d(0.540 * 0.8, 0.200 * 0.8, 0.0);

class GLSimulation {
 public:
  GLSimulation();
  void RegisterCallbacks();
  void InitGameObjects(std::vector<state::SoccerObject>& soccer_objects);
  void InitGameObjectsTwoTeams(std::vector<state::SoccerObject>& soccer_objects);
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(std::vector<state::SoccerObject>& soccer_objects, float dt, bool referee_tags = false);

  void Render(float dt);
  void RenderRefereeTags();
  bool Update();
  void UpdateGameObject(const state::SoccerObject& soccer_object);
  void SetVisualizationPath(state::Path& path, glm::vec3 color);
  void ClearVisualizationPath();

  // Outside Access
  std::map<std::string, GameObject>& GetGameObjects();

  static bool RobotAreaPressed(double robot_center_x, double robot_center_y,
                               double mouse_click_left_pos_x, double mouse_click_left_pos_y);

  static int ButtonAreaPressed(double mouse_click_left_pos_x, double mouse_click_left_pos_y);

  static bool inRectCenter(double x, double y, double cx, double cy, double w, double h);

  ~GLSimulation();

 private:
  GLFWwindow* window;
  std::map<std::string, GameObject> game_objects;
  std::map<std::string, GameObject> buttons;
  SpriteRenderer renderer;

  glm::vec2 size = glm::vec2(0.54f, 0.2f);
  glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);
  PathRenderer path_renderer;
};

}  // namespace vis

#endif  // GL_WINDOW_H