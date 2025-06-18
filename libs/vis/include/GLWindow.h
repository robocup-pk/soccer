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
#include "IntelligentMovement.h"
#include "IntelligentMovement2.h"
#include "MultiPlayerIntelligentMovement2.h"
#include "MultiPlayerRRT.h"
#include "TeamCompetition.h"

namespace vis {

class GLWindow {
 public:
  GLWindow(int width_px = cfg::Coordinates::window_width_px,
           int height_px = cfg::Coordinates::window_height_px,
           const char* window_title = GLConfig::window_title);
  void RegisterCallbacks();
  void InitGameObjects();
  void SetupTrajectoryMovement();
  GLFWwindow* GetRawGLFW() const;

  // Logic used in simulation
  bool RunSimulationStep(float dt);
  void Render(float dt);
  bool Update();
  void ProcessInput(float dt);

  ~GLWindow();

 private:
  GLFWwindow* window;
  std::map<std::string, GameObject> game_objects;
  SpriteRenderer renderer;
  std::unique_ptr<IntelligentMovement> intelligent_movement;
  std::unique_ptr<IntelligentMovement2> intelligent_movement2;
  std::unique_ptr<MultiPlayerIntelligentMovement2> multi_player_movement;
  std::unique_ptr<MultiPlayerRRT> multi_player_rrt;
  std::unique_ptr<TeamCompetition> team_competition;
  
  // Movement system selection
  enum MovementMode {
    RRT_MOVEMENT = 1,           // IntelligentMovement (RRT-based, single player)
    INTERCEPT_MOVEMENT = 2,     // IntelligentMovement2 (Ball intercept-based, single player)
    MULTIPLAYER_MOVEMENT = 3,   // MultiPlayerIntelligentMovement2 (Multiple players)
    MULTIPLAYER_RRT = 4,        // MultiPlayerRRT (Multiple players with RRT)
    TEAM_COMPETITION = 5        // TeamCompetition (Two teams with different strategies)
  };
  MovementMode current_movement_mode;
};

}  // namespace vis

#endif  // GL_WINDOW_H