#include <algorithm>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

// self libs
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Kinematics.h"
#include "Utils.h"
#include "Coordinates.h"
#include "RobotManager.h"
#include "SoccerField.h"
#include "AutoRef.h"
#include "Game.h"
#include <cmath>

int main(int argc, char* argv[]) {
  // TODO: Game State
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  ref::Game::MoveToFormation(cfg::SystemConfig::team_one_start_formation,
                             cfg::SystemConfig::team_two_start_formation, soccer_objects);
  ref::Game game;

  // Simulation - OpenGL
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjectsTwoTeams(soccer_objects);
  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

  while (1) {
    float dt = util::CalculateDt();

    game.UpdateGameState(soccer_objects);

    // ref::ViolatedKickOffSetUp(soccer_objects, 1, game);
    // ref::ViolatedKickOffSetUp(soccer_objects, 2, game);
    ref::AttackerDoubleTouchedBall(soccer_objects, game);

    vis::ProcessInputTwoTeams(gl_window, soccer_objects);
    kin::UpdateKinematics(soccer_objects, dt);

    game.DoGoals(soccer_objects);

    game.CheckCollisions(soccer_objects);
    kin::CheckAndResolveCollisions(soccer_objects);

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }
  return 0;
}