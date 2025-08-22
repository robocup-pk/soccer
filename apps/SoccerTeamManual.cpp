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
#include "Controller.h"

int main(int argc, char* argv[]) {
  // TODO: Game State
  std::vector<state::SoccerObject> soccer_objects;
  state::InitSoccerObjects(soccer_objects);
  ref::Game game(soccer_objects);
  ref::AutoRef referee;
  // make this a separate function clean up in main put into one
  soccer_objects[cfg::SystemConfig::team_one_kicker].SetRobotRole(state::SoccerObject::Kicker);
  soccer_objects[cfg::SystemConfig::team_two_kicker].SetRobotRole(state::SoccerObject::Kicker);

  game.SetUpKickOff(soccer_objects);
  // Simulation - OpenGL
  vis::GLSimulation gl_simulation;
  gl_simulation.InitGameObjectsTwoTeams(soccer_objects);
  GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
  glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);
  ref::Controller controller;

  int i = 0;
  while (1) {
    float dt = util::CalculateDt();

    game.UpdateGameState(soccer_objects);

    game.CheckCollisions(soccer_objects);
    kin::CheckAndResolveCollisions(soccer_objects);
    controller.DoFouls(soccer_objects, game, referee);

    referee.DefenderInDefenseArea(soccer_objects, game);
    referee.AttackerDoubleTouchedBallInOpponentDefenseArea(soccer_objects, game);
    referee.BotCrashUnique(soccer_objects, game);

    vis::ProcessInputTwoTeams(gl_window, soccer_objects);
    kin::UpdateKinematics(soccer_objects, dt);

    if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
      std::cout << "[main] Simulation finished" << std::endl;
      break;
    }
  }
  return 0;
}