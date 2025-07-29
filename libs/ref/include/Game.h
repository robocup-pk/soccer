#ifndef GAME_H
#define GAME_H

#include <vector>

#include "SoccerObject.h"

namespace ref {

class Game {
 public:
  static void MoveToFormation(std::vector<Eigen::Vector3d> team_one_formation,
                              std::vector<Eigen::Vector3d> team_two_formation,
                              std::vector<state::SoccerObject>& soccer_objects);
  void DoGoals(std::vector<state::SoccerObject>& soccer_objects);
  void UpdateGameState(std::vector<state::SoccerObject>& soccer_objects);

  enum GameState {
    Halt,
    Timeout,
    Stop,
    PrepareKickoff,
    BallPlacement,
    PreparePenalty,
    Kickoff,
    FreeKick,
    Penalty,
    Run
  };
  Game();

  GameState state;
  int team_with_ball = 1;
  int team_one_score = 0;
  int team_two_score = 0;
  Eigen::Vector3d state_start_ball_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
};

};  // namespace ref

#endif  // GAME_Ha