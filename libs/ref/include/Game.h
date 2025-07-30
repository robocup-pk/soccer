#ifndef GAME_H
#define GAME_H

#include <vector>

#include "SoccerObject.h"

namespace ref {

class Game {
 public:
  void MoveToKickOffFormation(std::vector<state::SoccerObject>& soccer_objects);
  void DoGoals(std::vector<state::SoccerObject>& soccer_objects);
  void UpdateGameState(std::vector<state::SoccerObject>& soccer_objects);
  void CheckCollisions(std::vector<state::SoccerObject>& soccer_objects);
  bool CheckCollisionWithWall(state::SoccerObject& obj);
  bool IsOutsidePlayingField(state::SoccerObject& obj);

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
  bool kicker_released = false;
};

};  // namespace ref

#endif  // GAME_H