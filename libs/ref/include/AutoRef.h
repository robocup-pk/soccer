#ifndef AUTOREF_H
#define AUTOREF_H

#include <vector>

#include "SoccerObject.h"

namespace ref {
void CheckCollisions(std::vector<state::SoccerObject>& soccer_objects);
void CheckForGoals(std::vector<state::SoccerObject>& soccer_objects);
bool CheckCollisionWithWall(state::SoccerObject& obj);
bool IsOutsidePlayingField(state::SoccerObject& obj);

class AutoRef {
 public:
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

  AutoRef();
  bool AttackerDoubleTouchedBall(GameState state, bool released_ball,
                                 double disp_ball_since_kickoff, state::SoccerObject& player);

  GameState state;
  int team_one_score;
  int team_two_score;
};

};  // namespace ref

#endif  // AUTOREF_H