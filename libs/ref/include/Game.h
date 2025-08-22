#ifndef GAME_H
#define GAME_H

#include <vector>

#include "SoccerObject.h"

namespace ref {

class Game {
 public:
  void SetUpKickOff(std::vector<state::SoccerObject>& soccer_objects);
  void SetUpFreeKick(std::vector<state::SoccerObject>& soccer_objects,
                     Eigen::Vector3d ball_fall_off_pos);
  void DoGoals(std::vector<state::SoccerObject>& soccer_objects, int scoring_team);
  void UpdateGameState(std::vector<state::SoccerObject>& soccer_objects);
  void CheckCollisions(std::vector<state::SoccerObject>& soccer_objects);
  bool CheckCollisionWithWall(state::SoccerObject& obj);

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

  Game(std::vector<state::SoccerObject>& soccer_objects);

  GameState state = Kickoff;
  int team_with_ball = 1;
  int team_one_score = 0;
  int team_two_score = 0;

  // variable to determine who has ball possesion when it goes out
  state::SoccerObject last_bot_touched_ball;

  // variable for determining when to transition from kickoff/freekick to run
  Eigen::Vector3d state_start_ball_pos;

  bool was_ball_prev_attached = true;
  Eigen::Vector3d last_attached_pos;

  // variable for attacker double touched ball
  bool kicker_released = false;
  bool nonkicking_robot_intercepted = false;
  GameState prev_state = Kickoff;

  // variable for bot dribbled too much
  bool bot_got_foul_for_overdribbling = false;
  Eigen::Vector3d last_valid_pos;

  float displacement = 0;
};

};  // namespace ref

#endif  // GAME_H