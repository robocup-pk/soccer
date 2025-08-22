#ifndef AUTOREF_H
#define AUTOREF_H

#include <vector>

#include "SoccerObject.h"
#include "Game.h"
#include "GLSimulation.h"

namespace ref {

class AutoRef {
 public:
  int AttackerDoubleTouchedBall(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  bool ViolatedKickOffSetUp(std::vector<state::SoccerObject>& soccer_objects, int team_id,
                            Game& g);
  // helper for ViolatedKickOffSetUp
  bool RobotInCenterCircle(Eigen::Vector3d v);
  int CheckGoal(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  bool BallLeftFieldTouchLine(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  bool IsOutsidePlayingFieldTouchLines(state::SoccerObject& obj);
  std::vector<bool> BallLeftFieldGoalLines(std::vector<state::SoccerObject>& soccer_objects,
                                           Game& g);
  void DefenderInDefenseArea(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  bool BotDribbledBallTooFar(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  void AttackerDoubleTouchedBallInOpponentDefenseArea(
      std::vector<state::SoccerObject>& soccer_objects, Game& g);
  void BotKickedBallTooFast(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  void BotCrashUnique(std::vector<state::SoccerObject>& soccer_objects, Game& g);

  void BotTooFastInStop(std::vector<state::SoccerObject>& soccer_objects, Game& g);
  int team_one_foul_counter = 0;
  int team_two_foul_counter = 0;
};

/* ok right now there are a lot of issues with freekick
like the team that kicks the ball out gets it and selected player logic
and the ball being at the right position */

};  // namespace ref

#endif  // AUTOREF_H