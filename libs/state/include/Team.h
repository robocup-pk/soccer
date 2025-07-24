#ifndef TEAM_H
#define TEAM_H

#include <string>
#include "RobotManager.h"
#include "SoccerObject.h"

namespace state {

class Team {
 public:
  // Constructors
  Team(int id);

  int GetScore();
  int GetFoulTally();
  void IncrementScore();
  void IncrementNumFouls();

 private:
  int score = 0;
  int foul_tally = 0;
  int team_id;
};

}  // namespace state

#endif  // TEAM_H