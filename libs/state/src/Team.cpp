#include <iostream>
#include "Team.h"
#include "SoccerObject.h"
#include "SystemConfig.h"

state::Team::Team(int id) {
  team_id = id;

  // initalize soccer_objects
}

int state::Team::GetScore() { return score; }

int state::Team::GetFoulTally() { return foul_tally; }

void state::Team::IncrementScore() { score++; }

void state::Team::IncrementNumFouls() { foul_tally++; }