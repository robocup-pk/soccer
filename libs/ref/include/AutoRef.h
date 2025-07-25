#ifndef AUTOREF_H
#define AUTOREF_H

#include <vector>

#include "SoccerObject.h"

namespace ref {
void CheckCollisions(std::vector<state::SoccerObject>& soccer_objects);
void CheckForGoals(std::vector<state::SoccerObject>& soccer_objects);
bool CheckCollisionWithWall(state::SoccerObject& obj);
bool IsOutsidePlayingField(state::SoccerObject& obj);
}  // namespace ref

#endif  // AUTOREF_H