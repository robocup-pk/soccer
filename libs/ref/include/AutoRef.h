#ifndef AUTOREF_H
#define AUTOREF_H

#include <vector>

#include "SoccerObject.h"
#include "Game.h"

namespace ref {
void CheckCollisions(std::vector<state::SoccerObject>& soccer_objects);
bool CheckCollisionWithWall(state::SoccerObject& obj);
bool IsOutsidePlayingField(state::SoccerObject& obj);

// bool AttackerDoubleTouchedBall(std::vector<state::SoccerObject>& soccer_objects, Game g);
bool ViolatedKickOffSetUp(std::vector<state::SoccerObject>& soccer_objects, int team_id, Game& g);
// helper for ViolatedKickOffSetUp
bool RobotInCenterCircle(Eigen::Vector3d v);
};  // namespace ref

#endif  // AUTOREF_H