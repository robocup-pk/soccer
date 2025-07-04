#ifndef COLLISION_H
#define COLLISION_H

#include <map>

#include "SoccerObject.h"

namespace kin {

void CheckAndResolveCollisions(std::vector<state::SoccerObject>& soccer_objects);
bool CheckCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2);
void ResolveCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2);

bool IsInsideBoundary(const state::SoccerObject& obj);
void ClampInsideBoundary(state::SoccerObject& obj);
void ResolveCollisionWithWall(std::vector<state::SoccerObject>& soccer_objects);

// Ball-front detection function
bool IsBallInFrontOfRobot(state::SoccerObject& robot, state::SoccerObject& ball);

// Ball Sticking to flat surface
void HandleBallSticking(state::SoccerObject& robot, state::SoccerObject& ball);

// Update Attached Ball Position Continously
void UpdateAttachedBallPosition(state::SoccerObject& robot, state::SoccerObject& ball);

// Ball Detachment with positioning
void DetachBall(state::SoccerObject& ball, float detach_velocity = 2.0f);

}  // namespace kin

#endif  // COLLISION_H