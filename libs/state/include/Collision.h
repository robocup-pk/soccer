#ifndef COLLISION_H
#define COLLISION_H

#include <map>

#include "SoccerObject.h"

namespace state {

void CheckAndResolveCollisions(std::vector<SoccerObject>& soccer_objects);
bool CheckCircularCollision(SoccerObject& obj1, SoccerObject& obj2);
void ResolveCircularCollision(SoccerObject& obj1, SoccerObject& obj2);

bool IsInsideBoundary(const SoccerObject& obj);
void ClampInsideBoundary(SoccerObject& obj);
void ResolveCollisionWithWall(std::vector<SoccerObject>& soccer_objects);

// Ball-front detection function
bool IsBallInFrontOfRobot(SoccerObject& robot,SoccerObject& ball);

// Ball Sticking to flat surface
void HandleBallSticking(SoccerObject &robot, SoccerObject &ball);

// Update Attached Ball Position Continously
void UpdateAttachedBallPosition(SoccerObject& robot, SoccerObject& ball);

}  // namespace state

#endif  // COLLISION_H