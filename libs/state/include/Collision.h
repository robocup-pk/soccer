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

}  // namespace state

#endif  // COLLISION_H