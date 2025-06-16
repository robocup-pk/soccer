#ifndef COLLISION_H
#define COLLISION_H

#include <map>

#include "GameObject.h"

namespace vis {

void CheckAndResolveCollisions(std::map<std::string, GameObject>& game_objects);
bool CheckCircularCollision(GameObject& obj1, GameObject& obj2);
void ResolveCircularCollision(GameObject& obj1, GameObject& obj2);

bool IsInsideBoundary(const GameObject& obj);
void ClampInsideBoundary(GameObject& obj);
void ResolveCollisionWithWall(std::map<std::string, GameObject>& game_objects);

}  // namespace vis

#endif  // COLLISION_H