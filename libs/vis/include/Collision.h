#ifndef COLLISION_H
#define COLLISION_H

#include <map>

#include "GameObject.h"

namespace vis {

void CheckCollision(std::map<std::string, GameObject>& game_objects);
bool CheckCircularCollision(const GameObject& obj1, const GameObject& obj2);
void ResolveCircularCollision(GameObject& obj1, GameObject& obj2);

}  // namespace vis

#endif  // COLLISION_H