#ifndef COLLISION_H
#define COLLISION_H

#include <map>

#include "GameObject.h"

namespace vis {

void CheckCollision(std::map<std::string, GameObject>& game_objects);
bool CheckCircularCollision( GameObject& obj1,  GameObject& obj2);
void ResolveCircularCollision(GameObject& obj1, GameObject& obj2, int mass1 = 1, int mass2 = 100);

}  // namespace vis

#endif  // COLLISION_H