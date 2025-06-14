#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <string>

#include "SpriteRenderer.h"

namespace vis {
class GameObject {
 public:
  glm::vec2 Position, Size, Velocity;
  glm::vec3 Color;
  float Rotation;
  bool IsSolid;
  bool Destroyed;
  Texture2D Sprite;
  std::string Name;

  GameObject();
  GameObject(std::string name, glm::vec2 pos, glm::vec2 size, Texture2D sprite,
             glm::vec2 velocity = glm::vec2(0.0f, 0.0f), glm::vec3 color = glm::vec3(1.0f));
  GameObject(std::string name, glm::vec2 pos, double radius, Texture2D sprite,
             glm::vec2 velocity = glm::vec2(0.0f, 0.0f), glm::vec3 color = glm::vec3(1.0f));
  void Draw(SpriteRenderer &renderer, glm::vec2 position_offset = glm::vec2(0, 0));
  void Move(float dt);
};
}  // namespace vis

#endif  // GAME_OBJECT_H