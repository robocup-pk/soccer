#include <iostream>

#include "GameObject.h"
#include "Texture.h"

vis::GameObject::GameObject()
    : Position(0.0f, 0.0f),
      Size(1.0f, 1.0f),
      Velocity(0.0f),
      Color(1.0f),
      Rotation(0.0f),
      Sprite(),
      IsSolid(false),
      Destroyed(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, glm::vec2 size, Texture2D sprite,
                            glm::vec2 velocity, glm::vec3 color)
    : Name(name),
      Position(pos),
      Size(size),
      Velocity(velocity),
      Color(color),
      Rotation(0.0f),
      Sprite(sprite),
      IsSolid(false),
      Destroyed(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, double radius, Texture2D sprite,
                            glm::vec2 velocity, glm::vec3 color)
    : GameObject(name, pos, glm::vec2(radius * 2, radius * 2), sprite, velocity, color) {}

void vis::GameObject::Draw(SpriteRenderer& renderer, glm::vec2 position_offset_worldf) {
  glm::vec2 moved_position = glm::vec2(this->Position.x + position_offset_worldf.x,
                                       this->Position.y + position_offset_worldf.y);
  renderer.DrawSprite(this->Sprite, moved_position, this->Size, this->Rotation, this->Color);
}

void vis::GameObject::Move(float dt) {
  Position = Velocity * dt;
  // std::cout << "New Position: " << Position.x << " " << Position.y << std::endl;
}