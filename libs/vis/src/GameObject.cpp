#include <iostream>

#include "GameObject.h"
#include "Texture.h"

vis::GameObject::GameObject()
    : position(0.0f, 0.0f),
      size(1.0f, 1.0f),
      velocity(0.0f),
      color(1.0f),
      rotation(0.0f),
      sprite(),
      is_solid(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, glm::vec2 size, Texture2D sprite,
                            glm::vec2 velocity, glm::vec3 color)
    : name(name),
      position(pos),
      size(size),
      velocity(velocity),
      color(color),
      rotation(0.0f),
      sprite(sprite),
      is_solid(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, double radius, Texture2D sprite,
                            glm::vec2 velocity, glm::vec3 color)
    : GameObject(name, pos, glm::vec2(radius * 2, radius * 2), sprite, velocity, color) {}

void vis::GameObject::Draw(SpriteRenderer& renderer, glm::vec2 position_offset_worldf) {
  glm::vec2 moved_position = glm::vec2(this->position.x + position_offset_worldf.x,
                                       this->position.y + position_offset_worldf.y);
  renderer.DrawSprite(this->sprite, moved_position, this->size, this->rotation, this->color);
}

void vis::GameObject::Move(float dt) { position += velocity * dt; }

glm::vec2 vis::GameObject::GetCenterPosition() {
  std::cout << "size: " << size.x << std::endl;
  std::cout << "corner: " << position.x << std::endl;
  std::cout << "center: " << position.x + (size.x / 2) << std::endl;
  return glm::vec2(position.x + (size.x / 2), position.y + (size.y / 2));
}