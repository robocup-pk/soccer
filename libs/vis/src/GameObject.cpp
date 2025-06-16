#include <iostream>

#include "GameObject.h"
#include "Texture.h"
#include "Collision.h"

vis::GameObject::GameObject()
    : position(0.0f, 0.0f),
      size(1.0f, 1.0f),
      velocity(0.0f),
      acceleration(0.0, 0.0),
      color(1.0f),
      rotation(0.0f),
      sprite(),
      is_solid(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, glm::vec2 size, glm::vec2 velocity,
                            glm::vec2 acceleration, Texture2D sprite, glm::vec3 color)
    : name(name),
      position(pos),
      size(size),
      velocity(velocity),
      acceleration(acceleration),
      color(color),
      rotation(0.0f),
      sprite(sprite),
      is_solid(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, double radius, glm::vec2 velocity,
                            glm::vec2 acceleration, Texture2D sprite, glm::vec3 color)
    : GameObject(name, pos, glm::vec2(radius * 2, radius * 2), velocity, acceleration, sprite,
                 color) {}

void vis::GameObject::Draw(SpriteRenderer& renderer, glm::vec2 position_offset_worldf) {
  position.x += position_offset_worldf.x;
  position.y += position_offset_worldf.y;
  if (!IsInsideBoundary(*this)) {
    ClampInsideBoundary(*this);
  }
  renderer.DrawSprite(this->sprite, position, this->size, this->rotation, this->color);
}

void vis::GameObject::Move(float dt) {
  // TODO: think of a better way
  // if (acceleration.x * velocity.x > 0) {
  //   acceleration.x *= -1;
  // }
  // if (acceleration.y * velocity.y > 0) {
  //   acceleration.y *= -1;
  // }

  // glm::vec2 new_velocity = velocity + acceleration * dt;
  // if (new_velocity.x * velocity.x < 0) {
  //   new_velocity.x = 0;
  // }
  // if (new_velocity.y * velocity.y < 0) {
  //   new_velocity.y = 0;
  // }
  // velocity = new_velocity;
  float friction = 1.5f;
  velocity.x -= velocity.x * friction * dt;
  velocity.y -= velocity.y * friction * dt;

  position += velocity * dt;

  if (name == "ball") {
    std::cout << name << " Accelert: " << acceleration.x << " " << acceleration.y << std::endl;
    std::cout << name << " Velocity: " << velocity.x << " " << velocity.y << std::endl;
    std::cout << name << " Position: " << position.x << " " << position.y << std::endl;
    std::cout << "dt: " << dt << std::endl << std::endl;
  }
}

glm::vec2 vis::GameObject::GetCenterPosition() {
  return glm::vec2(position.x + (size.x / 2), position.y + (size.y / 2));
}