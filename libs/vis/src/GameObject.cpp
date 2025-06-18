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
      is_solid(false),
      has_ball_attached(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, glm::vec2 size, glm::vec2 velocity,
                            glm::vec2 acceleration, float mass_kg, Texture2D sprite,
                            glm::vec3 color)
    : name(name),
      position(pos),
      size(size),
      velocity(velocity),
      acceleration(acceleration),
      mass_kg(mass_kg),
      color(color),
      rotation(0.0f),
      sprite(sprite),
      is_solid(false),
      has_ball_attached(false) {}

vis::GameObject::GameObject(std::string name, glm::vec2 pos, double radius, glm::vec2 velocity,
                            glm::vec2 acceleration, float mass_kg, Texture2D sprite,
                            glm::vec3 color)
    : GameObject(name, pos, glm::vec2(radius * 2, radius * 2), velocity, acceleration, mass_kg,
                 sprite, color) {}

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

bool vis::GameObject::IsPointInFrontSector(glm::vec2 point) {
  glm::vec2 robot_center = GetCenterPosition();

  // Adjust rotation: subtract 90° to make 0° point up instead of right
  float rotation_rad = glm::radians(rotation - 90.0f);
  glm::vec2 front_dir(cos(rotation_rad), sin(rotation_rad));

  glm::vec2 to_point = point - robot_center;
  float dot_product = glm::dot(glm::normalize(to_point), front_dir);
  float angle_threshold = cos(glm::radians(30.0f));

  return dot_product > angle_threshold;
}

glm::vec2 vis::GameObject::GetFrontAttachmentPoint() {
  glm::vec2 robot_center = GetCenterPosition();

  float rotation_rad = glm::radians(rotation - 90.0f);

  float attach_distance = size.x * 0.6f;
  glm::vec2 front_dir(cos(rotation_rad), sin(rotation_rad));

  return robot_center + front_dir * attach_distance;
}

void vis::GameObject::KickBall(GameObject& ball) {
  if (!has_ball_attached) {
    return;  // No ball to kick
  }

  // Calculate kick direction based on robot's rotation
  float rotation_rad = glm::radians(rotation - 90.0f);
  glm::vec2 kick_direction(cos(rotation_rad), sin(rotation_rad));

  // Set ball velocity
  ball.velocity = kick_direction * kick_speed;

  // Immediately move ball away from robot to prevent re-attachment
  float separation_distance = (size.x + ball.size.x) * 0.7f;
  ball.position = GetCenterPosition() + kick_direction * separation_distance - ball.size * 0.5f;

  has_ball_attached = false;
}
