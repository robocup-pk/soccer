#include <iostream>

#include "GameObject.h"
#include "Texture.h"
#include "Collision.h"

vis::GameObject::GameObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_,
                            Eigen::Vector3d velocity_, Eigen::Vector3d acceleration_,
                            float mass_kg_, Texture2D sprite_, Eigen::Vector3d color_)
    : name(name_), sprite(sprite_), mass_kg(mass_kg_) {
  position = ConvertEigenVecToGlm(position_);
  velocity = ConvertEigenVecToGlm(velocity_);
  acceleration = ConvertEigenVecToGlm(acceleration_);
  size = ConvertEigenVecToGlm(size_);
  color = ConvertEigenVecToGlm(color_);
  radius = std::min(size.x / 2, size.y / 2);
}

void vis::GameObject::Draw(SpriteRenderer& renderer) {
  renderer.DrawSprite(sprite, glm::vec2(position.x, position.y), size, 0, color);
}

void vis::GameObject::Move(float dt) {
  // TODO: think of a better way
  float friction = 0.1f;
  velocity.x -= velocity.x * friction * dt;
  velocity.y -= velocity.y * friction * dt;

  position += velocity * dt;
}

glm::vec2 vis::GameObject::GetCenterPosition() {
  return glm::vec2(position.x + (size.x / 2), position.y + (size.y / 2));
}

glm::vec2 vis::ConvertEigenVecToGlm(const Eigen::Vector2d& vec2d) {
  return glm::vec2(vec2d[0], vec2d[1]);
}

glm::vec3 vis::ConvertEigenVecToGlm(const Eigen::Vector3d& vec3d) {
  return glm::vec3(vec3d[0], vec3d[1], vec3d[2]);
}