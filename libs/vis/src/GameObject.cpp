#include <iostream>

#include "GameObject.h"
#include "Texture.h"
#include "Kinematics.h"
#include "Coordinates.h"

vis::GameObject::GameObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_,
                            Eigen::Vector3d velocity_, Eigen::Vector3d acceleration_,
                            float mass_kg_, Texture2D sprite_, Eigen::Vector3d color_)
    : name(name_), sprite(sprite_), mass_kg(mass_kg_) {
  // GameObject works on millimeters (later converted to pixels for display)
  // SoccerObject works on meters
  position = glm::vec3(1000) * ConvertEigenVecToGlm(position_);
  position[2] = position_[2]; // Angle should stay as it is.
  velocity = glm::vec3(1000) * ConvertEigenVecToGlm(velocity_);
  acceleration = glm::vec3(1000) * ConvertEigenVecToGlm(acceleration_);
  size = glm::vec2(1000) * ConvertEigenVecToGlm(size_);
  color = ConvertEigenVecToGlm(color_);
  radius = std::min(size.x / 2, size.y / 2);
}

/*
 It converts all the soccer_object stuff from ft to pixel
 We then draw these game objects
*/
vis::GameObject& vis::GameObject::operator=(const state::SoccerObject& soccer_object) {
  position = ConvertEigenVecToGlm(static_cast<Eigen::Vector3d>(
                 soccer_object.position.cwiseProduct(cfg::Coordinates::m_px_coords))) *
             cfg::Coordinates::px_per_m;

  // TODO: The angle is in radians, we do not need to convert it to pixels. Need to add proper for
  // it later, and also for velocity and acceleration
  position.z = soccer_object.position[2];  // angle in radians

  velocity = ConvertEigenVecToGlm(static_cast<Eigen::Vector3d>(
                 soccer_object.velocity.cwiseProduct(cfg::Coordinates::m_px_coords))) *
             cfg::Coordinates::px_per_m;

  acceleration = ConvertEigenVecToGlm(static_cast<Eigen::Vector3d>(
                     soccer_object.acceleration.cwiseProduct(cfg::Coordinates::m_px_coords))) *
                 cfg::Coordinates::px_per_m;

  size = ConvertEigenVecToGlm(static_cast<Eigen::Vector2d>(soccer_object.size)) *
         cfg::Coordinates::px_per_m;

  mass_kg = soccer_object.mass_kg;

  radius = soccer_object.radius_m * cfg::Coordinates::px_per_m;

  return *this;
}

void vis::GameObject::Draw(SpriteRenderer& renderer) {
  renderer.DrawSprite(sprite, glm::vec2(position.x, position.y), size, glm::degrees(position.z),
                      color);
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