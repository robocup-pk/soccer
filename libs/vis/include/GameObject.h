#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <string>

#include "SpriteRenderer.h"
#include "SoccerObject.h"

namespace vis {

glm::vec2 ConvertEigenVecToGlm(const Eigen::Vector2d& vec2d);
glm::vec3 ConvertEigenVecToGlm(const Eigen::Vector3d& vec3d);

class GameObject : public state::SoccerObject {
 public:
  glm::vec3 position, velocity, acceleration;
  glm::vec2 size;
  glm::vec3 color;
  Texture2D sprite;
  std::string name;
  float mass_kg;
  float radius;

  GameObject() = default;

  GameObject(std::string name, Eigen::Vector3d position,
             Eigen::Vector2d size = Eigen::Vector2d::Zero(),
             Eigen::Vector3d velocity = Eigen::Vector3d::Zero(),
             Eigen::Vector3d acceleration = Eigen::Vector3d::Zero(), float mass_kg = 1,
             Texture2D sprite = Texture2D(false),
             Eigen::Vector3d color = Eigen::Vector3d(1.0f, 1.0f, 1.0f));

  GameObject(std::string name, glm::vec2 pos, glm::vec2 size,
             glm::vec2 velocity = glm::vec2(0.0f, 0.0f),
             glm::vec2 acceleration = glm::vec2(0.0f, 0.0f), float mass_kg = 1,
             Texture2D sprite = Texture2D(false), glm::vec3 color = glm::vec3(1.0f));
  void Draw(SpriteRenderer &renderer);
  void Move(float dt);
  glm::vec2 GetCenterPosition();
};
}  // namespace vis

#endif  // GAME_OBJECT_H