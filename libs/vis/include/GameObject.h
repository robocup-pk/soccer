#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <string>

#include "SpriteRenderer.h"

namespace vis {
class GameObject {
 public:
  glm::vec2 position, size, velocity, acceleration;
  glm::vec3 color;
  float rotation;
  bool is_solid;
  Texture2D sprite;
  std::string name;
  float mass_kg;
  bool has_ball_attached;
  float kick_speed;

  GameObject();
  GameObject(std::string name, glm::vec2 pos, glm::vec2 size,
             glm::vec2 velocity = glm::vec2(0.0f, 0.0f),
             glm::vec2 acceleration = glm::vec2(0.0f, 0.0f), float mass_kg = 1,
             Texture2D sprite = Texture2D(false), glm::vec3 color = glm::vec3(1.0f));
  GameObject(std::string name, glm::vec2 pos, double radius,
             glm::vec2 velocity = glm::vec2(0.0f, 0.0f),
             glm::vec2 acceleration = glm::vec2(0.0f, 0.0f), float mass_kg = 10,
             Texture2D sprite = Texture2D(false), glm::vec3 color = glm::vec3(1.0f));
  void Draw(SpriteRenderer& renderer, glm::vec2 position_offset = glm::vec2(0, 0));
  void Move(float dt);
  glm::vec2 GetCenterPosition();
  bool IsPointInFrontSector(glm::vec2 point);  // Check if point is in front sector
  glm::vec2 GetFrontAttachmentPoint();         // Get position where ball should stick
  void KickBall(GameObject& ball);
};
}  // namespace vis

#endif  // GAME_OBJECT_H