#include <iostream>
#include <algorithm>

#include "GLConfig.h"
#include "Collision.h"

void vis::CheckAndResolveCollisions(std::map<std::string, GameObject>& game_objects) {
  for (auto it1 = game_objects.begin(); it1 != game_objects.end(); ++it1) {
    if (it1->second.name == "background") continue;
    auto it2 = it1;
    ++it2;
    for (; it2 != game_objects.end(); ++it2) {
      if (it2->second.name == "background") continue;
      GameObject& obj1 = it1->second;
      GameObject& obj2 = it2->second;

      if (CheckCircularCollision(obj1, obj2)) {
        ResolveCircularCollision(obj1, obj2);
      }
    }
  }

  // Collision with the wall
  ResolveCollisionWithWall(game_objects);

  // Stay inside the boundary
  for (auto& [name, g] : game_objects) {
    if (!IsInsideBoundary(g)) {
      ClampInsideBoundary(g);
    }
  }
}

void vis::ResolveCollisionWithWall(std::map<std::string, GameObject>& game_objects) {
  for (auto& [name, g] : game_objects) {
    if (name == "background") continue;

    glm::vec2 center = g.GetCenterPosition();
    float left = g.position.x;
    float right = g.position.x + g.size.x;
    float top = g.position.y;
    float bottom = g.position.y + g.size.y;

    float boundary_x = cfg::Coordinates::window_width_px / 2.0f;
    float boundary_y = cfg::Coordinates::window_height_px / 2.0f;

    // X-axis collision
    if ((right > boundary_x && g.velocity.x > 0) || (left < -boundary_x && g.velocity.x < 0)) {
      g.position.x = std::clamp(g.position.x, -boundary_x, boundary_x - g.size.x);
      g.velocity.x *= -1;
      g.velocity.x *= cfg::SystemConfig::wall_velocity_damping_factor;
    }

    // Y-axis collision
    if ((bottom > boundary_y && g.velocity.y > 0) || (top < -boundary_y && g.velocity.y < 0)) {
      g.position.y = std::clamp(g.position.y, -boundary_y, boundary_y - g.size.y);
      g.velocity.y *= -1;
      g.velocity.y *= cfg::SystemConfig::wall_velocity_damping_factor;
    }
  }
}

bool vis::IsInsideBoundary(const GameObject& obj) {
  float half_width = cfg::Coordinates::window_width_px / 2;
  float half_height = cfg::Coordinates::window_height_px / 2;
  float left = obj.position.x;
  float right = obj.position.x + obj.size.x;
  float top = obj.position.y;
  float bottom = obj.position.y + obj.size.y;

  return left >= -half_width && right <= half_width && top >= -half_height &&
         bottom <= half_height;
}

void vis::ClampInsideBoundary(GameObject& obj) {
  if (obj.name == "background") return;
  // x direction
  float half_width = cfg::Coordinates::window_width_px / 2;
  float half_height = cfg::Coordinates::window_height_px / 2;

  obj.position.x = std::clamp(obj.position.x, -half_width, half_width - obj.size.x);
  obj.position.y = std::clamp(obj.position.y, -half_height, half_height - obj.size.y);
}

bool vis::CheckCircularCollision(vis::GameObject& obj1, vis::GameObject& obj2) {
  glm::vec2 pos1 = obj1.GetCenterPosition();
  glm::vec2 pos2 = obj2.GetCenterPosition();

  // Calculate distance between centers
  float dx = pos2.x - pos1.x;
  float dy = pos2.y - pos1.y;
  float distance = sqrt(dx * dx + dy * dy);

  // Calculate combined radius
  float radius1 = std::min(obj1.size.x, obj1.size.y) / 2.0f;
  float radius2 = std::min(obj2.size.x, obj2.size.y) / 2.0f;

  return distance <= (radius1 + radius2);
}

void vis::ResolveCircularCollision(vis::GameObject& obj1, vis::GameObject& obj2) {
  glm::vec2 pos1 = obj1.GetCenterPosition();
  glm::vec2 pos2 = obj2.GetCenterPosition();

  // Calculate distance and direction
  float dx = pos2.x - pos1.x;
  float dy = pos2.y - pos1.y;
  float distance = sqrt(dx * dx + dy * dy);

  // Prevent division by zero
  if (distance < 0.0001f) {
    // Objects are at same position, separate them artificially
    dx = 1.0f;
    dy = 0.0f;
    distance = 1.0f;
  }

  // Calculate radii
  float radius1 = std::min(obj1.size.x, obj1.size.y) / 2.0f;
  float radius2 = std::min(obj2.size.x, obj2.size.y) / 2.0f;

  // Calculate normal vector (unit vector from obj1 to obj2)
  float nx = dx / distance;
  float ny = dy / distance;

  // Calculate tangential vector (perpendicular to normal)
  float tx = -ny;
  float ty = nx;

  // Project velocities onto normal and tangential directions
  float v1n = obj1.velocity.x * nx + obj1.velocity.y * ny;  // obj1 normal component
  float v1t = obj1.velocity.x * tx + obj1.velocity.y * ty;  // obj1 tangential component
  float v2n = obj2.velocity.x * nx + obj2.velocity.y * ny;  // obj2 normal component
  float v2t = obj2.velocity.x * tx + obj2.velocity.y * ty;  // obj2 tangential component

  // Assume equal mass for simplicity (you can add mass property to GameObject)
  float m1 = obj1.mass_kg;
  float m2 = obj2.mass_kg;

  // Calculate new normal velocities (1D elastic collision)
  float v1n_new = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2);
  float v2n_new = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2);

  // Convert back to 2D velocities
  obj1.velocity.x = v1n_new * nx + v1t * tx;
  obj1.velocity.y = v1n_new * ny + v1t * ty;
  obj2.velocity.x = v2n_new * nx + v2t * tx;
  obj2.velocity.y = v2n_new * ny + v2t * ty;

  // Separate overlapping objects
  float overlap = (radius1 + radius2) - distance;
  if (overlap > 0) {
    float separation = overlap / 2.0f;

    // Move objects apart along the normal
    obj1.position.x -= separation * nx;
    obj1.position.y -= separation * ny;
    obj2.position.x += separation * nx;
    obj2.position.y += separation * ny;
  }
}