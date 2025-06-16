#include <iostream>

#include "GLConfig.h"
#include "Collision.h"

void vis::CheckCollision(std::map<std::string, GameObject>& game_objects) {
  // Let's just do collision between ball and wall:
  for (auto& [name, g] : game_objects) {
    if (name == "ball") {
      int ball_x = g.GetCenterPosition().x;
      int ball_y = g.GetCenterPosition().y;

      // x direction
      if (ball_x + g.size.x / 2 >= GLConfig::window_width_px / 2) {
        if (g.velocity.x > 0) {
          g.velocity.x *= -1;
        }
      }
      if (ball_x - g.size.x / 2 <= -GLConfig::window_width_px / 2) {
        std::cout << "Collision with left wall" << std::endl;
        if (g.velocity.x < 0) {
          g.velocity.x *= -1;
        }
      }

      // y direction
      if (ball_y + g.size.y / 2 >= GLConfig::window_height_px / 2) {
        if (g.velocity.y > 0) {
          g.velocity.y *= -1;
        }
      }
      if (ball_y - g.size.y / 2 <= -GLConfig::window_height_px / 2) {
        if (g.velocity.y < 0) {
          g.velocity.y *= -1;
        }
      }

      break;
    } else if (name == "robot0") {
      // std::cout << "robot0: " << g.GetCenterPosition().x << " " << g.GetCenterPosition().y
      //           << std::endl;
    }
  }

  for (auto it1 = game_objects.begin(); it1 != game_objects.end(); ++it1) {
    if (it1->second.name == "background") continue;
    auto it2 = it1;
    ++it2;
    for (; it2 != game_objects.end(); ++it2) {
      if (it2->second.name == "background") continue;

      GameObject& obj1 = it1->second;
      GameObject& obj2 = it2->second;

      /*
        // MY OWN IMPLEMENTATION
        // Positions of both the game objects
        glm::vec2 pos1 = obj1.GetCenterPosition();
        glm::vec2 pos2 = obj2.GetCenterPosition();

        // Check if they collide
        bool collision_x = std::fabs(pos1.x - pos2.x) < (obj1.size.x / 2 + obj2.size.x / 2);
        bool collision_y = std::fabs(pos1.y - pos2.y) < (obj1.size.y / 2 + obj2.size.y / 2);

        if (collision_x && collision_y) {
          if (std::fabs(pos1.x - pos2.x) < std::fabs(pos1.y - pos2.y)) {
            obj1.velocity.y *= -1;
            obj2.velocity.y *= -1;
          } else {
            obj1.velocity.x *= -1;
            obj2.velocity.x *= -1;
          }
        }
      */

      if (CheckCircularCollision(obj1, obj2)) {
        std::cout << it1->first << " and " << it2->first << " collide" << std::endl;
        ResolveCircularCollision(obj1, obj2);
      }
    }
  }
}

bool vis::CheckCircularCollision( vis::GameObject& obj1,  vis::GameObject& obj2) {
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

void vis::ResolveCircularCollision(vis::GameObject& obj1, vis::GameObject& obj2, int mass1, int mass2) {
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
  float m1 = mass1;    // obj1.mass;
  float m2 = mass2;  // obj2.mass;

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