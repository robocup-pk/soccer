#include <iostream>
#include <algorithm>

#include "Collision.h"
#include "Coordinates.h"

void state::CheckAndResolveCollisions(std::vector<SoccerObject>& soccer_objects) {
  for (int i = 0; i < soccer_objects.size(); ++i) {
    if (soccer_objects[i].name == "background") continue;

    for (int j = i + 1; j < soccer_objects.size(); ++j) {
      if (soccer_objects[j].name == "background") continue;

      SoccerObject& obj1 = soccer_objects[i];
      SoccerObject& obj2 = soccer_objects[j];

      if (CheckCircularCollision(obj1, obj2)) {
        ResolveCircularCollision(obj1, obj2);
      }
    }
  }

  // Collision with the wall
  ResolveCollisionWithWall(soccer_objects);

  // Stay inside the boundary
  for (SoccerObject& soccer_object : soccer_objects) {
    if (!IsInsideBoundary(soccer_object)) {
      ClampInsideBoundary(soccer_object);
    }
  }
}

void state::ResolveCollisionWithWall(std::vector<SoccerObject>& soccer_objects) {
  for (SoccerObject& soccer_object : soccer_objects) {
    if (soccer_object.name == "background") continue;

    Eigen::Vector3d center = soccer_object.GetCenterPosition();
    float left = soccer_object.position[0];
    float right = soccer_object.position[0] + soccer_object.size[0];
    float top = soccer_object.position[1];
    float bottom = soccer_object.position[1] - soccer_object.size[1];

    double boundary_x = cfg::Coordinates::field_width_ft / 2.0f;
    double boundary_y = cfg::Coordinates::field_height_ft / 2.0f;

    // X-axis collision
    if ((right > boundary_x && soccer_object.velocity[0] > 0) ||
        (left < -boundary_x && soccer_object.velocity[0] < 0)) {
      soccer_object.position[0] =
          std::clamp(soccer_object.position[0], -boundary_x, boundary_x - soccer_object.size[0]);
      soccer_object.velocity[0] *= -cfg::SystemConfig::wall_velocity_damping_factor;
    }

    // Y-axis collision
    if ((top > boundary_y && soccer_object.velocity[1] > 0) ||
        (bottom < -boundary_y && soccer_object.velocity[1] < 0)) {
      soccer_object.position[1] =
          std::clamp(soccer_object.position[1], -boundary_y + soccer_object.size[1], boundary_y);
      soccer_object.velocity[1] *= -1;
      soccer_object.velocity[1] *= cfg::SystemConfig::wall_velocity_damping_factor;
    }
  }
}

bool state::IsInsideBoundary(const SoccerObject& obj) {
  float half_width = cfg::Coordinates::window_width_px / 2;
  float half_height = cfg::Coordinates::window_height_px / 2;
  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1];
  float bottom = obj.position[1] + obj.size[1];

  return left >= -half_width && right <= half_width && top >= -half_height &&
         bottom <= half_height;
}

void state::ClampInsideBoundary(SoccerObject& obj) {
  if (obj.name == "background") return;
  // x direction
  double half_width = cfg::Coordinates::window_width_px / 2;
  double half_height = cfg::Coordinates::window_height_px / 2;

  obj.position[0] = std::clamp(obj.position[0], -half_width, half_width - obj.size[0]);
  obj.position[1] = std::clamp(obj.position[1], -half_height, half_height - obj.size[1]);
}

bool state::CheckCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2) {
  Eigen::Vector3d pos1 = obj1.GetCenterPosition();
  Eigen::Vector3d pos2 = obj2.GetCenterPosition();

  // Calculate distance between centers
  float dx = pos2[0] - pos1[0];
  float dy = pos2[1] - pos1[1];
  float distance = sqrt(dx * dx + dy * dy);

  // Calculate combined radius
  float radius1 = std::min(obj1.size[0], obj1.size[1]) / 2.0f;
  float radius2 = std::min(obj2.size[0], obj2.size[1]) / 2.0f;

  return distance <= (radius1 + radius2);
}

void state::ResolveCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2) {
  Eigen::Vector3d pos1 = obj1.GetCenterPosition();
  Eigen::Vector3d pos2 = obj2.GetCenterPosition();

  // Calculate distance and direction
  float dx = pos2[0] - pos1[0];
  float dy = pos2[1] - pos1[1];
  float distance = sqrt(dx * dx + dy * dy);

  // Prevent division by zero
  if (distance < 0.0001f) {
    // Objects are at same position, separate them artificially
    dx = 1.0f;
    dy = 0.0f;
    distance = 1.0f;
  }

  // Calculate radii
  float radius1 = std::min(obj1.size[0], obj1.size[1]) / 2.0f;
  float radius2 = std::min(obj2.size[0], obj2.size[1]) / 2.0f;

  // Calculate normal vector (unit vector from obj1 to obj2)
  float nx = dx / distance;
  float ny = dy / distance;

  // Calculate tangential vector (perpendicular to normal)
  float tx = -ny;
  float ty = nx;

  // Project velocities onto normal and tangential directions
  float v1n = obj1.velocity[0] * nx + obj1.velocity[1] * ny;  // obj1 normal component
  float v1t = obj1.velocity[0] * tx + obj1.velocity[1] * ty;  // obj1 tangential component
  float v2n = obj2.velocity[0] * nx + obj2.velocity[1] * ny;  // obj2 normal component
  float v2t = obj2.velocity[0] * tx + obj2.velocity[1] * ty;  // obj2 tangential component

  // Assume equal mass for simplicity (you can add mass property to SoccerObject)
  float m1 = obj1.mass_kg;
  float m2 = obj2.mass_kg;

  // Calculate new normal velocities (1D elastic collision)
  float v1n_new = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2);
  float v2n_new = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2);

  // Convert back to 2D velocities
  obj1.velocity[0] = v1n_new * nx + v1t * tx;
  obj1.velocity[1] = v1n_new * ny + v1t * ty;
  obj2.velocity[0] = v2n_new * nx + v2t * tx;
  obj2.velocity[1] = v2n_new * ny + v2t * ty;

  // Separate overlapping objects
  float overlap = (radius1 + radius2) - distance;
  if (overlap > 0) {
    float separation = overlap / 2.0f;

    // Move objects apart along the normal
    obj1.position[0] -= separation * nx;
    obj1.position[1] -= separation * ny;
    obj2.position[0] += separation * nx;
    obj2.position[1] += separation * ny;
  }
}