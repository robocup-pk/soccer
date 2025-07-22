#include <iostream>
#include <algorithm>

#include "SystemConfig.h"
#include "Kinematics.h"
#include "Coordinates.h"
#include "SoccerField.h"

// Global ball model instance
kin::BallModel kin::global_ball_model;

void kin::UpdateKinematics(std::vector<state::SoccerObject>& soccer_objects, float dt) {
  // First update ball physics with advanced model
  UpdateBallPhysics(soccer_objects, dt);
  
  // Then update all other objects with basic physics
  for (state::SoccerObject& soccer_object : soccer_objects) {
    if (soccer_object.name != "ball") {  // Skip ball, handled above
      soccer_object.Move(dt);
    }
  }
}

void kin::CheckAndResolveCollisions(std::vector<state::SoccerObject>& soccer_objects) {
  // Update Ball Attachment
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball" && obj.is_attached) {
      state::SoccerObject& robot = *obj.attached_to;
      UpdateAttachedBallPosition(robot, obj);
      break;
    }
  }

  // Check Collisions for all Soccer Objects
  for (int i = 0; i < soccer_objects.size(); ++i) {
    for (int j = i + 1; j < soccer_objects.size(); ++j) {
      state::SoccerObject& obj1 = soccer_objects[i];
      state::SoccerObject& obj2 = soccer_objects[j];

      // Normal collision detection continues for all other objects
      if (CheckCircularCollision(obj1, obj2)) {
        if (obj1.name == "ball") {
          if (IsBallInFrontOfRobot(obj2, obj1)) {
            if (obj1.is_attached) continue;  // Skip only this pair

            HandleBallSticking(obj2, obj1);
            continue;  // Continue to next pair
          }
        }

        if (obj2.name == "ball") {
          if (IsBallInFrontOfRobot(obj1, obj2)) {
            if (obj2.is_attached) continue;  // Skip only this pair

            HandleBallSticking(obj1, obj2);
            continue;  // Continue to next pair
          }
        }
        ResolveCircularCollision(obj1, obj2);
      }
    }
  }

  // Wall and boundary collisions still work
  ResolveCollisionWithWall(soccer_objects);

  for (state::SoccerObject& soccer_object : soccer_objects) {
    if (!IsInsideBoundary(soccer_object)) {
      ClampInsideBoundary(soccer_object);
    }
  }
}

void kin::ResolveCollisionWithWall(std::vector<state::SoccerObject>& soccer_objects) {
  for (state::SoccerObject& soccer_object : soccer_objects) {
    Eigen::Vector3d center = soccer_object.GetCenterPosition();
    float left = soccer_object.position[0] - (soccer_object.size[0] / 2);
    float right = soccer_object.position[0] + (soccer_object.size[0] / 2);
    float top = soccer_object.position[1] + (soccer_object.size[1] / 2);
    float bottom = soccer_object.position[1] - (soccer_object.size[1] / 2);

    double boundary_x = (vis::SoccerField::GetInstance().width_mm / 2.0f) / 1000.0f;
    double boundary_y = (vis::SoccerField::GetInstance().height_mm / 2.0f) / 1000.0f;

    // X-axis collision
    if ((right >= boundary_x && soccer_object.velocity[0] >= 0) ||
        (left <= -boundary_x && soccer_object.velocity[0] <= 0)) {
      soccer_object.position[0] =
          std::clamp(soccer_object.position[0], -boundary_x + (soccer_object.size[0]), boundary_x - (soccer_object.size[0]));
      soccer_object.velocity[0] *= -cfg::SystemConfig::wall_velocity_damping_factor;
    }

    // Y-axis collision
    if ((top > boundary_y && soccer_object.velocity[1] >= 0) ||
        (bottom < -boundary_y && soccer_object.velocity[1] <= 0)) {
      soccer_object.position[1] =
          std::clamp(soccer_object.position[1], -boundary_y + (soccer_object.size[1]), boundary_y - (soccer_object.size[1]));
      soccer_object.velocity[1] *= -cfg::SystemConfig::wall_velocity_damping_factor;
    }

  }
}

bool kin::IsInsideBoundary(const state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;
  float left = obj.position[0]- (obj.size[0] / 2);
  float right = obj.position[0] + (obj.size[0] / 2);
  float top = obj.position[1] + (obj.size[1] / 2);
  float bottom = obj.position[1] - (obj.size[1] / 2);
  return left >= -half_width && right <= half_width && top >= -half_height &&
         bottom <= half_height;
}

void kin::ClampInsideBoundary(state::SoccerObject& obj) {
  // x direction
  double half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  double half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;

  obj.position[0] = std::clamp(obj.position[0], -half_width, half_width - obj.size[0]);
  obj.position[1] = std::clamp(obj.position[1], -half_height, half_height - obj.size[1]);
}

bool kin::CheckCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2) {
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

void kin::ResolveCircularCollision(state::SoccerObject& obj1, state::SoccerObject& obj2) {
  Eigen::Vector3d pos1 = obj1.GetCenterPosition();
  Eigen::Vector3d pos2 = obj2.GetCenterPosition();

  // Calculate distance and direction
  float dx = pos2[0] - pos1[0];
  float dy = pos2[1] - pos1[1];
  float distance = sqrt(dx * dx + dy * dy);

  // Prevent division by zero
  if (distance < 0.001f) {
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

  // Assume equal mass for simplicity (you can add mass property to state::SoccerObject)
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

bool kin::IsBallInFrontOfRobot(state::SoccerObject& robot, state::SoccerObject& ball) {
  Eigen::Vector3d ball_center = ball.GetCenterPosition();
  Eigen::Vector2d ball_point(ball_center.x(), ball_center.y());
  return robot.IsPointInFrontSector(ball_point);
}

void kin::HandleBallSticking(state::SoccerObject& robot, state::SoccerObject& ball) {
  ball.is_attached = true;
  ball.attached_to = &robot;
  robot.is_attached = true;
  robot.attached_to = &ball;

  UpdateAttachedBallPosition(robot, ball);
}

void kin::UpdateAttachedBallPosition(state::SoccerObject& robot, state::SoccerObject& ball) {
  Eigen::Vector3d robot_center = robot.GetCenterPosition();
  float robot_rotation = robot.position[2];

  // Use flat surface distance for D-shaped robot
  float robot_radius = std::min(robot.size[0], robot.size[1]) / 2.0f;
  float ball_radius = ball.size[0] / 2.0f;

  // Attachment distance for flat surface contact
  float attachment_distance = robot_radius * 0.5f + ball_radius;

  // Calculate front position using unit vector
  float front_x = robot_center.x() + attachment_distance * cos(robot_rotation);
  float front_y = robot_center.y() + attachment_distance * sin(robot_rotation);

  // Position ball center at attachment point (not corner-based)
  ball.position[0] = front_x;
  ball.position[1] = front_y;
  ball.position[2] = 0;
  ball.velocity = Eigen::Vector3d(0, 0, 0);
}

void kin::DetachBall(state::SoccerObject& ball, float detach_velocity) {
  if (!ball.is_attached || !ball.attached_to) {
    return;  // Ball is not attached
  }

  state::SoccerObject* robot = ball.attached_to;

  // Get robot's current orientation for detachment direction
  float robot_rotation = robot->position[2];

  // Calculate front direction vector (same as in IsPointInFrontSector)
  Eigen::Vector2d front_dir(cos(robot_rotation), sin(robot_rotation));

  // Apply detach velocity in the front direction
  float detach_vel_x = detach_velocity * front_dir.x();
  float detach_vel_y = detach_velocity * front_dir.y();

  // Debug: Check if velocities make sense
  float velocity_magnitude = sqrt(detach_vel_x * detach_vel_x + detach_vel_y * detach_vel_y);

  // Move ball slightly away from robot before detaching to prevent immediate re-collision
  Eigen::Vector3d robot_center = robot->GetCenterPosition();
  float separation_distance = 0.2f;  // Distance to separate ball from robot

  // Position ball in front of robot using the same front direction
  ball.position[0] = robot_center.x() + separation_distance * front_dir.x();
  ball.position[1] = robot_center.y() + separation_distance * front_dir.y();
  // Set ball velocity for natural detachment
  ball.velocity[0] = detach_vel_x;
  ball.velocity[1] = detach_vel_y;
  ball.velocity[2] = 0;

  // Clear attachment state
  ball.is_attached = false;
  robot->is_attached = false;
  robot->attached_to = nullptr;
  ball.attached_to = nullptr;
}

// New BallModel integration functions
void kin::UpdateBallPhysics(std::vector<state::SoccerObject>& soccer_objects, float dt) {
  for (state::SoccerObject& obj : soccer_objects) {
    if (obj.name == "ball") {
      // Only update physics if ball is not attached to robot
      if (!obj.is_attached) {
        global_ball_model.UpdatePhysics(obj.position, obj.velocity, obj.acceleration, dt);
      }
      break;  // Assuming only one ball
    }
  }
}

void kin::ApplyKickToBall(state::SoccerObject& ball, const Eigen::Vector2d& kick_direction, double kick_power) {
  // Detach ball if it's attached to a robot
  if (ball.is_attached) {
    DetachBall(ball, 0.0f);  // Detach without extra velocity, kick will provide it
  }
  
  // Apply kick using the ball model
  global_ball_model.ApplyKick(ball.velocity, kick_direction, kick_power);
}

Eigen::Vector2d kin::PredictBallPosition(const state::SoccerObject& ball, double prediction_time) {
  return global_ball_model.PredictPosition(ball.position, ball.velocity, prediction_time);
}