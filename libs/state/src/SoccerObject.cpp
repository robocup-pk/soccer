#include <iostream>
#include "Coordinates.h"
#include "SoccerObject.h"
#include "SystemConfig.h"
#include "Kinematics.h"
#include "BallModel.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  // Robots (team one)
  for (int i = 0; i < cfg::SystemConfig::num_robots / 2; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_m =
        i < cfg::RightRobotHomeCoordinates.size()
            ? cfg::RightRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(i))
            : Eigen::Vector3d(0.3, 0, 0);
    soccer_objects.push_back(state::SoccerObject(
        name, robot_position_m, cfg::SystemConfig::robot_size_m, 1,
        cfg::SystemConfig::init_robot_velocity_mps,
        cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
  }

  // Robots (team two)
  for (int i = cfg::SystemConfig::num_robots / 2; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    int index = i - cfg::SystemConfig::num_robots / 2;
    Eigen::Vector3d robot_position_m =
        index < cfg::LeftRobotHomeCoordinates.size()
            ? cfg::LeftRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(index))
            : Eigen::Vector3d(-0.3, 0, 0);
    soccer_objects.push_back(state::SoccerObject(
        name, robot_position_m, cfg::SystemConfig::robot_size_m, 2,
        cfg::SystemConfig::init_robot_velocity_mps,
        cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
  }

  // Ball
  soccer_objects.push_back(state::SoccerObject(
      "ball", cfg::SystemConfig::init_ball_position,
      Eigen::Vector2d(cfg::SystemConfig::ball_radius_m * 2, cfg::SystemConfig::ball_radius_m * 2),
      0, cfg::SystemConfig::init_ball_velocity_mps,
      cfg::SystemConfig::init_ball_acceleration_mpsps, cfg::SystemConfig::ball_mass_kg));
}

bool state::SoccerObject::IsPointInFrontSector(Eigen::Vector2d point) {
  Eigen::Vector3d center = GetCenterPosition();
  Eigen::Vector2d robot_center(center.x(), center.y());

  // Calculate front direction using same coordinate system as ball attachment
  float rotation_rad = position[2]; // Use the z-component as the angle
  Eigen::Vector2d front_dir(cos(rotation_rad), sin(rotation_rad));

  // Vector from robot center to point
  Eigen::Vector2d to_point = point - robot_center;
  float distance = to_point.norm();

  // Check if point is within maximum detection distance
  const float MAX_DISTANCE = 0.3f; // 30cm
  if (distance > MAX_DISTANCE) return false;

  // Check if point is in the front sector (120 degree cone)
  float dot_product = front_dir.dot(to_point.normalized());
  const float ANGLE_THRESHOLD = cos(60.0f * M_PI / 180.0f); // 60 degrees = half of 120 degree cone

  return dot_product > ANGLE_THRESHOLD;
}

Eigen::Vector3d state::SoccerObject::GetCenterPosition() {
  return position;
}

void state::SoccerObject::SetRobotRole(state::SoccerObject::Role r) {
  role = r;
}

void state::SoccerObject::Move(float dt) {
  // Basic kinematic update
  velocity += acceleration * dt;
  position += velocity * dt;
}