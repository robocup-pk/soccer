#include <iostream>
#include "Coordinates.h"
#include "SoccerObject.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  // soccer_objects.push_back(state::SoccerObject(
  //     "ball",
  //     Eigen::Vector3d(-cfg::SystemConfig::ball_radius_ft, cfg::SystemConfig::ball_radius_ft, 0),
  //     Eigen::Vector2d(cfg::SystemConfig::ball_radius_ft * 2,
  //                     cfg::SystemConfig::ball_radius_ft * 2),
  //     cfg::SystemConfig::init_ball_velocity_ftps, cfg::SystemConfig::init_ball_acceleration_ftpsps,
  //     1));

  // Robots
  for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_ft(i * 3, 0, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_ft, cfg::SystemConfig::robot_size_ft,
                            cfg::SystemConfig::init_robot_velocity_ftps,
                            cfg::SystemConfig::init_robot_acceleration_ftpsps, 10));
  }
}

bool state::SoccerObject::IsPointInFrontSector(Eigen::Vector2d point) {
  Eigen::Vector3d center = GetCenterPosition();
  Eigen::Vector2d robot_center(center.x(), center.y());

  // Calculate front direction and flip Y to match graphics coordinate system
  float rotation_rad = (position[2] - M_PI / 2.0f);
  Eigen::Vector2d front_dir(cos(rotation_rad), -sin(rotation_rad));

  Eigen::Vector2d to_point = point - robot_center;
  float forward_component = to_point.dot(front_dir);

  if (forward_component <= 0) {
    return false;
  }

  float dot_product = to_point.normalized().dot(front_dir);
  float angle_threshold = cos(M_PI / 6.0f);

  return dot_product > angle_threshold;
}