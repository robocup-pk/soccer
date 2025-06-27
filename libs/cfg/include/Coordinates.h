#ifndef COORDINATES_H
#define COORDINATES_H

#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace cfg {

struct Coordinates {
  // Field size
  static const float field_width_ft;
  static const float field_height_ft;

  // Window size
  static const int window_width_px;
  static const int window_height_px;

  // Conversion factor
  static const float px_per_ft;

  // Axis-flip conversion (not constexpr because of Eigen)
  static const Eigen::Vector3d ft_px_coords;
};

struct SystemConfig {
  static constexpr int num_robots = 1;

  // Wall
  static const float wall_velocity_damping_factor;

  // Ball
  static const Eigen::Vector3d init_ball_velocity_ftps;
  static const Eigen::Vector3d init_ball_acceleration_ftpsps;
  static const float ball_radius_ft;

  // Robots
  static const Eigen::Vector2d robot_size_ft;
  static const float init_robot_speed_ftps;
  static const float max_robot_speed_ftps;
  static const Eigen::Vector3d init_robot_velocity_ftps;
  static const Eigen::Vector3d init_robot_acceleration_ftpsps;
};

}  // namespace cfg

#endif  // COORDINATES_H
