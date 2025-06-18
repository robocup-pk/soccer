#ifndef COORDINATES_H
#define COORDINATES_H

#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace cfg {

struct Coordinates {
  // Field size
  static constexpr float field_width_ft = 18;
  static constexpr float field_height_ft = 11;

  // Window size
  static constexpr int window_width_px = 1200;
  static constexpr int window_height_px = 733;

  // Conversion factor
  static constexpr float px_per_ft = window_width_px / field_width_ft;

  // Axis-flip conversion (not constexpr because of Eigen)
  static const Eigen::Vector3d ft_px_coords;
};

struct SystemConfig {
  static constexpr int num_robots = 2;
  static constexpr float wall_velocity_damping_factor = 0.9;

  // Ball
  static const Eigen::Vector3d init_ball_velocity_ftps;
  static const Eigen::Vector3d init_ball_acceleration_ftpsps;
  static constexpr float ball_radius_ft = 0.4;

  // Robots
  static const Eigen::Vector2d robot_size_ft;
  static constexpr float init_robot_speed_ftps = 3;
  static constexpr float max_robot_speed_ftps = 7.5;
  static const Eigen::Vector3d init_robot_acceleration_ftpsps;
};

}  // namespace cfg

#endif  // COORDINATES_H
