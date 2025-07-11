#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Eigen/Dense>

namespace cfg {
struct SystemConfig {
  static constexpr int num_robots = 1;
  static const Eigen::Vector3d max_velocity_fBody_mps;
  static const Eigen::Vector3d max_acceleration_mpsps_radpsps;

  // Wall
  static const float wall_velocity_damping_factor;

  // Ball
  static const Eigen::Vector3d init_ball_velocity_mps;
  static const Eigen::Vector3d init_ball_acceleration_mpsps;
  static const float ball_radius_m;

  // Robots
  static const Eigen::Vector2d robot_size_m;
  static const float init_robot_speed_mps;
  static const float max_robot_speed_mps;
  static const Eigen::Vector3d init_robot_velocity_mps;
  static const Eigen::Vector3d init_robot_acceleration_mpsps;
};
}  // namespace cfg

#endif  // SYSTEM_CONFIG_H