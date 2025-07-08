#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Eigen/Dense>

namespace cfg {
struct SystemConfig {
  static constexpr int num_robots = 1;
  static const Eigen::Vector3d max_velocity_fBody;

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
}

#endif // SYSTEM_CONFIG_H