#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Eigen/Dense>

namespace cfg {
struct SystemConfig {
  static constexpr int num_robots = 12;
  static const Eigen::Vector3d max_velocity_fBody_mps;
  static const Eigen::Vector3d max_acc_m_radpsps;
  static const double avg_velocity_fBody_mps;

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

  // Robot Team
  static const std::vector<Eigen::Vector3d> team_one_start_formation;
  static const std::vector<Eigen::Vector3d> team_two_start_formation;

  // players that are responsible for initiating the free kick
  static const int team_one_kicker = 0;
  static const int team_two_kicker = num_robots / 2;
};
}  // namespace cfg

#endif  // SYSTEM_CONFIG_H