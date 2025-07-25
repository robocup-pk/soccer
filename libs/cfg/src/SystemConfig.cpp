#include "SystemConfig.h"
#include "Utils.h"
#include "GLConfig.h"
#include <iostream>

// Limits
const double cfg::SystemConfig::avg_velocity_fBody_mps = 0.5;
const Eigen::Vector3d cfg::SystemConfig::max_velocity_fBody_mps = Eigen::Vector3d(1, 1, 5);
const Eigen::Vector3d cfg::SystemConfig::max_acc_m_radpsps = Eigen::Vector3d(0.5, 0.5, 0.5);

// Wall
const float cfg::SystemConfig::wall_velocity_damping_factor = 0.3;

// Ball config
const float cfg::SystemConfig::ball_radius_m = 0.05;  // actual radius of golf ball 0.021335 m
const Eigen::Vector3d cfg::SystemConfig::init_ball_velocity_mps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_ball_acceleration_mpsps = Eigen::Vector3d(0.0, 0, 0);

// Robot config
const float cfg::SystemConfig::init_robot_speed_mps = 0;
const float cfg::SystemConfig::max_robot_speed_mps = 1;
const Eigen::Vector2d cfg::SystemConfig::robot_size_m = Eigen::Vector2d(0.204, 0.204);
const Eigen::Vector3d cfg::SystemConfig::init_robot_velocity_mps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_robot_acceleration_mpsps = Eigen::Vector3d(0, 0, 0);

double robot_width = 512.0 / cfg::Coordinates::px_per_m;

// team 1 initial formation
const std::vector<Eigen::Vector3d> cfg::SystemConfig::team_one_start_formation = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::num_robots);
  v[0] = Eigen::Vector3d(-0.5f, -0.0f, 0);
  v[1] = Eigen::Vector3d(-0.3f, 0.3f, 0);
  v[2] = Eigen::Vector3d(-0.3f, -0.3f, 0);
  v[3] = Eigen::Vector3d(-0.7f, -0.3f, 0);
  v[4] = Eigen::Vector3d(-0.7f, 0.3f, 0);
  v[5] = Eigen::Vector3d(-0.9f, -0.0f, 0);
  return v;
}();

// need to subtract robot_width from each x_coordinate bellow
// team 2 initial formation
const std::vector<Eigen::Vector3d> cfg::SystemConfig::team_two_start_formation = []() {
  std::vector<Eigen::Vector3d> v(SystemConfig::num_robots);
  double robot_width = (cfg::SystemConfig::robot_size_m)[0];
  robot_width = 0;

  // the reason this still looks off is because the center of the field is not (0,0)

  // once gyro rotation is working well rotate the robots on team 2 by 180 degrees so they face the
  // other goal
  v[0] = Eigen::Vector3d(0.5f, 0.0f, 0);
  v[1] = Eigen::Vector3d(0.3f, 0.3f, 0);
  v[2] = Eigen::Vector3d(0.3f, -0.3f, 0);
  v[3] = Eigen::Vector3d(0.7f, -0.3f, 0);
  v[4] = Eigen::Vector3d(0.7f, 0.3f, 0);
  v[5] = Eigen::Vector3d(0.9f, -0.0f, 0);
  return v;
}();