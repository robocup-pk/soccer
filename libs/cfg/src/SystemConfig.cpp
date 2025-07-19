#include "SystemConfig.h"

// Limits
const double cfg::SystemConfig::avg_velocity_fBody_mps = 0.5;
const Eigen::Vector3d cfg::SystemConfig::max_velocity_fBody_mps = Eigen::Vector3d(1, 1, 1);
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
