#include "SystemConfig.h"

// Limits
const Eigen::Vector3d cfg::SystemConfig::max_velocity_fBody_mps = Eigen::Vector3d(1, 1, 1);
const Eigen::Vector3d cfg::SystemConfig::max_acceleration_mpsps_radpsps =
    Eigen::Vector3d(0.3, 0.3, 0.3);

// Wall
const float cfg::SystemConfig::wall_velocity_damping_factor = 0.9;

// Ball config
const float cfg::SystemConfig::ball_radius_m = 0.12192;  // 0.4 ft to mm
const Eigen::Vector3d cfg::SystemConfig::init_ball_velocity_mps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_ball_acceleration_mpsps = Eigen::Vector3d(0.0, 0, 0);

// Robot config
const float cfg::SystemConfig::init_robot_speed_mps = 0;
const float cfg::SystemConfig::max_robot_speed_mps = 1;
const Eigen::Vector2d cfg::SystemConfig::robot_size_m = Eigen::Vector2d(0.204, 0.204);
const Eigen::Vector3d cfg::SystemConfig::init_robot_velocity_mps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_robot_acceleration_mpsps = Eigen::Vector3d(0, 0, 0);
