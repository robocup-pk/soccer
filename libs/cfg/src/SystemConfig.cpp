#include "SystemConfig.h"

// Limits
const Eigen::Vector3d cfg::SystemConfig::max_velocity_fBody = Eigen::Vector3d(1, 1, 1);

// Wall
const float cfg::SystemConfig::wall_velocity_damping_factor = 0.9;

// Ball config
const float cfg::SystemConfig::ball_radius_ft = 0.4;
const Eigen::Vector3d cfg::SystemConfig::init_ball_velocity_ftps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_ball_acceleration_ftpsps =
    Eigen::Vector3d(0.0, 0, 0);

// Robot config
const float cfg::SystemConfig::init_robot_speed_ftps = 0;
const float cfg::SystemConfig::max_robot_speed_ftps = 7.5;
const Eigen::Vector2d cfg::SystemConfig::robot_size_ft = Eigen::Vector2d(0.7, 0.7);
const Eigen::Vector3d cfg::SystemConfig::init_robot_velocity_ftps =
    Eigen::Vector3d(init_robot_speed_ftps, 0, 0);
const Eigen::Vector3d cfg::SystemConfig::init_robot_acceleration_ftpsps = Eigen::Vector3d(0, 0, 0);