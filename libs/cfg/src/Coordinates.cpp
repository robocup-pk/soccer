#include "Coordinates.h"

namespace cfg {
// Field size
const float Coordinates::field_width_ft = 16;
const float Coordinates::field_height_ft = 10;
const float Coordinates::px_per_ft = 75;

// Window size
const int Coordinates::window_width_px = Coordinates::field_width_ft * Coordinates::px_per_ft;
const int Coordinates::window_height_px = Coordinates::field_height_ft * Coordinates::px_per_ft;

// Coordinate transformation: flip y-axis
const Eigen::Vector3d Coordinates::ft_px_coords = Eigen::Vector3d(1, -1, 1);

// Wall
const float SystemConfig::wall_velocity_damping_factor = 0.9;

// Ball config
const float SystemConfig::ball_radius_ft = 0.4;
const Eigen::Vector3d SystemConfig::init_ball_velocity_ftps = Eigen::Vector3d(2, 2, 0);
const Eigen::Vector3d SystemConfig::init_ball_acceleration_ftpsps = Eigen::Vector3d(-7, -7, 0);

// Robot config
const float SystemConfig::init_robot_speed_ftps = 3;
const float SystemConfig::max_robot_speed_ftps = 7.5;
const Eigen::Vector2d SystemConfig::robot_size_ft = Eigen::Vector2d(1.5, 1.5);
const Eigen::Vector3d SystemConfig::init_robot_velocity_ftps =
    Eigen::Vector3d(init_robot_speed_ftps, 0, 0);
const Eigen::Vector3d SystemConfig::init_robot_acceleration_ftpsps = Eigen::Vector3d(0, 0, 0);

}  // namespace cfg
