#include "Coordinates.h"

namespace cfg {
const float Coordinates::px_per_mm = 0.2f;

// Coordinate transformation: flip y-axis
const Eigen::Vector3d Coordinates::mm_px_coords = Eigen::Vector3d(1, -1, 1);

// Wall
const float SystemConfig::wall_velocity_damping_factor = 0.9;

// Ball config
const float SystemConfig::ball_radius_mm = 121.92;  // 0.4 ft to mm
const Eigen::Vector3d SystemConfig::init_ball_velocity_mmps = Eigen::Vector3d(0, 0, 0);
const Eigen::Vector3d SystemConfig::init_ball_acceleration_mmpsps = Eigen::Vector3d(0.0, 0, 0);

// Robot config
const float SystemConfig::init_robot_speed_mmps = 0;
const float SystemConfig::max_robot_speed_mmps = 2286;  // 7.5 ft to mm
const Eigen::Vector2d SystemConfig::robot_size_mm =
    Eigen::Vector2d(213.36, 213.36);  // 0.7 ft to mm
const Eigen::Vector3d SystemConfig::init_robot_velocity_mmps =
    Eigen::Vector3d(init_robot_speed_mmps, 0, 0);
const Eigen::Vector3d SystemConfig::init_robot_acceleration_mmpsps = Eigen::Vector3d(0, 0, 0);

}  // namespace cfg