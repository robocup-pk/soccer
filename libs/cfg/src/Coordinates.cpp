#include "Coordinates.h"

namespace cfg {

// Coordinate transformation: flip y-axis
const Eigen::Vector3d Coordinates::ft_px_coords = Eigen::Vector3d(1, -1, 1);

// Ball config
const Eigen::Vector3d SystemConfig::init_ball_velocity_ftps = Eigen::Vector3d(2, 2, 0);
const Eigen::Vector3d SystemConfig::init_ball_acceleration_ftpsps = Eigen::Vector3d(-7, -7, 0);

// Robot config
const Eigen::Vector2d SystemConfig::robot_size_ft = Eigen::Vector2d(1.5, 1.5);
const Eigen::Vector3d SystemConfig::init_robot_acceleration_ftpsps = Eigen::Vector3d(0, 0, 0);

}  // namespace cfg
