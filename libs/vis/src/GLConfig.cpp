#include "GLConfig.h"

namespace vis {

// Ball
const double GLConfig::ball_radius =
    cfg::SystemConfig::ball_radius_ft * cfg::Coordinates::px_per_ft;

const Eigen::Vector3d GLConfig::init_ball_pos =
    Eigen::Vector3d(-ball_radius, ball_radius, 0).cwiseProduct(cfg::Coordinates::ft_px_coords);

const Eigen::Vector3d GLConfig::init_ball_velocity =
    cfg::SystemConfig::init_ball_velocity_ftps.cwiseProduct(cfg::Coordinates::ft_px_coords) *
    cfg::Coordinates::px_per_ft;

const Eigen::Vector3d GLConfig::init_ball_acceleration =
    cfg::SystemConfig::init_ball_acceleration_ftpsps.cwiseProduct(cfg::Coordinates::ft_px_coords) *
    cfg::Coordinates::px_per_ft;

// Robots
const Eigen::Vector2d GLConfig::robot_size =
    Eigen::Vector2d(50, 50);  // cfg::SystemConfig::robot_size_ft * cfg::Coordinates::px_per_ft;

const float GLConfig::init_robot_speed =
    cfg::SystemConfig::init_robot_speed_ftps * cfg::Coordinates::px_per_ft;

const float GLConfig::max_robot_speed =
    cfg::SystemConfig::max_robot_speed_ftps * cfg::Coordinates::px_per_ft;

const Eigen::Vector3d GLConfig::init_robot_acceleration =
    cfg::SystemConfig::init_robot_acceleration_ftpsps.cwiseProduct(
        cfg::Coordinates::ft_px_coords) *
    cfg::Coordinates::px_per_ft;

Eigen::Vector2d GLConfig::GetRobotSize() {
  return cfg::SystemConfig::robot_size_ft * cfg::Coordinates::px_per_ft;
}

}  // namespace vis
