#ifndef GL_CONFIG_H
#define GL_CONFIG_H

#include "glm/glm.hpp"

#include "Coordinates.h"

namespace vis {

struct GLConfig {
  // Window
  static constexpr char* window_title = (char*)"RoboCup Simulator";

  // Ball
  static constexpr double ball_radius =
      cfg::SystemConfig::ball_radius_ft * cfg::Coordinates::px_per_ft;
  static constexpr glm::vec2 init_ball_pos =
      glm::vec2(-ball_radius, ball_radius) * cfg::Coordinates::ft_to_px_coords;
  static constexpr glm::vec2 init_ball_velocity = cfg::SystemConfig::init_ball_velocity_ftps *
                                                  cfg::Coordinates::px_per_ft *
                                                  cfg::Coordinates::ft_to_px_coords;
  static constexpr glm::vec2 init_ball_acceleration =
      cfg::SystemConfig::init_ball_acceleration_ftpsps * cfg::Coordinates::px_per_ft *
      cfg::Coordinates::ft_to_px_coords;

  // Robots
  static constexpr glm::vec2 robot_size =
      cfg::SystemConfig::robot_size_ft * cfg::Coordinates::px_per_ft;
  static constexpr float init_robot_speed =
      cfg::SystemConfig::init_robot_speed_ftps * cfg::Coordinates::px_per_ft;
  static constexpr float max_robot_speed =
      cfg::SystemConfig::max_robot_speed_ftps * cfg::Coordinates::px_per_ft;
  static constexpr glm::vec2 init_robot_acceleration =
      cfg::SystemConfig::init_robot_acceleration_ftpsps * cfg::Coordinates::px_per_ft *
      cfg::Coordinates::ft_to_px_coords;

  // Robot Rotation Speed
  static constexpr float init_robot_rotation_speed = 90.0f;

  // Movement
};
}  // namespace vis

#endif  // GL_CONFIG_H