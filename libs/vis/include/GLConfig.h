#ifndef GL_CONFIG_H
#define GL_CONFIG_H

#include "glm/glm.hpp"

namespace vis {

struct GLConfig {
  // System. TODO: make system config
  static constexpr int num_robots = 2;

  // Window
  static constexpr char* window_title = "RoboCup Simulator";
  static constexpr int window_width_px = 1200;
  static constexpr int window_height_px = 900;

  // Ball
  static constexpr double ball_radius_cm = 50;
  static constexpr glm::vec2 init_ball_pos = glm::vec2(0, 0);
  static constexpr glm::vec2 init_ball_velocity = glm::vec2(10, 0);

  // Robots
  static constexpr glm::vec2 robot_size_cm = glm::vec2(100, 100);

  // Movement

};
}  // namespace vis

#endif  // GL_CONFIG_H