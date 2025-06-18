#ifndef GL_CONFIG_H
#define GL_CONFIG_H

#include <Eigen/Dense>

#include "Coordinates.h"

namespace vis {

struct GLConfig {
  // Window
  static constexpr char* window_title = (char*)"RoboCup Simulator";

  // Ball
  static const double ball_radius;
  static const Eigen::Vector3d init_ball_pos;
  static const Eigen::Vector3d init_ball_velocity;
  static const Eigen::Vector3d init_ball_acceleration;

  // Robots
  static const Eigen::Vector2d robot_size;
  static const float init_robot_speed;
  static const float max_robot_speed;
  static const Eigen::Vector3d init_robot_acceleration;
  static Eigen::Vector2d GetRobotSize();
};

}  // namespace vis

#endif  // GL_CONFIG_H
