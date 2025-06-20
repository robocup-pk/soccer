#ifndef GL_CONFIG_H
#define GL_CONFIG_H

#include <Eigen/Dense>

#include "Coordinates.h"

namespace vis {

struct GLConfig {
  // Window
  static constexpr char* window_title = (char*)"RoboCup Simulator";
  static Eigen::Vector2d GetRobotSize();
};

}  // namespace vis

#endif  // GL_CONFIG_H
