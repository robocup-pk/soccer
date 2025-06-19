#include "GLConfig.h"

namespace vis {

Eigen::Vector2d GLConfig::GetRobotSize() {
  return cfg::SystemConfig::robot_size_ft * cfg::Coordinates::px_per_ft;
}

}  // namespace vis
