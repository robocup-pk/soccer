#include "SystemConfig.h"
#include "GLConfig.h"

namespace vis {

Eigen::Vector2d GLConfig::GetRobotSize() {
  return cfg::SystemConfig::robot_size_m * cfg::Coordinates::px_per_m;
}

}  // namespace vis
