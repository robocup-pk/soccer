#include "Waypoint.h"

bool state::NearPosition(const state::Waypoint& wp1, const state::Waypoint& wp2,
                         double tolerance) {
  return std::fabs(wp1.x - wp2.x) < tolerance && std::fabs(wp1.y - wp2.y) < tolerance;
}

bool state::NearPose(const state::Waypoint& wp1, const state::Waypoint& wp2, double tolerance) {
  return NearPosition(wp1, wp2, tolerance) && std::fabs(wp1.angle - wp2.angle);
}

std::ostream& state::operator<<(std::ostream& os, const state::Waypoint& wp) {
  os << "(" << wp.x << ", " << wp.y << ")";
  return os;
}

std::ostream& state::operator<<(std::ostream& os, const state::Path& path) {
  os << "[";
  for (size_t i = 0; i < path.size(); ++i) {
    os << path[i];
    if (i != path.size() - 1) os << ", ";
  }
  os << "]";
  return os;
}

std::ostream& state::operator<<(std::ostream& os, const std::vector<Eigen::Vector3d>& path) {
  for (int i = 0; i < path.size() - 1; ++i) {
    os << path[i].transpose() << " -> ";
  }
  os << path[path.size() - 1].transpose();
  return os;
}