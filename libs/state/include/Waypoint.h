#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <vector>
#include <cmath>
#include <cassert>

namespace state {

class Waypoint {
 public:
  Waypoint() = default;
  Waypoint(double x = 0, double y = 0, double angle = 0) : x(x), y(y), angle(angle) {}

  inline state::Waypoint operator-(const Waypoint& wp) const {
    return state::Waypoint(x - wp.x, y - wp.y, angle - wp.angle);
  }

  inline bool operator<(const Waypoint& wp) const {
    if (x < wp.x && y < wp.y) return true;
    return false;
  }

  inline state::Waypoint operator+(const Waypoint& wp) const {
    assert(wp.angle == angle && "[state::Waypoint] + with different angles");
    return state::Waypoint(x + wp.x, y + wp.y, angle);
  }

  inline state::Waypoint operator*(double n) { return state::Waypoint(x * n, y * n, angle); }

  double Norm() { return std::sqrt(x * x + y * y); }

  state::Waypoint Normalize() {
    double magnitude = Norm();
    x /= magnitude;
    y /= magnitude;
  }

  double x, y, angle;
};

typedef std::vector<Waypoint> Path;

bool NearPosition(const Waypoint& wp1, const Waypoint& wp2, double tolerance = 0.1) {
  return std::fabs(wp1.x - wp2.x) < tolerance && std::fabs(wp1.y - wp2.y) < tolerance;
}

bool NearPose(const Waypoint& wp1, const Waypoint& wp2, double tolerance = 0.1) {
  return NearPosition(wp1, wp2, tolerance) && std::fabs(wp1.angle - wp2.angle);
}

}  // namespace state

#endif  // WAYPOINT_H