#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

#include <Eigen/Dense>

namespace state {

class Waypoint {
 public:
  Waypoint(double x = 0, double y = 0, double angle = 0) : x(x), y(y), angle(angle) {}

  inline state::Waypoint operator-(const Waypoint& wp) const {
    return state::Waypoint(x - wp.x, y - wp.y, angle - wp.angle);
  }

  inline bool operator<(const Waypoint& wp) const {
    if (x < wp.x && y < wp.y) return true;
    return false;
  }

  inline bool operator>(const Waypoint& wp) const {
    if (x > wp.x && y > wp.y) return true;
    return false;
  }

  inline state::Waypoint operator+(const Waypoint& wp) const {
    assert(wp.angle == angle && "[state::Waypoint] + with different angles");
    return state::Waypoint(x + wp.x, y + wp.y, angle);
  }

  inline bool operator==(const Waypoint& wp) const {
    const double tolerance = 1e-6;  // Small tolerance for floating point comparison
    return (std::abs(x - wp.x) < tolerance && std::abs(y - wp.y) < tolerance &&
            std::abs(angle - wp.angle) < tolerance);
  }

  inline state::Waypoint operator*(double n) { return state::Waypoint(x * n, y * n, angle); }

  double Norm() { return std::sqrt(x * x + y * y); }

  state::Waypoint Normalize() {
    double magnitude = Norm();
    x /= magnitude;
    y /= magnitude;
    return *this;
  }

  double x, y, angle;
};

typedef std::vector<Waypoint> Path;

bool NearPosition(const Waypoint& wp1, const Waypoint& wp2, double tolerance = 0.1);
bool NearPose(const Waypoint& wp1, const Waypoint& wp2, double tolerance = 0.1);
std::ostream& operator<<(std::ostream& os, const Waypoint& wp);
std::ostream& operator<<(std::ostream& os, const Path& path);

std::ostream& operator<<(std::ostream& os, const std::vector<Eigen::Vector3d>& path);

}  // namespace state

#endif  // WAYPOINT_H