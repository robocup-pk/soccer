#ifndef COORDINATES_H
#define COORDINATES_H

#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace cfg {

struct Coordinates {
  // Conversion factor
  static const float px_per_m;  // Conversion factor from mm to pixels

  // Axis-flip conversion (not constexpr because of Eigen)
  static const Eigen::Vector3d m_px_coords;
};

}  // namespace cfg

#endif  // COORDINATES_H
