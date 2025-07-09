#include "Coordinates.h"

namespace cfg {
const float Coordinates::px_per_m = 200.0f;

// Coordinate transformation: flip y-axis
const Eigen::Vector3d Coordinates::m_px_coords = Eigen::Vector3d(1, -1, 1);
}  // namespace cfg