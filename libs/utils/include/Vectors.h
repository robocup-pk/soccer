#ifndef VECTORS_H
#define VECTORS_H

#include <Eigen/Dense>
#include "glm/glm.hpp"

namespace util {

glm::vec2 operator*(const Eigen::Vector2f& eigen_vec, const glm::vec2& glm_vec) {
  return glm::vec2(eigen_vec.x() * glm_vec.x, eigen_vec.y() * glm_vec.y);
}

}  // namespace util

#endif  // VECTORS_H