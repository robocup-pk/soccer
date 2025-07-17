#include <iostream>

#include "SystemConfig.h"
#include "Trajectory3D.h"

std::pair<bool, std::optional<Eigen::Vector3d>> ctrl::Trajectory3D::IsFeasible(
    Eigen::Vector3d h, double T, Eigen::Vector3d v0) {
  std::cout << "[ctrl::Trajectory3D::IsFeasible] h: " << h.transpose()
            << ". v0: " << v0.transpose() << ". T: " << T << std::endl;
  Eigen::Vector3d t_acc_s(-1, -1, -1);
  Eigen::Vector3d a = cfg::SystemConfig::max_acc_m_radpsps;
  Eigen::Vector3d disc;  // Discriminant
  Eigen::Vector3d v_cruise;

  for (int i = 0; i < 3; ++i) {
    if (v0[i] * h[i] >= 0) {
      // Case 1: No sign change
      // Feasibility condition:
      disc[i] = a[i] * a[i] * T * T - 4 * a[i] * std::fabs(h[i]) +
                2 * a[i] * std::fabs(v0[i]) * T - v0[i] * v0[i];
      if (disc[i] <= 0) return {false, std::nullopt};

      v_cruise[i] = 0.5 * (a[i] * T + std::fabs(v0[i]) - std::sqrt(disc[i]));
      // Take care of the sign here
      if (h[i] < 0) v_cruise[i] *= -1;
    } else {
      // Case 2: Sign change

      // Triangular part
      double d1;
      d1 = v0[i] * v0[i] / (2 * a[i]);
      if (v0[i] < 0) d1 *= -1;
      double t1 = std::fabs(v0[i]) / a[i];
      if (t1 >= T) return {false, std::nullopt};

      // Trapezoidal part
      double t2 = T - t1;
      double d2 = h[i] - d1;
      disc[i] = a[i] * a[i] * t2 * t2 - 4 * a[i] * std::fabs(d2);
      if (disc[i] <= 0) return {false, std::nullopt};

      for (int i = 0; i < 3; ++i) {
        v_cruise[i] = 0.5 * (a[i] * t2 - std::sqrt(disc[i]));
        // Take care of the sign here
        if (h[i] < 0) v_cruise[i] *= -1;
      }
    }
  }
  return {true, v_cruise};
}