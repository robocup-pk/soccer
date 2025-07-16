#include <iostream>

#include "SystemConfig.h"
#include "Trajectory3D.h"

std::pair<bool, std::optional<Eigen::Vector3d>> ctrl::Trajectory3D::IsFeasible(
    Eigen::Vector3d h, double T, Eigen::Vector3d v0) {
  std::cout << "[ctrl::Trajectory3D::IsFeasibleNonZeroVelocity] h: " << h.transpose()
            << ". v0: " << v0.transpose() << ". T: " << T << std::endl;
  Eigen::Vector3d t_acc_s(-1, -1, -1);
  Eigen::Vector3d a = cfg::SystemConfig::max_acc_m_radpsps;
  Eigen::Vector3d disc;  // Discriminant

  for (int i = 0; i < 3; ++i) {
    if (v0[i] * h[i] >= 0) {
      // Case 1: No sign change
      // Feasibility condition:
      disc[i] = a[i] * a[i] * T * T - 4 * a[i] * std::fabs(h[i]) +
                2 * a[i] * std::fabs(v0[i]) * T - v0[i] * v0[i];
      if (disc[i] <= 0) return {false, std::nullopt};
    } else {
      // Case 2: Sign change
      // double distance_1 = v0[i] * v0[i] / (2 * cfg::SystemConfig::max_acc_m_radpsps[i]);
      // double total_time_1 = std::fabs(v0[i] / cfg::SystemConfig::max_acc_m_radpsps[i]);
      // distance_m_rad[i] -= distance_1;

      // // Feasibility condition:
      // // Move from v0 to 0

      // // Then follow the normal trapezoidal
      // std::cout << "Distance 1: " << distance_1 << ". Distance 2: " << distance_m_rad[i]
      //           << std::endl;
      // std::pair<bool, std::optional<double>> traj_t_acc =
      //     IsFeasible(distance_m_rad[i], total_time_s - total_time_1, i);
      // if (!traj_t_acc.first) {
      // std::cout << "[ctrl::Trajectory3D::IsFeasibleNonZeroVelocity] Trajectory not "
      //              "feasible. Try different params"
      //           << std::endl;
      // return;
      // }
      // t_acc_s[i] = traj_t_acc.second.value();
    }
  }

  // It is feasible. Calculate the parameters
  Eigen::Vector3d v_cruise;
  for (int i = 0; i < 3; ++i) {
    v_cruise[i] = 0.5 * (a[i] * T + std::fabs(v0[i]) - std::sqrt(disc[i]));
    // Take care of the sign here
    if (h[i] < 0) v_cruise[i] *= -1;
  }
  return {true, v_cruise};
}