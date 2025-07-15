#include <iostream>

#include "SystemConfig.h"
#include "Trajectory3D.h"

std::pair<bool, std::optional<Eigen::Vector3d>> ctrl::Trajectory3D::IsFeasibleNonZeroVelocity(
    Eigen::Vector3d distance_m_rad, double total_time_s, Eigen::Vector3d v0) {
  std::cout << "[ctrl::Trajectory3D::IsFeasibleNonZeroVelocity] distance: " << distance_m_rad.transpose() << ". v0: " << v0.transpose() << ". total_time_s: " << total_time_s << std::endl;
  // Condition 1: a * h < v^2 / 2
  for (int i = 0; i < 3; ++i) {
    std::cout << "A\n"; 
    if (cfg::SystemConfig::max_acc_m_radpsps[i] * std::fabs(distance_m_rad[i]) < v0[i] * v0[i] / 2)
      return {false, std::nullopt};
  }

  // Condition 2: a < a_max
  // for (int i = 0; i < 3; ++i) {
  //   std::cout << "B\n";
  //   double a =
  //       2 * distance_m_rad[i] - total_time_s * v0[i] +
  //       std::sqrt(4 * std::pow(distance_m_rad[i], 2) - 4 * distance_m_rad[i] * v0[i] * total_time_s +
  //                 2 * std::pow(v0[i], 2) * std::pow(total_time_s, 2));
  //   if (a > cfg::SystemConfig::max_acc_m_radpsps[i]) return {false, std::nullopt};
  // }

  // Condition 3: disciminant
  Eigen::Vector3d disc;
  for (int i = 0; i < 3; ++i) {
    std::cout << "C\n";
    disc[i] = std::pow(cfg::SystemConfig::max_acc_m_radpsps[i], 2) * std::pow(total_time_s, 2) -
              4 * cfg::SystemConfig::max_acc_m_radpsps[i] * std::fabs(distance_m_rad[i]) +
              2 * cfg::SystemConfig::max_acc_m_radpsps[i] * v0[i] * total_time_s - v0[i] * v0[i];
    if (disc[i] <= 0) return {false, std::nullopt};
  }

  // It is feasible. Calculate the parameters
  Eigen::Vector3d v_constant;
  for (int i = 0; i < 3; ++i) {
    v_constant[i] =
        0.5 * (v0[i] + cfg::SystemConfig::max_acc_m_radpsps[i] * total_time_s - std::sqrt(disc[i]));
  }

  return {true, v_constant};
}

std::pair<bool, std::optional<Eigen::Vector3d>> ctrl::Trajectory3D::IsFeasible(
    Eigen::Vector3d distance_m_rad, double total_time_s) {
  Eigen::Vector3d t_acc(-1, -1, -1);

  for (int i = 0; i < 3; ++i) {
    if (std::abs(distance_m_rad[i]) < 1e-6) {
      t_acc[i] = 0.0;  // no motion needed
      continue;
    }
    double abs_dist = std::abs(distance_m_rad[i]);
    double disc =
        total_time_s * total_time_s - 4 * abs_dist / cfg::SystemConfig::max_acc_m_radpsps[i];

    if (disc < 0) return {false, std::nullopt};
    double sqrt_disc = std::sqrt(disc);
    double t_acc_1 = (total_time_s - sqrt_disc) / 2.0;
    double t_acc_2 = (total_time_s + sqrt_disc) / 2.0;

    if (t_acc_1 > 0 && t_acc_1 < total_time_s) {
      t_acc[i] = t_acc_1;
    } else if (t_acc_2 > 0 && t_acc_2 < total_time_s) {
      t_acc[i] = t_acc_2;
    }
    if (t_acc[i] < 0) return {false, std::nullopt};

    // t_acc is fine. check v_limit = a * t
    if (cfg::SystemConfig::max_acc_m_radpsps[i] * t_acc[i] >
        cfg::SystemConfig::max_velocity_fBody_mps[i]) {
      return {false, std::nullopt};
    }
  }

  return {true, t_acc};
}