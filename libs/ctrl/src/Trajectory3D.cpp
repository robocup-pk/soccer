#include <iostream>

#include "SystemConfig.h"

#include "Trajectory3D.h"

ctrl::Trajectory3D::Trajectory3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end,
                                 double t_start_s, double t_end_s) {
  this->pose_start = pose_start;
  this->pose_end = pose_end;
  this->t_start_s = t_start_s;
  this->t_finish_s = t_end_s;

  distance_m_rad = pose_end - pose_start;
  total_time_s = t_finish_s - t_start_s;

  std::pair<bool, std::optional<Eigen::Vector3d>> traj_t_acc =
      IsFeasible(distance_m_rad, total_time_s);

  if (!traj_t_acc.first) {
    std::cout << "[ctrl::Trajectory3D::Trajectory3D] Trajectory not feasible. Try different params"
              << std::endl;
    return;
  }

  t_acc_s = traj_t_acc.second.value();
  v_max = cfg::SystemConfig::max_acceleration_mpsps_radpsps.cwiseProduct(t_acc_s).cwiseProduct(
      distance_m_rad.cwiseSign());
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
    double disc = total_time_s * total_time_s -
                  4 * abs_dist / cfg::SystemConfig::max_acceleration_mpsps_radpsps[i];

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
    if (cfg::SystemConfig::max_acceleration_mpsps_radpsps[i] * t_acc[i] >
        cfg::SystemConfig::max_velocity_fBody_mps[i]) {
      return {false, std::nullopt};
    }
  }

  return {true, t_acc};
}

Eigen::Vector3d ctrl::Trajectory3D::VelocityAtT(double t_sec) {
  std::cout << "[ctrl::Trajectory3D::VelocityAtT] t_sec: " << t_sec << ". t_start: " << t_start_s
            << ". t_finish: " << t_finish_s << ". total_time_s: " << total_time_s
            << ". t_acc: " << t_acc_s.transpose() << std::endl;
  if (t_sec < t_start_s || t_sec > t_finish_s) {
    return Eigen::Vector3d(0, 0, 0);
  }

  t_sec -= t_start_s;

  Eigen::Vector3d velocity_fBody_mps;
  for (int i = 0; i < 3; ++i) {
    double sign = (v_max[i] >= 0.0) ? 1.0 : -1.0;
    if (t_sec < t_acc_s[i]) {
      velocity_fBody_mps[i] = sign * cfg::SystemConfig::max_acceleration_mpsps_radpsps[i] * t_sec;
      std::cout << "[ctrl::Trajectory3D::VelocityAtT] i = " << i
                << ". v = " << velocity_fBody_mps[i] << std::endl;
    } else if (t_sec < (total_time_s - t_acc_s[i])) {
      velocity_fBody_mps[i] = v_max[i];
    } else {
      velocity_fBody_mps[i] = v_max[i] - sign *
                                             cfg::SystemConfig::max_acceleration_mpsps_radpsps[i] *
                                             (t_sec - total_time_s + t_acc_s[i]);
    }
  }

  return velocity_fBody_mps;
}

void ctrl::Trajectory3D::Print() {
  std::cout << "Trajectory Info\n";
  std::cout << "Pose: " << pose_start.transpose() << " -> " << pose_end.transpose() << std::endl;
  std::cout << "v_max: " << v_max.transpose() << std::endl;
  std::cout << "time: " << t_start_s << " -> " << t_finish_s << "\n\n";
}

Eigen::Vector3d ctrl::Trajectory3D::TotalDistance() {
  Eigen::Vector3d distance(0, 0, 0);
  for (double t = t_start_s; t < t_finish_s; t += 0.01) {
    distance += VelocityAtT(t) * 0.01;
  }
  return distance;
}