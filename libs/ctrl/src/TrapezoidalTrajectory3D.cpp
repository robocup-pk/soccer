#include <iostream>

#include "SystemConfig.h"

#include "TrapezoidalTrajectory3D.h"

ctrl::TrapezoidalTrajectory3D::TrapezoidalTrajectory3D(Eigen::Vector3d pose_start,
                                                       Eigen::Vector3d pose_end, double t_start_s,
                                                       double t_end_s) {
  this->pose_start = pose_start;
  this->pose_end = pose_end;
  this->t_start_s = t_start_s;
  this->t_finish_s = t_end_s;

  distance_m_rad = pose_end - pose_start;
  total_time_s = t_finish_s - t_start_s;

  std::pair<bool, std::optional<Eigen::Vector3d>> traj_t_acc =
      IsFeasible(distance_m_rad, total_time_s);

  if (!traj_t_acc.first) {
    std::cout << "[ctrl::TrapezoidalTrajectory3D::TrapezoidalTrajectory3D] Trajectory not "
                 "feasible. Try different params"
              << std::endl;
    return;
  }

  t_acc_s = traj_t_acc.second.value();
  v_max = cfg::SystemConfig::max_acc_m_radpsps.cwiseProduct(t_acc_s).cwiseProduct(
      distance_m_rad.cwiseSign());
}

Eigen::Vector3d ctrl::TrapezoidalTrajectory3D::VelocityAtT(double t_sec) {
  // t_sec is global
  if (t_sec < t_start_s || t_sec > t_finish_s) {
    return Eigen::Vector3d(0, 0, 0);
  }

  t_sec -= t_start_s;
  // t_sec is from 0 to total_time_s
  Eigen::Vector3d velocity_fBody_mps;
  for (int i = 0; i < 3; ++i) {
    double sign = (v_max[i] >= 0.0) ? 1.0 : -1.0;
    if (t_sec < t_acc_s[i]) {
      velocity_fBody_mps[i] = sign * cfg::SystemConfig::max_acc_m_radpsps[i] * t_sec;
    } else if (t_sec < (total_time_s - t_acc_s[i])) {
      velocity_fBody_mps[i] = v_max[i];
    } else {
      velocity_fBody_mps[i] = v_max[i] - sign * cfg::SystemConfig::max_acc_m_radpsps[i] *
                                             (t_sec - total_time_s + t_acc_s[i]);
    }
  }
  std::cout << "v_a = " << velocity_fBody_mps.transpose() << std::endl;
  return velocity_fBody_mps;
}

void ctrl::TrapezoidalTrajectory3D::Print() {
  std::cout << "[ctrl::TrapezoidalTrajectory3D::Print] Trajectory Info. ";
  std::cout << "Pose: " << pose_start.transpose() << " -> " << pose_end.transpose();
  std::cout << ". Time: " << t_start_s << " -> " << t_finish_s << "\n";
}

Eigen::Vector3d ctrl::TrapezoidalTrajectory3D::TotalDistance() {
  Eigen::Vector3d distance(0, 0, 0);
  for (double t = t_start_s; t < t_finish_s; t += 0.01) {
    distance += VelocityAtT(t) * 0.01;
  }
  return distance;
}