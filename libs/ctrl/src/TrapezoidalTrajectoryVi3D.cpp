#include <iostream>

#include "SystemConfig.h"

#include "TrapezoidalTrajectoryVi3D.h"

ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D(
    std::unique_ptr<ctrl::Trajectory3D> traj, Eigen::Vector3d v0) {
  this->pose_start = traj->GetPoseStart();
  this->pose_end = traj->GetPoseEnd();
  this->t_start_s = traj->GetTStart();
  this->t_finish_s = traj->GetTFinish();
  std::cout << "\n\n\nMerge Traj\n";
  traj->Print();
  this->v0 = v0;

  distance_m_rad = pose_end - pose_start;
  total_time_s = t_finish_s - t_start_s;

  std::pair<bool, std::optional<Eigen::Vector3d>> traj_v_constant =
      IsFeasibleNonZeroVelocity(distance_m_rad, total_time_s, v0);

  if (!traj_v_constant.first) {
    std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] Trajectory not "
                 "feasible. Try different params"
              << std::endl;
    return;
  }

  v_max = traj_v_constant.second.value();

  for (int i = 0; i < 3; ++i) {
    t_acc_s[i] = (v_max[i] - v0[i]) / cfg::SystemConfig::max_acc_m_radpsps[i];
    t_dec_s[i] = v_max[i] / cfg::SystemConfig::max_acc_m_radpsps[i];
  }
  std::cout << "v_max: " << v_max.transpose() << std::endl;
  std::cout << "merged t_acc: " << t_acc_s.transpose() << std::endl;
  std::cout << "merged t_dec: " << t_dec_s.transpose() << std::endl;
}

ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D(Eigen::Vector3d pose_start,
                                                           Eigen::Vector3d pose_end,
                                                           double t_start_s, double t_end_s,
                                                           Eigen::Vector3d v0) {
  this->pose_start = pose_start;
  this->pose_end = pose_end;
  this->t_start_s = t_start_s;
  this->t_finish_s = t_end_s;
  this->v0 = v0;

  distance_m_rad = pose_end - pose_start;
  total_time_s = t_finish_s - t_start_s;

  std::pair<bool, std::optional<Eigen::Vector3d>> traj_v_constant =
      IsFeasibleNonZeroVelocity(distance_m_rad, total_time_s, v0);

  if (!traj_v_constant.first) {
    std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] Trajectory not "
                 "feasible. Try different params"
              << std::endl;
    return;
  }

  v_max = traj_v_constant.second.value();

  for (int i = 0; i < 3; ++i) {
    t_acc_s[i] = (v_max[i] - v0[i]) / cfg::SystemConfig::max_acc_m_radpsps[i];
    t_dec_s[i] = v_max[i] / cfg::SystemConfig::max_acc_m_radpsps[i];
  }
}

Eigen::Vector3d ctrl::TrapezoidalTrajectoryVi3D::VelocityAtT(double t_sec) {
  // t_sec is global
  if (t_sec < t_start_s || t_sec > t_finish_s) {
    return Eigen::Vector3d(0, 0, 0);
  }

  t_sec -= t_start_s;
  // t_sec is from 0 to total_time_s
  Eigen::Vector3d velocity_fBody_mps;
  for (int i = 0; i < 3; ++i) {
    double sign = (v_max[i] >= 0.0) ? 1.0 : -1.0;
    if (t_sec < std::abs(t_acc_s[i])) {
      if (t_acc_s[i] < 0) sign *= -1;
      velocity_fBody_mps[i] = v0[i] + sign * cfg::SystemConfig::max_acc_m_radpsps[i] * t_sec;
    } else if (t_sec < (total_time_s - t_dec_s[i])) {
      velocity_fBody_mps[i] = v_max[i];
    } else {
      velocity_fBody_mps[i] = v_max[i] - sign * cfg::SystemConfig::max_acc_m_radpsps[i] *
                                             (t_sec - total_time_s + t_dec_s[i]);
    }
  }

  std::cout << "v_m = " << velocity_fBody_mps.transpose() << std::endl;
  return velocity_fBody_mps;
}

void ctrl::TrapezoidalTrajectoryVi3D::Print() {
  std::cout << "[ctrl::TrapezoidalTrajectory3D::Print] Trajectory Info. ";
  std::cout << "Pose: " << pose_start.transpose() << " -> " << pose_end.transpose();
  std::cout << ". Time: " << t_start_s << " -> " << t_finish_s;
  std::cout << ". v0: " << v0.transpose() << "\n\n";
}

Eigen::Vector3d ctrl::TrapezoidalTrajectoryVi3D::TotalDistance() {
  Eigen::Vector3d distance(0, 0, 0);
  for (double t = t_start_s; t < t_finish_s; t += 0.01) {
    distance += VelocityAtT(t) * 0.01;
  }
  return distance;
}