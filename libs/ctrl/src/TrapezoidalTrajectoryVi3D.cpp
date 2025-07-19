#include <iostream>

#include "SystemConfig.h"

#include "TrapezoidalTrajectoryVi3D.h"
#include "Utils.h"

ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D(Eigen::Vector3d pose_start,
                                                           Eigen::Vector3d pose_end,
                                                           double t_start_s, double t_end_s,
                                                           Eigen::Vector3d v0) {
  this->pose_start = pose_start;
  this->pose_end = pose_end;
  this->t_start_s = t_start_s;
  this->t_finish_s = t_end_s;
  this->v0 = v0;
  h = pose_end - pose_start;
  T = t_finish_s - t_start_s;
  a = cfg::SystemConfig::max_acc_m_radpsps;

  std::pair<bool, std::optional<Eigen::Vector3d>> traj_v_cruise = IsFeasible(h, T, v0);

  if (!traj_v_cruise.first) {
    std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] Trajectory not "
                 "feasible. Try different params"
              << std::endl;
    return;
  }

  v_cruise = traj_v_cruise.second.value();
  for (int i = 0; i < 3; ++i) {
    if (v0[i] * h[i] >= 0) {
      // Case 1: No sign change
      t_a[i] = std::fabs(v_cruise[i] - v0[i]) / a[i];
      t_d[i] = std::fabs(v_cruise[i]) / a[i];
    } else {
      // Case 2: Sign change
      t_1 = std::fabs(v0[i]) / a[i];
      t_2 = T - t_1;
      t_a[i] = std::fabs(v_cruise[i]) / a[i];
      t_d[i] = t_a[i];
    }
  }

  std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] v_cruise: "
            << v_cruise.transpose() << std::endl;
  std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] t_a: "
            << t_a.transpose() << std::endl;
  std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] t_d: "
            << t_d.transpose() << std::endl;
  std::cout << "[ctrl::TrapezoidalTrajectoryVi3D::TrapezoidalTrajectoryVi3D] T: " << T
            << std::endl;
}

Eigen::Vector3d ctrl::TrapezoidalTrajectoryVi3D::VelocityAtT(double t_sec) {
  // t_sec is global
  if (t_sec < t_start_s || t_sec > t_finish_s) {
    return Eigen::Vector3d(0, 0, 0);
  }
  t_sec -= t_start_s;
  Eigen::Vector3d a = cfg::SystemConfig::max_acc_m_radpsps;
  // t_sec is from 0 to total_time_s
  Eigen::Vector3d v_fWorld;
  for (int i = 0; i < 3; ++i) {
    if (v0[i] * h[i] >= 0) {
      // Case 1: No sign change
      int sign = v_cruise[i] >= v0[i] ? 1 : -1;
      if (t_sec < t_a[i]) {
        v_fWorld[i] = v0[i] + sign * a[i] * t_sec;
      } else if (t_sec < (T - t_d[i])) {
        v_fWorld[i] = v_cruise[i];
      } else {
        int sign = 0 >= v_cruise[i] ? 1 : -1;
        double t_dec_s = t_sec - (T - t_d[i]);
        v_fWorld[i] = v_cruise[i] + sign * a[i] * t_dec_s;
      }
    } else {
      // Case 2: Sign change
      int sign = v0[i] > 0 ? -1 : 1;
      if (t_sec < t_1) {
        v_fWorld[i] = v0[0] + sign * a[i] * t_sec;
      } else {
        // Now we are in the trapezoidal part
        t_sec -= t_1;
        if (t_sec < t_a[i]) {
          v_fWorld[i] = sign * a[i] * t_sec;
        } else if (t_sec < t_2 - t_d[i]) {
          v_fWorld[i] = v_cruise[i];
        } else {
          double t_dec_s = t_sec - (t_2 - t_d[i]);
          v_fWorld[i] = v_cruise[i] - sign * a[i] * t_dec_s;
        }
      }
    }
  }

  return v_fWorld;
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