#ifndef TRAPEZOIDAL_TRAJECTORY_VI_3D_H
#define TRAPEZOIDAL_TRAJECTORY_VI_3D_H

#include <optional>

#include <Eigen/Dense>

#include "Trajectory3D.h"
#include "TrapezoidalTrajectory3D.h"

namespace ctrl {

class TrapezoidalTrajectoryVi3D : public Trajectory3D {
 public:
  TrapezoidalTrajectoryVi3D(std::unique_ptr<ctrl::Trajectory3D> traj, Eigen::Vector3d v0);

  TrapezoidalTrajectoryVi3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end, double t_start_s,
                            double t_end_s, Eigen::Vector3d v0);

  Eigen::Vector3d VelocityAtT(double t) override;

  // Helpers
  void Print() override;
  Eigen::Vector3d TotalDistance() override;

  void SetTFinish(double t_finish_s) override { this->t_finish_s = t_finish_s; }
  double GetTStart() override { return t_start_s; }
  double GetTFinish() override { return t_finish_s; }
  Eigen::Vector3d GetPoseStart() override { return pose_start; }
  Eigen::Vector3d GetPoseEnd() override { return pose_end; }

 private:
  Eigen::Vector3d pose_start;
  Eigen::Vector3d pose_end;
  double t_start_s;
  double t_finish_s;

  double total_time_s;
  Eigen::Vector3d distance_m_rad;

  // Trajectory limits
  Eigen::Vector3d t_acc_s;
  Eigen::Vector3d t_dec_s;
  Eigen::Vector3d v_max;
  Eigen::Vector3d v0;
};

}  // namespace ctrl

#endif  // TRAPEZOIDAL_TRAJECTORY_VI_3D_H