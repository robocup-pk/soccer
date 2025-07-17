#ifndef TRAPEZOIDAL_TRAJECTORY_VI_3D_H
#define TRAPEZOIDAL_TRAJECTORY_VI_3D_H

#include <optional>

#include <Eigen/Dense>

#include "Trajectory3D.h"

namespace ctrl {

class TrapezoidalTrajectoryVi3D : public Trajectory3D {
 public:
  TrapezoidalTrajectoryVi3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end, double t_start_s,
                            double t_end_s, Eigen::Vector3d v0 = Eigen::Vector3d(0, 0, 0));

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

  Eigen::Vector3d total_time_1;  // TODO: make it better. just trying to make it work right now
  double T;
  Eigen::Vector3d h;

  // Trajectory limits
  Eigen::Vector3d a;
  double t_1;
  double t_2;
  Eigen::Vector3d t_a;
  Eigen::Vector3d t_d;
  Eigen::Vector3d v_cruise;
  Eigen::Vector3d v0;
};

}  // namespace ctrl

#endif  // TRAPEZOIDAL_TRAJECTORY_VI_3D_H