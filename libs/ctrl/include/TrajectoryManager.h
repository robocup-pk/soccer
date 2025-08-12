#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <queue>
#include <mutex>
#include <Eigen/Dense>

#include "Spline2D.h"
#include "Heading1D.h"
#include "TrapezoidalTrajectoryVi3D.h"
#include "Utils.h"


namespace ctrl {

class TrajectoryManager {
 public:
  bool CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path,
                                  double t_start_s = util::GetCurrentTime());
  Eigen::Vector3d GetVelocityAtT(double time_s);
  std::pair<bool, Eigen::Vector3d> Update(Eigen::Vector3d pose_est);

  // Merge Logic

  Eigen::Vector3d FindV0AtT(double t);
  // Helpers
  void Print();

  double active_traj_t_finish_s;

 private:
  std::mutex active_traj_mutex;
  std::unique_ptr<Spline2D> spline_traj;
  std::unique_ptr<Heading1D> heading_traj;
};

}  // namespace ctrl

#endif  // TRAJECTORY_MANAGER_H