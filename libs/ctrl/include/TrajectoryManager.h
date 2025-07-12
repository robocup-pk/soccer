#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <queue>

#include <Eigen/Dense>

#include "Trajectory3D.h"

namespace ctrl {

typedef std::queue<std::unique_ptr<ctrl::Trajectory3D>> Trajectories;

class TrajectoryManager {
 public:
  bool CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path);
  void MergeNewTrajectories(Trajectories&& new_trajectories);
  Eigen::Vector3d GetVelocityAtT(double time_s);
  std::pair<bool, Eigen::Vector3d> Update();
  void SetActiveTrajectories(Trajectories&& new_trajectories);

  // Helpers
  void Print();

  Trajectories active_trajectories;

 private:
  double active_traj_t_finish_s;
  std::unique_ptr<ctrl::Trajectory3D> current_trajectory;
};

}  // namespace ctrl

#endif  // TRAJECTORY_MANAGER_H