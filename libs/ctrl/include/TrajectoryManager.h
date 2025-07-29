#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <queue>
#include <mutex>
#include <Eigen/Dense>

#include "Trajectory3D.h"
class Trajectory3D;
#include "Utils.h"

namespace ctrl {

typedef std::queue<std::unique_ptr<ctrl::Trajectory3D>> Trajectories;

class TrajectoryManager {
 public:
  bool CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path,
                                  double t_start_s = util::GetCurrentTime());
  void MergeNewTrajectories(Trajectories&& new_trajectories);
  Eigen::Vector3d GetVelocityAtT(double time_s);
  std::pair<bool, Eigen::Vector3d> Update(Eigen::Vector3d pose_est);
  Eigen::Vector3d GetPositionAtT(double time_s);
  // Merge Logic

  Eigen::Vector3d FindV0AtT(double t);
  void MergeNewTrajectoriesFirstCall(Trajectories&& new_trajectories);
  void MergeNewTrajectoriesInFuture(Trajectories&& new_trajectories);
  void MergeNewTrajectoriesAtT(Trajectories&& new_trajectories);
  void MergeNewTrajectoriesAtCurrentTime(Trajectories&& new_trajectories);
  void UpdateCurrentTrajectory();

  // Helpers
  void Print();

  Trajectories active_trajectories;

  double active_traj_t_finish_s;

 private:
  Eigen::Vector3d p_fworld;
  std::unique_ptr<ctrl::Trajectory3D> current_trajectory;
  std::mutex active_traj_mutex;
};

}  // namespace ctrl

#endif  // TRAJECTORY_MANAGER_H