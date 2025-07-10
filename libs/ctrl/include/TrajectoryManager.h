#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <Eigen/Dense>

#include "Trajectory3D.h"

namespace ctrl {

typedef std::vector<std::unique_ptr<ctrl::Trajectory3D>> Trajectories;

class TrajectoryManager {
 public:
  bool CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path);
  void MergeNewTrajectories(Trajectories&& new_trajectories);
  Eigen::Vector3d GetVelocityAtT(double time_s);

  inline double GetTFinish() { return t_finish_s; };

  bool Update();

  // Helpers
  void Print();

  Trajectories active_trajectories;

 private:
  double t_finish_s;
  std::unique_ptr<ctrl::Trajectory3D> current_trajectory;
};

}  // namespace ctrl

#endif  // TRAJECTORY_MANAGER_H