#include <iostream>

#include "TrajectoryManager.h"

void ctrl::TrajectoryManager::MergeNewTrajectories(Trajectories&& new_trajectories) {
  if (new_trajectories.empty()) return;
  double new_traj_t_start_s = new_trajectories.front()->GetTStart();

  // Case 1a: First call (and its analogous cases)
  if (active_trajectories.empty()) {
    MergeNewTrajectoriesFirstCall(std::move(new_trajectories));
    return;
  }

  // Case 1b: New trajectories arrive in future (no overlap with active trajectories)
  if (new_traj_t_start_s >= active_traj_t_finish_s) {
    MergeNewTrajectoriesInFuture(std::move(new_trajectories));
    return;
  }

  // Case 2a: New trajectories overlap with active trajectory
  if (active_traj_t_finish_s > new_trajectories.front()->GetTStart()) {
    MergeNewTrajectoriesAtT(std::move(new_trajectories));
  }

  // Case 2b: New trajectories overlap with current trajectory
}

void ctrl::TrajectoryManager::MergeNewTrajectoriesAtT(Trajectories&& new_trajectories) {
  std::unique_ptr<ctrl::Trajectory3D> new_trajectory_merge = std::move(new_trajectories.front());
  new_trajectories.pop();

  // Find T where merge should happen
  Eigen::Vector3d v0(0, 0, 0);
  double new_traj_t_start_s = new_trajectory_merge->GetTStart();

  // Make a copy of the active trajectories queue
  std::queue<std::unique_ptr<Trajectory3D>> active_trajectories_cpy;
  std::unique_lock<std::mutex> lock(active_traj_mutex);
  while (!active_trajectories.empty()) {
    // Find the overlapping trajectory
    if (active_trajectories.front()->GetTStart() < new_traj_t_start_s &&
        active_trajectories.front()->GetTFinish() > new_traj_t_start_s) {
      // This is the overlapping trajectory that we need. Find v0
      v0 = active_trajectories.front()->VelocityAtT(new_traj_t_start_s);
      // Trim the active trajectory at T and add it to the active trajectories
      active_trajectories_cpy.push(std::move(active_trajectories.front()));
      active_trajectories_cpy.back()->SetTFinish(new_traj_t_start_s);
      active_trajectories.pop();
      break;  // Get out. Then we'll add the new trajectories from T
    }
    // Keep adding the previous active trajectories
    active_trajectories_cpy.push(std::move(active_trajectories.front()));
    active_trajectories.pop();
  }

  while (!active_trajectories.empty()) active_trajectories.pop();

  // Create a trajectory from T that starts with v0
  Eigen::Vector3d pose_start = new_trajectory_merge->GetPoseStart();
  Eigen::Vector3d pose_end = new_trajectory_merge->GetPoseEnd();
  double t_start_s = new_trajectory_merge->GetTStart();
  double t_finish_s = new_trajectory_merge->GetTFinish();
  std::unique_ptr<ctrl::TrapezoidalTrajectoryVi3D> merged_traj =
      std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(pose_start, pose_end, t_start_s,
                                                        t_finish_s, v0);
  if (merged_traj) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryAtT] Merged Trajectory: \n";
    merged_traj->Print();
  }
  active_trajectories_cpy.push(std::move(merged_traj));

  // Update the active trajectories
  while (!active_trajectories_cpy.empty()) {
    auto& traj = active_trajectories_cpy.front();
    traj->Print();
    active_trajectories.push(std::move(traj));
    active_trajectories_cpy.pop();
  }

  // Add all the remaining new trajectories to active trajectories
  while (!new_trajectories.empty()) {
    active_trajectories.push(std::move(new_trajectories.front()));
    new_trajectories.pop();
  }
  active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
}

void ctrl::TrajectoryManager::MergeNewTrajectoriesInFuture(Trajectories&& new_trajectories) {
  double t = new_trajectories.front()->GetTStart();
  std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryInFuture] t = " << t << std::endl;
  if (t - active_traj_t_finish_s > 0.01) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryInFuture] Zero-vel traj from t "
              << active_traj_t_finish_s << " " << t << std::endl;
    // Step 1: Make a zero-velocity trajectory
    std::unique_lock<std::mutex> lock(active_traj_mutex);
    active_trajectories.push(std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), active_traj_t_finish_s, t));
  }

  // Step 2: Add all the new trajectories into active trajectories
  MergeNewTrajectoriesFirstCall(std::move(new_trajectories));
}

void ctrl::TrajectoryManager::MergeNewTrajectoriesFirstCall(Trajectories&& new_trajectories) {
  std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoriesFirstCall]" << std::endl;
  // Add all the the new trajectories into active trajectories
  std::unique_lock<std::mutex> lock(active_traj_mutex);
  while (!new_trajectories.empty()) {
    active_trajectories.push(std::move(new_trajectories.front()));
    new_trajectories.pop();
  }
  active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
}

Eigen::Vector3d ctrl::TrajectoryManager::FindV0AtT(double t) {
  if (active_traj_t_finish_s <= t) return Eigen::Vector3d(0, 0, 0);
  std::queue<std::unique_ptr<Trajectory3D>> active_trajectories_cpy;
  Eigen::Vector3d v0(0, 0, 0);
  std::unique_lock<std::mutex> lock(active_traj_mutex);
  while (!active_trajectories.empty()) {
    // Find the overlapping trajectory
    if (active_trajectories.front()->GetTStart() < t &&
        active_trajectories.front()->GetTFinish() > t) {
      v0 = active_trajectories.front()->VelocityAtT(t);
    }
    // Keep adding the previous active trajectories
    active_trajectories_cpy.push(std::move(active_trajectories.front()));
    active_trajectories.pop();
  }
  // Add back
  while (!active_trajectories_cpy.empty()) {
    active_trajectories.push(std::move(active_trajectories_cpy.front()));
    active_trajectories_cpy.pop();
  }
  active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
  return v0;
}