#include <iostream>

#include "TrajectoryManager.h"

void ctrl::TrajectoryManager::MergeNewTrajectories(Trajectories&& new_trajectories) {
  if (new_trajectories.empty()) return;
  double new_traj_t_start_s = new_trajectories.front()->GetTStart();

  // Case 1a: First calls
  bool first_call = active_trajectories.empty();
  if (first_call) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectories] First Call" << std::endl;
    // Add all the new trajectories
    while (!new_trajectories.empty()) {
      active_trajectories.push(std::move(new_trajectories.front()));
      new_trajectories.pop();
    }
    active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
    return;
  }
  // Case 1b: New trajectories arrive in future (no overlap with active trajectories)
  bool new_traj_in_future = new_traj_t_start_s >= active_traj_t_finish_s;
  if (new_traj_in_future) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectories] Future" << std::endl;
    // Step 1: Make a zero-velocity trajectory
    active_trajectories.push(std::make_unique<ctrl::TrapezoidalTrajectory3D>(
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), active_traj_t_finish_s,
        new_traj_t_start_s));
    // Add all the the new trajectories to active trajectories
    while (!new_trajectories.empty()) {
      active_trajectories.push(std::move(new_trajectories.front()));
      new_trajectories.pop();
    }
    active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
  }

  // Case 2a: New trajectories overlap with current trajectory
  // Case 2b: New trajectories overlap with active trajectory
  bool overlap = active_traj_t_finish_s > new_trajectories.front()->GetTStart();
  if (overlap) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectories] Overlap" << std::endl;
    // Step 1: Merge the overlapping trajectory with the required active trajectory
    MergeNewTrajectoryAtT(std::move(new_trajectories.front()));
    new_trajectories.pop();

    // Step 2: Add all the remaining new trajectories
    while (!new_trajectories.empty()) {
      active_trajectories.push(std::move(new_trajectories.front()));
      new_trajectories.pop();
    }
    active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
  }
}

void ctrl::TrajectoryManager::MergeNewTrajectoryAtT(
    std::unique_ptr<ctrl::Trajectory3D> new_trajectory_merge) {
  std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryAtT] Overlapping new trajectory: \n";
  new_trajectory_merge->Print();
  Eigen::Vector3d v0(0, 0, 0);
  double new_traj_t_start_s = new_trajectory_merge->GetTStart();

  // Make a copy of the active trajectories queue
  std::queue<std::unique_ptr<Trajectory3D>> active_trajectories_cpy;
  while (!active_trajectories.empty()) {
    // Find the overlapping trajectory
    if (active_trajectories.front()->GetTStart() < new_traj_t_start_s &&
        active_trajectories.front()->GetTFinish() > new_traj_t_start_s) {
      // This is the overlapping trajectory that we need. Find v0
      v0 = active_trajectories.front()->VelocityAtT(new_traj_t_start_s);
      std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryAtT] Overlapping active "
                   "trajectory:\n";
      active_trajectories.front()->Print();
      std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryAtT] v0: " << v0.transpose()
                << std::endl;
      active_trajectories_cpy.push(std::move(active_trajectories.front()));
      active_trajectories_cpy.back()->SetTFinish(new_traj_t_start_s);
      active_trajectories.pop();
      break;
    }
    active_trajectories_cpy.push(std::move(active_trajectories.front()));
    active_trajectories.pop();
  }

  // Create a trajectory from t_start_new that starts with v0
  while (!active_trajectories.empty()) active_trajectories.pop();

  std::unique_ptr<ctrl::TrapezoidalTrajectoryVi3D> merged_traj =
      std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(std::move(new_trajectory_merge), v0);
  if (merged_traj) {
    std::cout << "[ctrl::TrajectoryManager::MergeNewTrajectoryAtT] Merged Trajectory: \n";
    merged_traj->Print();
  }
  active_trajectories_cpy.push(std::move(merged_traj));

  // Update the active trajectories
  std::cout << "\n\nAll Trajectories: \n";
  while (!active_trajectories_cpy.empty()) {
    auto& traj = active_trajectories_cpy.front();
    traj->Print();
    active_trajectories.push(std::move(traj));
    active_trajectories_cpy.pop();
  }
}