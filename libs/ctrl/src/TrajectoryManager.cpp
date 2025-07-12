#include <iostream>

#include "Utils.h"
#include "TrajectoryManager.h"

/*
  Given a path, it conditionally returns list of trajectories
  Condition is that each of the trajectories is feasible
*/
bool ctrl::TrajectoryManager::CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path) {
  if (path.size() <= 1) return false;
  Trajectories trajectories;
  double t_start_s = util::GetCurrentTime();
  double t_end_s = util::GetCurrentTime();
  for (int path_index = 1; path_index < path.size(); ++path_index) {
    Eigen::Vector3d pose_start = path[path_index - 1];
    Eigen::Vector3d pose_end = path[path_index];
    Eigen::Vector3d distance_m_rad = pose_end - pose_start;
    double total_time_s = 4.0;  // TODO: Should be dynamic
    if (!ctrl::Trajectory3D::IsFeasible(distance_m_rad, total_time_s).first) return false;
    // Create Trajectory
    t_start_s = t_end_s;
    t_end_s += total_time_s;
    trajectories.push(
        std::make_unique<ctrl::Trajectory3D>(pose_start, pose_end, t_start_s, t_end_s));
  }
  MergeNewTrajectories(std::move(trajectories));
  return true;
}

void ctrl::TrajectoryManager::MergeNewTrajectories(Trajectories&& new_trajectories) {
  if (new_trajectories.empty()) return;
  double new_traj_t_start_s = new_trajectories.front()->GetTStart();

  // Case 1a: First calls
  bool first_call = active_trajectories.empty();
  if (first_call) {
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
    // Step 1: Make a zero-velocity trajectory
    active_trajectories.push(
        std::make_unique<ctrl::Trajectory3D>(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0),
                                             active_traj_t_finish_s, new_traj_t_start_s));
    // Add all the the new trajectories to active trajectories
    while (!new_trajectories.empty()) {
      active_trajectories.push(std::move(new_trajectories.front()));
      new_trajectories.pop();
    }
    active_traj_t_finish_s = active_trajectories.back()->GetTFinish();
  }

  // Case 2a: New trajectories overlap with current trajectory
  // Case 2b: New trajectories overlap with active trajectory
  bool overlap = active_traj_t_finish_s > new_trajectories.front()->GetTFinish();
  if (overlap) {
    // Step 1: Merge happens at t_start of new trajectories
    double t_merge_s = new_traj_t_start_s;

    // Step 2: Find out the exact trajectory that needs to be merged
    // FindActiveTrajectoryAtT(t_merge_s);

    // Step 3: Find out where you need to change the sign of velocity
    // min_dist = force stop....
    // if required is smaller than min_dist --> then we change sign
    // active_merged_traj...current_pose <=====> new_traj pose_end

    // Step 4:

    // Step 5:

    // TODO: when you have to change the sign of velocity. HARD
  }
}

Eigen::Vector3d ctrl::TrajectoryManager::GetVelocityAtT(double current_time_s) {
  return current_trajectory->VelocityAtT(current_time_s);
}

void ctrl::TrajectoryManager::Print() {
  // std::cout << "[ctrl::TrajectoryManager::Print] Active Trajectories: \n";
  // for (int i = 0; i < active_trajectories.size(); ++i) {
  //   active_trajectories[i]->Print();
  // }
}

std::pair<bool, Eigen::Vector3d> ctrl::TrajectoryManager::Update() {
  // Step 1: Get current time
  double current_time_s = util::GetCurrentTime();

  // Step 2: Did active trajectories finish?. Motion is finished
  if (current_time_s >= active_traj_t_finish_s) {
    std::cout << "[ctrl::TrajectoryManager::Update] Finished motion. t_finish_s = "
              << active_traj_t_finish_s << std::endl;
    current_trajectory = nullptr;
    return std::make_pair(true, Eigen::Vector3d(0, 0, 0));  // finished motion
  }

  // CHECK: You are about to update current trajectory but there is no active trajectory left
  assert(!(current_trajectory == nullptr && active_trajectories.empty()));

  // Step 3: Update the current trajectory
  if (current_trajectory == nullptr || current_time_s >= current_trajectory->GetTFinish()) {
    current_trajectory = std::move(active_trajectories.front());
    active_trajectories.pop();
    current_trajectory->Print();
  }

  // Step 4: Get reference velocity from current trajectory
  return std::make_pair(false, GetVelocityAtT(current_time_s));
}