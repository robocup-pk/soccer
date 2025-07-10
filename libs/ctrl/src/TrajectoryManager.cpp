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
  double t_start_s = 0;
  double t_end_s = 0;
  t_finish_s = 0;
  for (int path_index = 1; path_index < path.size(); ++path_index) {
    Eigen::Vector3d pose_start = path[path_index - 1];
    Eigen::Vector3d pose_end = path[path_index];
    Eigen::Vector3d distance_m_rad = pose_end - pose_start;
    double total_time_s = 4.0;  // TODO: Should be dynamic
    if (!ctrl::Trajectory3D::IsFeasible(distance_m_rad, total_time_s).first) return false;
    // Create Trajectory
    t_start_s = t_end_s;
    t_end_s += total_time_s;
    trajectories.push_back(
        std::make_unique<ctrl::Trajectory3D>(pose_start, pose_end, t_start_s, t_end_s));
  }
  t_finish_s = t_end_s;  // TODO: improve
  MergeNewTrajectories(std::move(trajectories));
  return true;
}

void ctrl::TrajectoryManager::MergeNewTrajectories(Trajectories&& new_trajectories) {
  // First call
  if (active_trajectories.empty()) {
    for (auto& traj : new_trajectories) {
      active_trajectories.push_back(std::move(traj));
    }
  }

  // Merge active_trajectories and new_trajectires IF POSSIBLE
  // TODO:
}

Eigen::Vector3d ctrl::TrajectoryManager::GetVelocityAtT(double time_s) {
  // TODO: remove this
  static double start_time = util::GetCurrentTime();
  double current_time = util::GetCurrentTime();
  double t = (current_time - start_time);// - current_trajectory->GetTStart();
  if (t > t_finish_s) {
    std::cout << "[ctrl::TrajectoryManager::GetVelocityAtT] Finished active trajectories. t = "
              << t << std::endl;
    return Eigen::Vector3d::Zero();
  }

  return current_trajectory->VelocityAtT(t);
}

void ctrl::TrajectoryManager::Print() {
  std::cout << "[ctrl::TrajectoryManager::Print] Active Trajectories: \n";
  for (int i = 0; i < active_trajectories.size(); ++i) {
    active_trajectories[i]->Print();
  }
}

bool ctrl::TrajectoryManager::Update() {
  static double start_time = util::GetCurrentTime();
  double current_time = util::GetCurrentTime();
  double t_s = current_time - start_time;
  std::cout << "\n[ctrl::TrajectoryManager::Update] t = " << t_s << std::endl;

  if (t_s > t_finish_s) {
    std::cout << "[ctrl::TrajectoryManager::Update] Finished motion. t_finish_s = " << t_finish_s
              << std::endl;
    current_trajectory = nullptr;
    return true;  // finished motion
  }

  // Update the current trajectory
  for (int i = 0; i < active_trajectories.size(); ++i) {
    if (t_s >= active_trajectories[i]->GetTStart() && t_s < active_trajectories[i]->GetTFinish()) {
      current_trajectory = std::make_unique<ctrl::Trajectory3D>(*active_trajectories[i]);
      std::cout << "[ctrl::TrajectoryManager::Update] Current trajectory index " << i << std::endl;
      break;
    }
  }

  if (current_trajectory == nullptr) {
    std::cout << "[ctrl::TrajectoryManager::Update] Error! Current trajectory is null. Finishing "
                 "trajectory motion."
              << std::endl;
    return true;
  }

  return false;
}