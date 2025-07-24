#include <iostream>
#include <cassert>

#include "Utils.h"
#include "TrajectoryManager.h"
#include "SystemConfig.h"

/*
  Given a path, it conditionally returns list of trajectories
  Condition is that each of the trajectories is feasible
*/
bool ctrl::TrajectoryManager::CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path_fWorld,
                                                         double t_start_s) {
  if (path_fWorld.size() <= 1) return false;
  Trajectories trajectories;
  double t_end_s = t_start_s;
  for (int path_index = 1; path_index < path_fWorld.size(); ++path_index) {
    Eigen::Vector3d pose_start(path_fWorld[path_index - 1]);
    Eigen::Vector3d pose_end(path_fWorld[path_index]);
    Eigen::Vector3d h(pose_end - pose_start);
    Eigen::Vector3d v0(0, 0, 0);

    // Calculate a more realistic duration for the trajectory accounting for acceleration constraints
    double linear_dist = sqrt(h.x() * h.x() + h.y() * h.y());
    double angular_dist = std::abs(h.z());
    
    // Calculate time needed considering trapezoidal motion profile (acceleration + constant + deceleration)
    // For trapezoidal motion: t = sqrt(2*d/a) for acceleration phase + constant velocity phase
    double max_linear_vel = cfg::SystemConfig::max_velocity_fBody_mps[0];
    double max_angular_vel = cfg::SystemConfig::max_velocity_fBody_mps[2];
    double max_linear_acc = cfg::SystemConfig::max_acc_m_radpsps[0];
    double max_angular_acc = cfg::SystemConfig::max_acc_m_radpsps[2];
    
    // Conservative time calculation for trapezoidal trajectory
    double linear_acc_time = max_linear_vel / max_linear_acc;  // Time to reach max velocity
    double linear_acc_dist = 0.5 * max_linear_acc * linear_acc_time * linear_acc_time;  // Distance during acceleration
    double linear_time;
    if (linear_dist <= 2 * linear_acc_dist) {
        // Short distance - mostly acceleration/deceleration
        linear_time = 2 * sqrt(linear_dist / max_linear_acc);
    } else {
        // Long distance - has constant velocity phase
        linear_time = 2 * linear_acc_time + (linear_dist - 2 * linear_acc_dist) / max_linear_vel;
    }
    
    double angular_acc_time = max_angular_vel / max_angular_acc;
    double angular_acc_dist = 0.5 * max_angular_acc * angular_acc_time * angular_acc_time;
    double angular_time;
    if (angular_dist <= 2 * angular_acc_dist) {
        angular_time = 2 * sqrt(angular_dist / max_angular_acc);
    } else {
        angular_time = 2 * angular_acc_time + (angular_dist - 2 * angular_acc_dist) / max_angular_vel;
    }
    
    double T = std::max(linear_time, angular_time) * 1.5; // Conservative buffer

    if (path_index == 1) v0 = FindV0AtT(t_start_s);

    //std::max({h[0], h[1], h[2]}) * cfg::SystemConfig::avg_velocity_fBody_mps;

    // Create Trajectory
    t_start_s = t_end_s;
    t_end_s += T;
    std::cout << "[ctrl::TrajectoryManager::CreateTrajectoriesFromPath] Added new traj "
              << path_index << std::endl;
    auto traj = std::make_unique<ctrl::TrapezoidalTrajectoryVi3D>(pose_start, pose_end, t_start_s,
                                                                  t_end_s, v0);
    if (traj)
      trajectories.push(std::move(traj));
    else
      return false;
  }
  std::cout << "[ctrl::TrajectoryManager::CreateTrajectoriesFromPath] Added all new trajs\n"
            << std::endl;
  MergeNewTrajectories(std::move(trajectories));
  return true;
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

std::pair<bool, Eigen::Vector3d> ctrl::TrajectoryManager::Update(Eigen::Vector3d pose_est) {
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
  {
    std::unique_lock<std::mutex> lock(active_traj_mutex);
    assert(!(current_trajectory == nullptr && active_trajectories.empty()));
  }

  // Step 3: Update the current trajectory
  if (current_trajectory == nullptr || current_time_s >= current_trajectory->GetTFinish()) {
    {
      std::unique_lock<std::mutex> lock(active_traj_mutex);
      current_trajectory = std::move(active_trajectories.front());
      active_trajectories.pop();
    }
    current_trajectory->Print();
  }
  // Step 4: Get reference velocity from current trajectory
  Eigen::Vector3d velocity_fWorld = GetVelocityAtT(current_time_s);
  Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);

  return std::make_pair(false, velocity_fBody);
}