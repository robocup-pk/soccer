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
  std::cout << "CreateTrajectoriesFromPath powered up" << std::endl;
  Trajectories trajectories;
  double t_end_s = t_start_s;
  for (int path_index = 1; path_index < path_fWorld.size(); ++path_index) {
    Eigen::Vector3d pose_start(path_fWorld[path_index - 1]);
    Eigen::Vector3d pose_end(path_fWorld[path_index]);
    Eigen::Vector3d h(pose_end - pose_start);
    Eigen::Vector3d v0(0, 0, 0);
    double T = 4.0;
    if (path_index == 1) v0 = FindV0AtT(t_start_s);
    std::cout << "b\n ";

    // std::max({h[0], h[1], h[2]}) * cfg::SystemConfig::avg_velocity_fBody_mps;

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

  std::cout << "CreateTrajectoriesFromPath powered off" << std::endl;
  return true;
}

Eigen::Vector3d ctrl::TrajectoryManager::GetVelocityAtT(double current_time_s) {
  double kp = 0.0;
  Eigen::Vector3d Current_speed = current_trajectory->VelocityAtT(current_time_s);
  Eigen::Vector3d Current_position_fWorld = p_fworld;

  Eigen::Vector3d Final_determined_velocity =
      Current_speed +
      kp * (current_trajectory->PositionAtT(current_time_s) - Current_position_fWorld);
  std::cout
      << "[ctrl::TrajectoryManager::GetVelocityAtT] Error: "
      << (current_trajectory->PositionAtT(current_time_s) - Current_position_fWorld).transpose()
      << std::endl;
  std::cout << "[ctrl::TrajectoryManager::GetVelocityAtT] Final_determinedvelocity: "
            << Final_determined_velocity.transpose() << std::endl;

  return Final_determined_velocity;
}

Eigen::Vector3d ctrl::TrajectoryManager::GetPositionAtT(double current_time_s) {
  if (current_trajectory == nullptr) {
    std::cout << "[ctrl::TrajectoryManager::GetPositionAtT] No current trajectory set."
              << std::endl;
    return Eigen::Vector3d(0, 0, 0);
  }
  return current_trajectory->PositionAtT(current_time_s);
}

void ctrl::TrajectoryManager::Print() {
  // std::cout << "[ctrl::TrajectoryManager::Print] Active Trajectories: \n";
  // for (int i = 0; i < active_trajectories.size(); ++i) {
  //   active_trajectories[i]->Print();
  // }
}

std::pair<bool, Eigen::Vector3d> ctrl::TrajectoryManager::Update(Eigen::Vector3d pose_est) {
  // std::cout << "update powered up" << std::endl;
  p_fworld = pose_est;

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
  if (current_trajectory == nullptr || current_time_s > current_trajectory->GetTFinish()) {
    {
      std::unique_lock<std::mutex> lock(active_traj_mutex);
      current_trajectory = std::move(active_trajectories.front());
      active_trajectories.pop();
    }
  }
  // Step 4: Get reference velocity from current trajectory
  Eigen::Vector3d velocity_fWorld = GetVelocityAtT(current_time_s);
  Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
  std::cout << "[ctrl::TrajectoryManager::Update] Velocity_Frame Body: "
            << velocity_fBody.transpose() << std::endl;
  // std::cout << "update powered down" << std::endl;
  return std::make_pair(false, velocity_fBody);
}