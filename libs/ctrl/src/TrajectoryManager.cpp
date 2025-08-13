#include <iostream>
#include <cassert>
#include <cmath>

#include "Utils.h"
#include "TrajectoryManager.h"
#include "SystemConfig.h"


bool ctrl::TrajectoryManager::CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path_fWorld,
                                                         double t_start_s) {
  if (path_fWorld.size() <= 1) return false;

  // === 1. Build 2D path and headings (no corner repetition here) ===
  std::vector<Eigen::Vector2d> processed_path_2d;
  std::vector<double> headings;
  processed_path_2d.reserve(path_fWorld.size());
  headings.reserve(path_fWorld.size());
  for (const auto &pt : path_fWorld) {
    processed_path_2d.emplace_back(pt.x(), pt.y());
    headings.push_back(pt[2]);
  }

  // === 2. Create Spline2D for XY motion ===
  spline_traj = std::make_unique<Spline2D>();
  spline_traj->Init(processed_path_2d, ::cfg::SystemConfig::max_velocity_fBody_mps.x(),
                    ::cfg::SystemConfig::max_acc_m_radpsps.x(),
                    ::cfg::SystemConfig::max_acc_m_radpsps.y(), t_start_s, 300, true);
  // === 3. Create Heading1D for orientation ===
  double heading_start = headings.front();
  double heading_end = headings.back();
  double spline_total_time = spline_traj->GetTotalTime();

  heading_traj = std::make_unique<Heading1D>();
  heading_traj->Init(heading_start, heading_end, t_start_s,
                     ::cfg::SystemConfig::max_velocity_fBody_mps.z(), ::cfg::SystemConfig::max_acc_m_radpsps.z(),
                     spline_total_time);

  active_traj_t_finish_s = t_start_s + spline_total_time;

  std::cout << "[ctrl::TrajectoryManager::CreateTrajectoriesFromPath] Created spline + heading"
            << " with total time: " << spline_total_time << "s\n";

  return true;
}

Eigen::Vector3d ctrl::TrajectoryManager::GetVelocityAtT(double current_time_s) {
  Eigen::Vector2d v_xy = spline_traj->VelocityAtT(current_time_s);
  double omega = heading_traj->VelocityAtT(current_time_s);
  return Eigen::Vector3d(v_xy[0], v_xy[1], omega);
}

void ctrl::TrajectoryManager::Print() {
  // std::cout << "[ctrl::TrajectoryManager::Print] Active Trajectories: \n";
  // for (int i = 0; i < active_trajectories.size(); ++i) {
  //   active_trajectories[i]->Print();
  // }
}

std::pair<bool, Eigen::Vector3d> ctrl::TrajectoryManager::Update(Eigen::Vector3d pose_est) {
  double current_time_s = util::GetCurrentTime();
  if (!spline_traj || !heading_traj) {
    return {true, Eigen::Vector3d(0, 0, 0)};
  }

  if (current_time_s >= active_traj_t_finish_s) {
    std::cout << "[ctrl::TrajectoryManager::Update] Finished motion.\n";
    return {true, Eigen::Vector3d(0, 0, 0)};
  }

  Eigen::Vector3d velocity_fWorld = GetVelocityAtT(current_time_s);
  Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
  return {false, velocity_fBody};
}