#include <iostream>
#include <cassert>
#include <cmath>

#include "Utils.h"
#include "TrajectoryManager.h"
#include "SystemConfig.h"
#include "Spline2D.h"
#include "Heading1D.h"

static double angleBetweenVectors(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  double dot = a.dot(b) / (a.norm() * b.norm());
  return std::acos(std::clamp(dot, -1.0, 1.0));  // radians
}

bool ctrl::TrajectoryManager::CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path_fWorld,
                                                         double t_start_s) {
  if (path_fWorld.size() <= 1) return false;

  // === 1. Preprocess: Repeat sharp corners ===
  std::vector<Eigen::Vector3d> processed_path;
  processed_path.push_back(path_fWorld[0]);

  for (size_t i = 1; i < path_fWorld.size() - 1; ++i) {
    Eigen::Vector2d prev = path_fWorld[i] - path_fWorld[i - 1];
    Eigen::Vector2d next = path_fWorld[i + 1] - path_fWorld[i];
    double angle_deg = angleBetweenVectors(prev, next) * 180.0 / M_PI;

    int repeats = 1;
    if (angle_deg <= 60)
      repeats = 4;
    else if (angle_deg <= 90)
      repeats = 3;
    else if (angle_deg <= 120)
      repeats = 2;

    for (int r = 0; r < repeats; r++) {
      processed_path.push_back(path_fWorld[i]);
    }
  }
  processed_path.push_back(path_fWorld.back());

  // === 2. Create Spline2D for XY motion ===
  spline_traj = std::make_unique<Spline2D>(processed_path, t_start_s);

  // === 3. Create Heading1D for orientation ===
  double heading_start = processed_path.front()[2];
  double heading_end = processed_path.back()[2];
  double spline_total_time = spline_traj->GetTotalTime();

  heading_traj = std::make_unique<Heading1D>(heading_start, heading_end, t_start_s,
                                             t_start_s + spline_total_time);

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