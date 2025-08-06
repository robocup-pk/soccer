#ifndef ADAPTIVE_TRAJECTORY_H
#define ADAPTIVE_TRAJECTORY_H

#include <queue>
#include <mutex>
#include <memory>
#include <Eigen/Dense>
#include <chrono>

#include "Trajectory3D.h"
#include "Utils.h"
#include "tinysplinecxx.h"

// Forward declarations
struct Point2D;
struct Vector2D;
class AdaptiveTrajectoryFollower;

namespace ctrl {

typedef std::queue<std::unique_ptr<ctrl::Trajectory3D>> Trajectories;

class AdaptiveTrajectory {
 public:
  // Constructor with motion parameters
  AdaptiveTrajectory(double max_velocity = 1.0, double max_acceleration = 2.0,
                     double position_tolerance = 0.1, double replan_threshold = 0.2,
                     int min_replan_interval_ms = 100);

  ~AdaptiveTrajectory();

  // Main interface methods (matching TrajectoryManager)
  bool CreateTrajectoriesFromPath(std::vector<Eigen::Vector3d> path_fWorld,
                                  double t_start_s = util::GetCurrentTime());

  void MergeNewTrajectories(Trajectories&& new_trajectories);

  Eigen::Vector3d GetVelocityAtT(double time_s);

  std::pair<bool, Eigen::Vector3d> Update(Eigen::Vector3d pose_est);

  // Additional methods for adaptive functionality
  void addWaypoints(const std::vector<Eigen::Vector3d>& new_waypoints);

  // Status queries
  bool isTrajectoryActive() const;
  bool hasRemainingWaypoints() const;
  size_t getRemainingWaypointCount() const;

  // Parameter setters
  void setMaxVelocity(double vel);
  void setMaxAcceleration(double acc);
  void setPositionTolerance(double tol);
  void setReplanThreshold(double thresh);

  // Legacy interface support
  Eigen::Vector3d FindV0AtT(double t);
  void MergeNewTrajectoriesFirstCall(Trajectories&& new_trajectories);
  void MergeNewTrajectoriesInFuture(Trajectories&& new_trajectories);
  void MergeNewTrajectoriesAtT(Trajectories&& new_trajectories);
  void Print();

  // Public member for compatibility (though not recommended)
  Trajectories active_trajectories;
  double active_traj_t_finish_s;

 private:
  // Helper methods
  std::vector<Point2D> convertPath(const std::vector<Eigen::Vector3d>& path);

  // Core adaptive follower
  std::unique_ptr<AdaptiveTrajectoryFollower> follower_;

  // State tracking
  Eigen::Vector3d last_pose_;
  Eigen::Vector3d last_velocity_;  // Store last computed velocity for GetVelocityAtT
  std::chrono::steady_clock::time_point last_update_time_;
  bool initialized_;

  // Thread safety
  mutable std::mutex follower_mutex_;

  // Z-axis (yaw) handling - no motion assumed for now
  double target_yaw_;
  double current_yaw_;
};

}  // namespace ctrl

#endif  // ADAPTIVE_TRAJECTORY_H