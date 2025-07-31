#ifndef WAYPOINT_HANDLER_H
#define WAYPOINT_HANDLER_H

#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Dense>

#include "TrajectoryManager.h"
#include "Utils.h"

namespace ctrl {

/**
 * @brief Handles multi-waypoint path following for omnidirectional robots
 *
 * This class manages sequential waypoint navigation, automatically advancing
 * through a path when waypoints are reached within tolerance. It integrates
 * with the existing TrajectoryManager for smooth trajectory generation.
 */
class WaypointHandler {
 public:
  /**
   * @brief Configuration parameters for waypoint following
   */
  struct Config {
    double position_tolerance;  // Position tolerance in meters
    double angle_tolerance;     // Angle tolerance in radians (~11 degrees)
    double waypoint_timeout;    // Maximum time to reach a waypoint (seconds)
    bool loop_path;             // Whether to loop back to start when path is complete

    Config()
        : position_tolerance(0.1),
          angle_tolerance(0.2),
          waypoint_timeout(30.0),
          loop_path(false) {}
    Config(double pos_tol, double ang_tol, double timeout = 30.0, bool loop = false)
        : position_tolerance(pos_tol),
          angle_tolerance(ang_tol),
          waypoint_timeout(timeout),
          loop_path(loop) {}
  };

  /**
   * @brief Callback function type for waypoint events
   * Parameters: waypoint_index, waypoint_pose, current_pose
   */
  using WaypointCallback =
      std::function<void(int, const Eigen::Vector3d&, const Eigen::Vector3d&)>;

  /**
   * @brief Current state of waypoint following
   */
  enum class State {
    IDLE,              // No active path
    FOLLOWING_PATH,    // Actively following waypoints
    WAYPOINT_REACHED,  // Just reached a waypoint
    PATH_COMPLETED,    // All waypoints reached
    WAYPOINT_TIMEOUT,  // Timeout waiting for waypoint
    ERROR              // Error state
  };

 private:
  // Path and waypoint management
  std::vector<Eigen::Vector3d> waypoints_;
  int current_waypoint_index_;
  double waypoint_start_time_;

  // Configuration and state
  Config config_;
  State state_;

  // Integration with trajectory system
  std::shared_ptr<TrajectoryManager> trajectory_manager_;

  // Callbacks
  WaypointCallback on_waypoint_reached_;
  WaypointCallback on_path_completed_;
  WaypointCallback on_waypoint_timeout_;

 public:
  /**
   * @brief Constructor
   * @param trajectory_manager Shared pointer to trajectory manager for path execution
   * @param config Configuration parameters
   */
  explicit WaypointHandler(std::shared_ptr<TrajectoryManager> trajectory_manager = nullptr,
                           const Config& config = Config());

  /**
   * @brief Set trajectory manager (can be done after construction)
   */
  void SetTrajectoryManager(std::shared_ptr<TrajectoryManager> trajectory_manager);

  /**
   * @brief Start following a multi-waypoint path
   * @param waypoints Vector of 3D poses (x, y, theta) to visit sequentially
   * @param start_immediately If true, immediately generate trajectory to first waypoint
   * @return true if path was successfully set, false otherwise
   */
  bool StartPath(const std::vector<Eigen::Vector3d>& waypoints, bool start_immediately = true);

  /**
   * @brief Update waypoint following logic (call this in your main loop)
   * @param current_pose Current robot pose in world frame
   * @return Current state of waypoint following
   */
  State Update(const Eigen::Vector3d& current_pose);

  /**
   * @brief Stop the current path and clear all waypoints
   */
  void StopPath();

  /**
   * @brief Pause/resume path following
   */
  void SetPaused(bool paused);
  bool IsPaused() const;

  // State and progress queries
  State GetState() const { return state_; }
  int GetCurrentWaypointIndex() const { return current_waypoint_index_; }
  int GetTotalWaypoints() const { return static_cast<int>(waypoints_.size()); }
  double GetProgressPercent() const;

  /**
   * @brief Get the current target waypoint
   * @return Current waypoint pose, or zero vector if no active waypoint
   */
  Eigen::Vector3d GetCurrentWaypoint() const;

  /**
   * @brief Get distance to current waypoint
   * @param current_pose Current robot pose
   * @return Distance to current waypoint (position + angle components)
   */
  std::pair<double, double> GetDistanceToCurrentWaypoint(
      const Eigen::Vector3d& current_pose) const;

  /**
   * @brief Check if a specific waypoint is reached
   * @param current_pose Current robot pose
   * @param target_waypoint Target waypoint to check
   * @return true if waypoint is within tolerance
   */
  bool IsWaypointReached(const Eigen::Vector3d& current_pose,
                         const Eigen::Vector3d& target_waypoint) const;

  // Configuration
  void SetConfig(const Config& config) { config_ = config; }
  const Config& GetConfig() const { return config_; }

  // Callback registration
  void SetOnWaypointReached(const WaypointCallback& callback) { on_waypoint_reached_ = callback; }
  void SetOnPathCompleted(const WaypointCallback& callback) { on_path_completed_ = callback; }
  void SetOnWaypointTimeout(const WaypointCallback& callback) { on_waypoint_timeout_ = callback; }

  // Utility functions
  std::string StateToString() const;
  void PrintStatus() const;
  void PrintPath() const;

 private:
  /**
   * @brief Advance to the next waypoint in the path
   * @return true if advanced to next waypoint, false if path is complete
   */
  bool AdvanceToNextWaypoint();

  /**
   * @brief Generate trajectory to current waypoint using TrajectoryManager
   */
  void GenerateTrajectoryToCurrentWaypoint();

  /**
   * @brief Check for waypoint timeout
   */
  void CheckWaypointTimeout();

  bool paused_;
};

}  // namespace ctrl

#endif  // WAYPOINT_HANDLER_H
