#include "WaypointHandler.h"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace ctrl {

WaypointHandler::WaypointHandler(std::shared_ptr<TrajectoryManager> trajectory_manager,
                                 const Config& config)
    : trajectory_manager_(trajectory_manager),
      config_(config),
      current_waypoint_index_(0),
      waypoint_start_time_(0.0),
      state_(State::IDLE),
      paused_(false) {}

void WaypointHandler::SetTrajectoryManager(std::shared_ptr<TrajectoryManager> trajectory_manager) {
  trajectory_manager_ = trajectory_manager;
}

bool WaypointHandler::StartPath(const std::vector<Eigen::Vector3d>& waypoints,
                                bool start_immediately) {
  if (waypoints.empty()) {
    std::cout << "[WaypointHandler] Error: Cannot start with empty waypoint list" << std::endl;
    state_ = State::ERROR;
    return false;
  }

  if (!trajectory_manager_) {
    std::cout << "[WaypointHandler] Error: No trajectory manager set" << std::endl;
    state_ = State::ERROR;
    return false;
  }

  // Store the waypoints
  waypoints_ = waypoints;
  current_waypoint_index_ = 0;
  waypoint_start_time_ = util::GetCurrentTime();
  state_ = State::FOLLOWING_PATH;
  paused_ = false;

  std::cout << "[WaypointHandler] Starting path with " << waypoints.size() << " waypoints"
            << std::endl;
  PrintPath();

  if (start_immediately) {
    GenerateTrajectoryToCurrentWaypoint();
  }

  return true;
}

WaypointHandler::State WaypointHandler::Update(const Eigen::Vector3d& current_pose) {
  if (state_ == State::IDLE || state_ == State::ERROR || paused_) {
    return state_;
  }

  if (state_ == State::PATH_COMPLETED) {
    if (config_.loop_path && !waypoints_.empty()) {
      // Restart the path
      current_waypoint_index_ = 0;
      waypoint_start_time_ = util::GetCurrentTime();
      state_ = State::FOLLOWING_PATH;
      GenerateTrajectoryToCurrentWaypoint();
      std::cout << "[WaypointHandler] Looping path - restarting from waypoint 0" << std::endl;
    }
    return state_;
  }

  if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
    // Path completed
    state_ = State::PATH_COMPLETED;
    if (on_path_completed_) {
      on_path_completed_(current_waypoint_index_, Eigen::Vector3d::Zero(), current_pose);
    }
    std::cout << "[WaypointHandler] Path completed!" << std::endl;
    return state_;
  }

  // Check for waypoint timeout
  CheckWaypointTimeout();
  if (state_ == State::WAYPOINT_TIMEOUT) {
    return state_;
  }

  // Check if current waypoint is reached
  Eigen::Vector3d target_waypoint = waypoints_[current_waypoint_index_];

  if (IsWaypointReached(current_pose, target_waypoint)) {
    state_ = State::WAYPOINT_REACHED;

    // Call callback
    if (on_waypoint_reached_) {
      on_waypoint_reached_(current_waypoint_index_, target_waypoint, current_pose);
    }

    auto [pos_dist, ang_dist] = GetDistanceToCurrentWaypoint(current_pose);
    std::cout << "[WaypointHandler] ✓ Reached waypoint " << current_waypoint_index_
              << " (pos_err: " << std::fixed << std::setprecision(3) << pos_dist
              << "m, ang_err: " << ang_dist << " rad)" << std::endl;

    // Advance to next waypoint
    if (AdvanceToNextWaypoint()) {
      state_ = State::FOLLOWING_PATH;
      GenerateTrajectoryToCurrentWaypoint();
    } else {
      state_ = State::PATH_COMPLETED;
      if (on_path_completed_) {
        on_path_completed_(current_waypoint_index_, target_waypoint, current_pose);
      }
      std::cout << "[WaypointHandler] 🎯 All waypoints reached - path completed!" << std::endl;
    }
  } else {
    state_ = State::FOLLOWING_PATH;
  }

  return state_;
}

void WaypointHandler::StopPath() {
  waypoints_.clear();
  current_waypoint_index_ = 0;
  state_ = State::IDLE;
  paused_ = false;
  std::cout << "[WaypointHandler] Path stopped" << std::endl;
}

void WaypointHandler::SetPaused(bool paused) {
  if (paused_ != paused) {
    paused_ = paused;
    std::cout << "[WaypointHandler] Path " << (paused ? "paused" : "resumed") << std::endl;
  }
}

bool WaypointHandler::IsPaused() const { return paused_; }

double WaypointHandler::GetProgressPercent() const {
  if (waypoints_.empty()) return 0.0;

  if (state_ == State::PATH_COMPLETED) return 100.0;

  return (static_cast<double>(current_waypoint_index_) / static_cast<double>(waypoints_.size())) *
         100.0;
}

Eigen::Vector3d WaypointHandler::GetCurrentWaypoint() const {
  if (current_waypoint_index_ >= 0 &&
      current_waypoint_index_ < static_cast<int>(waypoints_.size())) {
    return waypoints_[current_waypoint_index_];
  }
  return Eigen::Vector3d::Zero();
}

std::pair<double, double> WaypointHandler::GetDistanceToCurrentWaypoint(
    const Eigen::Vector3d& current_pose) const {
  if (current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
    return {0.0, 0.0};
  }

  Eigen::Vector3d target = waypoints_[current_waypoint_index_];

  // Position distance (2D)
  double position_distance = (current_pose.head<2>() - target.head<2>()).norm();

  // Angle distance (wrapped)
  double angle_distance = std::abs(util::WrapAngle(current_pose[2] - target[2]));

  return {position_distance, angle_distance};
}

bool WaypointHandler::IsWaypointReached(const Eigen::Vector3d& current_pose,
                                        const Eigen::Vector3d& target_waypoint) const {
  auto [pos_dist, ang_dist] = GetDistanceToCurrentWaypoint(current_pose);

  return (pos_dist < config_.position_tolerance) && (ang_dist < config_.angle_tolerance);
}

std::string WaypointHandler::StateToString() const {
  switch (state_) {
    case State::IDLE:
      return "IDLE";
    case State::FOLLOWING_PATH:
      return "FOLLOWING_PATH";
    case State::WAYPOINT_REACHED:
      return "WAYPOINT_REACHED";
    case State::PATH_COMPLETED:
      return "PATH_COMPLETED";
    case State::WAYPOINT_TIMEOUT:
      return "WAYPOINT_TIMEOUT";
    case State::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void WaypointHandler::PrintStatus() const {
  std::cout << "[WaypointHandler] Status:" << std::endl;
  std::cout << "  State: " << StateToString() << std::endl;
  std::cout << "  Waypoint: " << current_waypoint_index_ << "/" << waypoints_.size() << std::endl;
  std::cout << "  Progress: " << std::fixed << std::setprecision(1) << GetProgressPercent() << "%"
            << std::endl;
  std::cout << "  Paused: " << (paused_ ? "YES" : "NO") << std::endl;

  if (current_waypoint_index_ < static_cast<int>(waypoints_.size())) {
    Eigen::Vector3d target = GetCurrentWaypoint();
    std::cout << "  Current Target: (" << std::fixed << std::setprecision(2) << target[0] << ", "
              << target[1] << ", " << target[2] << ")" << std::endl;
  }
}

void WaypointHandler::PrintPath() const {
  std::cout << "[WaypointHandler] Path (" << waypoints_.size() << " waypoints):" << std::endl;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    std::cout << "  [" << i << "] (" << std::fixed << std::setprecision(2) << waypoints_[i][0]
              << ", " << waypoints_[i][1] << ", " << waypoints_[i][2] << ")";
    if (i == static_cast<size_t>(current_waypoint_index_)) {
      std::cout << " ← CURRENT";
    }
    std::cout << std::endl;
  }
}

bool WaypointHandler::AdvanceToNextWaypoint() {
  current_waypoint_index_++;
  waypoint_start_time_ = util::GetCurrentTime();

  if (current_waypoint_index_ < static_cast<int>(waypoints_.size())) {
    Eigen::Vector3d next_waypoint = waypoints_[current_waypoint_index_];
    std::cout << "[WaypointHandler] → Advanced to waypoint " << current_waypoint_index_ << ": ("
              << std::fixed << std::setprecision(2) << next_waypoint[0] << ", " << next_waypoint[1]
              << ", " << next_waypoint[2] << ")" << std::endl;
    return true;
  }

  return false;  // Path completed
}

void WaypointHandler::GenerateTrajectoryToCurrentWaypoint() {
  if (!trajectory_manager_ || current_waypoint_index_ >= static_cast<int>(waypoints_.size())) {
    return;
  }

  Eigen::Vector3d target_waypoint = waypoints_[current_waypoint_index_];

  // Create a simple two-point path for TrajectoryManager
  std::vector<Eigen::Vector3d> path_segment = {target_waypoint};

  // Use TrajectoryManager to generate trajectory
  if (trajectory_manager_->CreateTrajectoriesFromPath(path_segment)) {
    std::cout << "[WaypointHandler] Generated trajectory to waypoint " << current_waypoint_index_
              << ": (" << std::fixed << std::setprecision(2) << target_waypoint[0] << ", "
              << target_waypoint[1] << ", " << target_waypoint[2] << ")" << std::endl;
  } else {
    std::cout << "[WaypointHandler] Warning: Failed to generate trajectory to waypoint "
              << current_waypoint_index_ << std::endl;
  }
}

void WaypointHandler::CheckWaypointTimeout() {
  if (state_ != State::FOLLOWING_PATH) return;

  double elapsed = util::GetCurrentTime() - waypoint_start_time_;
  if (elapsed > config_.waypoint_timeout) {
    state_ = State::WAYPOINT_TIMEOUT;
    if (on_waypoint_timeout_) {
      Eigen::Vector3d target = GetCurrentWaypoint();
      on_waypoint_timeout_(current_waypoint_index_, target, Eigen::Vector3d::Zero());
    }
    std::cout << "[WaypointHandler] ⚠️ Timeout waiting for waypoint " << current_waypoint_index_
              << " (elapsed: " << elapsed << "s)" << std::endl;
  }
}

}  // namespace ctrl
