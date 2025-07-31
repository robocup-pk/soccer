#pragma once

#include "PurePursuitController.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace ctrl {

/**
 * Pure Pursuit-based Trajectory Manager for SSL Robots
 * 
 * This manager uses the Pure Pursuit algorithm to follow multi-waypoint paths,
 * providing smooth trajectory following for omnidirectional SSL robots.
 * It replaces the Bang-Bang trajectory approach with a more suitable algorithm
 * for waypoint following.
 */
class PurePursuitTrajectoryManager {
public:
    PurePursuitTrajectoryManager();
    ~PurePursuitTrajectoryManager() = default;

    // Main interface methods (compatible with existing M_TrajectoryManager)
    bool CreateTrajectoriesFromPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s = 0.0);
    void UpdateRobotState(const Eigen::Vector3d& pose_est, const Eigen::Vector3d& velocity_est);
    std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_est);
    
    // Velocity and position queries (for compatibility with existing interfaces)
    Eigen::Vector3d GetVelocityAtT(double current_time_s);
    Eigen::Vector3d GetPositionAtT(double current_time_s);
    
    // Configuration
    void SetPurePursuitConfig(const PurePursuitController::Config& config);
    const PurePursuitController::Config& GetPurePursuitConfig() const;
    
    // Status and debugging
    void Print() const;
    bool IsPathActive() const { return path_active_; }
    int GetCurrentWaypointIndex() const;
    double GetDistanceToCurrentTarget() const;

private:
    PurePursuitController pure_pursuit_controller_;
    PurePursuitController::RobotState robot_state_;
    
    std::vector<Eigen::Vector3d> current_path_;
    bool path_active_;
    double current_time_;
    
    // For compatibility with existing interface
    Eigen::Vector3d last_commanded_velocity_;
    
    void updateRobotState(const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity);
};

} // namespace ctrl