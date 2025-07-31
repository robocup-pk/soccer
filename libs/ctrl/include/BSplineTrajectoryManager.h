#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "BSplineTrajectory.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

class BSplineTrajectoryManager {
public:
    BSplineTrajectoryManager();
    ~BSplineTrajectoryManager() = default;
    
    // Create trajectories from RRT* waypoints
    bool CreateTrajectoriesFromPath(const std::vector<Eigen::Vector3d>& path_fWorld, 
                                    double t_start_s = 0.0);
    
    // Update and get velocity command
    std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_est);
    
    // Reset trajectory
    void Reset();
    
    // Enable/disable additional smoothing
    void SetSmoothingEnabled(bool enabled);
    
    // Set B-spline degree (1-5, default 3)
    void SetSplineDegree(int degree);
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Set velocity limits
    void SetVelocityLimits(double v_max, double omega_max);
    
    // Set feedback control gains
    void SetFeedbackGains(double kp, double kd);
    
    // Set minimum distance between waypoints (for filtering)
    void SetMinWaypointDistance(double distance);
    
    // Query functions
    Eigen::Vector3d GetVelocityAtT(double current_time_s);
    Eigen::Vector3d GetPositionAtT(double current_time_s);
    bool IsTrajectoryActive() const;
    double GetRemainingTime() const;
    
    // Print trajectory info
    void Print() { /* Implementation if needed */ }
    
private:
    // Preprocess waypoints for smoother trajectories
    std::vector<Eigen::Vector3d> PreprocessWaypoints(const std::vector<Eigen::Vector3d>& waypoints);
    
    // Apply additional smoothing filter to velocities
    void ApplySmoothingFilter(Eigen::Vector3d& velocity);
    
    // Apply safety limits to velocity command
    void ApplySafetyLimits(Eigen::Vector3d& velocity);
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle);
    
private:
    // B-spline trajectory planner
    std::unique_ptr<BSplineTrajectory> bspline_trajectory_;
    
    // Current robot pose
    Eigen::Vector3d p_fworld_;
    
    // Path waypoints
    std::vector<Eigen::Vector3d> current_waypoints_;
    
    // Trajectory state
    bool trajectory_active_;
    bool trajectory_finished_;
    
    // Smoothing parameters
    bool smoothing_enabled_;
    double min_waypoint_distance_;
    
    // Velocity filter state
    Eigen::Vector3d previous_velocity_;
    bool previous_velocity_initialized_ = false;
};

} // namespace ctrl