#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "CubicHermiteSplineTrajectory.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

class HermiteSplineTrajectoryManager {
public:
    HermiteSplineTrajectoryManager();
    ~HermiteSplineTrajectoryManager() = default;
    
    // Create trajectories from RRT* waypoints
    bool CreateTrajectoriesFromPath(const std::vector<Eigen::Vector3d>& path_fWorld, 
                                    double t_start_s = 0.0);
    
    // Update and get velocity command
    std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_est);
    
    // Reset trajectory
    void Reset();
    
    // Set controller parameters
    void SetParameters(double lookahead_distance, double speed_scaling);
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Set velocity limits
    void SetVelocityLimits(double v_max, double omega_max);
    
    // Set feedback control gains
    void SetFeedbackGains(double kp, double kd);
    
    // Query functions
    Eigen::Vector3d GetVelocityAtT(double current_time_s);
    Eigen::Vector3d GetPositionAtT(double current_time_s);
    bool IsTrajectoryActive() const;
    double GetRemainingTime() const;
    
    // Print trajectory info
    void Print() { /* Implementation if needed */ }
    
private:
    // Apply safety limits to velocity command
    void ApplySafetyLimits(Eigen::Vector3d& velocity);
    
private:
    // Hermite spline trajectory planner
    std::unique_ptr<CubicHermiteSplineTrajectory> hermite_trajectory_;
    
    // Current robot pose
    Eigen::Vector3d p_fworld_;
    
    // Path waypoints
    std::vector<Eigen::Vector3d> current_waypoints_;
    
    // Controller parameters
    double lookahead_distance_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double speed_scaling_factor_;
    
    // Velocity limits
    double max_linear_velocity_ = 2.0;    // m/s
    double max_angular_velocity_ = 5.0;   // rad/s
    
    // Trajectory state
    bool trajectory_active_;
    bool trajectory_finished_;
};

} // namespace ctrl