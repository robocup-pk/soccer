#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include "RobotDescription.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

struct HermiteSegment {
    Eigen::Vector3d p0;      // Start position
    Eigen::Vector3d p1;      // End position
    Eigen::Vector3d m0;      // Start tangent (velocity)
    Eigen::Vector3d m1;      // End tangent (velocity)
    double duration;         // Segment duration
    
    // Pre-computed coefficients for cubic Hermite polynomial
    Eigen::Vector3d c0, c1, c2, c3;
};

class CubicHermiteSplineTrajectory {
public:
    CubicHermiteSplineTrajectory();
    ~CubicHermiteSplineTrajectory() = default;
    
    // Set path from RRT* waypoints
    bool SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s = 0.0);
    
    // Update trajectory and get velocity command
    Eigen::Vector3d Update(const Eigen::Vector3d& current_pose, double current_time);
    
    // Add a new goal (for dynamic replanning)
    bool AddGoal(const Eigen::Vector3d& goal);
    
    // Get remaining time for current trajectory
    double GetRemainingTime() const;
    
    // Set velocity and acceleration limits
    void SetLimits(double v_max, double a_max, double omega_max, double alpha_max);
    
    // Set feedback control gains
    void SetFeedbackGains(double kp, double kd);
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Check if trajectory is active
    bool IsActive() const { return is_trajectory_active_; }
    
    // Check if trajectory is finished
    bool IsFinished() const { return is_trajectory_finished_; }
    
    // Reset trajectory
    void ResetTrajectory();
    
private:
    // Generate Hermite spline segments from waypoints
    void GenerateHermiteSplineSegments();
    
    // Compute tangent at a waypoint using neighboring points
    Eigen::Vector3d ComputeTangent(const Eigen::Vector3d& p_prev, 
                                   const Eigen::Vector3d& p_curr, 
                                   const Eigen::Vector3d& p_next);
    
    // Compute Hermite polynomial coefficients
    void ComputeHermiteCoefficients(HermiteSegment& segment);
    
    // Evaluate Hermite segment at normalized time t ∈ [0, 1]
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> 
    EvaluateHermiteSegment(const HermiteSegment& segment, double t);
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle);
    
private:
    // Robot description
    kin::RobotDescription robot_description_;
    
    // Trajectory segments
    std::vector<HermiteSegment> segments_;
    
    // Original waypoints
    std::vector<Eigen::Vector3d> waypoints_;
    
    // Trajectory timing
    double trajectory_start_time_;
    double total_trajectory_time_;
    
    // Trajectory state
    bool is_trajectory_active_;
    bool is_trajectory_finished_;
    size_t current_segment_index_;
    
    // Velocity and acceleration limits
    double v_max_ = 2.0;        // m/s
    double a_max_ = 3.0;        // m/s²
    double omega_max_ = 5.0;    // rad/s
    double alpha_max_ = 10.0;   // rad/s²
    
    // Feedback control gains
    double kp_ = 2.0;  // Proportional gain
    double kd_ = 0.5;  // Derivative gain (not used in current implementation)
};

} // namespace ctrl