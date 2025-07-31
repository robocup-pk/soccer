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

class BSplineTrajectory {
public:
    BSplineTrajectory();
    ~BSplineTrajectory() = default;
    
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
    
    // Set B-spline degree (1-5, default 3 for cubic)
    void SetSplineDegree(int degree);
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Check if trajectory is active
    bool IsActive() const { return is_trajectory_active_; }
    
    // Check if trajectory is finished
    bool IsFinished() const { return is_trajectory_finished_; }
    
    // Reset trajectory
    void ResetTrajectory();
    
private:
    // Generate B-spline control points from waypoints
    void GenerateBSplineControlPoints();
    
    // Generate knot vector for B-spline
    void GenerateKnotVector();
    
    // B-spline basis function (Cox-de Boor recursion)
    double BSplineBasis(int i, int p, double u);
    
    // Evaluate B-spline at parameter u
    Eigen::Vector3d EvaluateBSpline(double u);
    
    // Evaluate B-spline derivative at parameter u
    Eigen::Vector3d EvaluateBSplineDerivative(double u, int derivative_order);
    
    // Calculate arc length of the B-spline
    void CalculateArcLength();
    
    // Convert arc length to parameter u
    double ArcLengthToParameter(double arc_length);
    
    // Compute desired arc length at given time (trapezoidal velocity profile)
    double ComputeDesiredArcLength(double elapsed_time);
    
    // Compute desired speed at given time
    double ComputeDesiredSpeed(double elapsed_time);
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle);
    
private:
    // Robot description
    kin::RobotDescription robot_description_;
    
    // B-spline data
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<double> knot_vector_;
    int spline_degree_;
    
    // Arc length parameterization
    double total_arc_length_;
    std::vector<double> arc_length_samples_;
    
    // Original waypoints
    std::vector<Eigen::Vector3d> waypoints_;
    
    // Trajectory timing
    double trajectory_start_time_;
    double current_segment_progress_;
    
    // Trajectory state
    bool is_trajectory_active_;
    bool is_trajectory_finished_;
    
    // Velocity and acceleration limits
    double v_max_ = 0.6;        // m/s - conservative for smooth motion
    double a_max_ = 1.0;        // m/s² - smooth acceleration
    double omega_max_ = 2.0;    // rad/s - conservative angular velocity
    double alpha_max_ = 3.0;    // rad/s² - smooth angular acceleration
    
    // Feedback control gains
    double kp_ = 1.5;  // Proportional gain
    double kd_ = 0.3;  // Derivative gain (not used in current implementation)
};

} // namespace ctrl