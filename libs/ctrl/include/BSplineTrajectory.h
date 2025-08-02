#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include "RobotDescription.h"
#include "Utils.h"

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
    double BSplineBasis(int i, int p, double u) const;
    
    // Evaluate B-spline at parameter u
    Eigen::Vector3d EvaluateBSpline(double u) const;
    
    // Evaluate B-spline derivative at parameter u
    Eigen::Vector3d EvaluateBSplineDerivative(double u, int derivative_order) const;
    
    // Calculate arc length of the B-spline
    void CalculateArcLength();
    
    // Convert arc length to parameter u
    double ArcLengthToParameter(double arc_length) const;
    
    // Compute desired arc length at given time (trapezoidal velocity profile)
    double ComputeDesiredArcLength(double elapsed_time) const;
    
    // Compute desired speed at given time
    double ComputeDesiredSpeed(double elapsed_time) const;
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle) const;
    
    // Smart repetition methods for corner handling
    double CalculateAngleBetween(const Eigen::Vector3d& p1, 
                               const Eigen::Vector3d& p2, 
                               const Eigen::Vector3d& p3) const;
    
    std::vector<Eigen::Vector3d> SmartRepetition(const std::vector<Eigen::Vector3d>& points, 
                                                 int min_repeats = 1, 
                                                 int max_repeats = 4) const;
    
    std::vector<Eigen::Vector3d> LinearEntryExitClosedLoop(const std::vector<Eigen::Vector3d>& points, 
                                                           int repeats = 3, 
                                                           double tolerance = 0.05) const;
    
    // Alternative corner handling approach
    std::vector<Eigen::Vector3d> InsertCornerControlPoints(const std::vector<Eigen::Vector3d>& waypoints,
                                                          double corner_offset = 0.1) const;
    
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
    
    // Velocity and acceleration limits (respecting system constraints)
    double v_max_ = 0.8;        // m/s - below system limit of 1.0 m/s
    double a_max_ = 0.5;        // m/s² - moderate acceleration
    double omega_max_ = 2.5;    // rad/s - below system limit of 5.0 rad/s
    double alpha_max_ = 3.0;    // rad/s² - moderate angular acceleration
    
    // Feedback control gains
    double kp_ = 1.5;  // Proportional gain
    double kd_ = 0.3;  // Derivative gain (not used in current implementation)
    
public:
    // Additional methods for replanning support
    Eigen::Vector3d GetCurrentDesiredPosition() const {
        if (!is_trajectory_active_) return Eigen::Vector3d::Zero();
        double elapsed = util::GetCurrentTime() - trajectory_start_time_;
        double arc_length = ComputeDesiredArcLength(elapsed);
        double u = ArcLengthToParameter(arc_length);
        return EvaluateBSpline(u);
    }
    
    std::vector<Eigen::Vector3d> GetRemainingWaypoints() const {
        if (!is_trajectory_active_) return {};
        
        // Return waypoints that haven't been reached yet
        double elapsed = util::GetCurrentTime() - trajectory_start_time_;
        double arc_length = ComputeDesiredArcLength(elapsed);
        double progress = arc_length / total_arc_length_;
        
        std::vector<Eigen::Vector3d> remaining;
        int start_idx = static_cast<int>(progress * waypoints_.size());
        for (size_t i = start_idx; i < waypoints_.size(); ++i) {
            remaining.push_back(waypoints_[i]);
        }
        return remaining;
    }
};

} // namespace ctrl