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

/**
 * UniformBSplineTrajectoryPlanner - Based on EWOK approach
 * 
 * This implementation uses uniform B-splines with the following key features:
 * 1. Uniform knot vector (constant time intervals between knots)
 * 2. Cubic B-splines (degree 3) for C2 continuity
 * 3. Control points generated from RRT* waypoints
 * 4. Guaranteed to stay within convex hull of control points
 * 5. Proven to work in RoboCup SSL environments
 */
class UniformBSplineTrajectoryPlanner {
public:
    UniformBSplineTrajectoryPlanner();
    ~UniformBSplineTrajectoryPlanner() = default;
    
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
    
    // Set feedback control gains (legacy method)
    void SetFeedbackGains(double kp, double kd);
    
    
    // Set B-spline degree (typically 3 for cubic, max 5)
    void SetSplineDegree(int degree);
    
    // Set time interval between control points
    void SetTimeInterval(double dt);
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Check if trajectory is active
    bool IsActive() const { return is_trajectory_active_; }
    
    // Check if trajectory is finished
    bool IsFinished() const { return is_trajectory_finished_; }
    
    // Reset trajectory
    void ResetTrajectory();
    
    // Replanning functionality - EWOK-style partial replanning
    bool CheckAndReplan(const Eigen::Vector3d& state_estimation_pose, 
                       const Eigen::Vector3d& current_commanded_pose,
                       double current_time,
                       double position_error_threshold = 0.05,  // 5cm default
                       double angle_error_threshold = 0.1);     // 0.1 rad default
    
    // Partial trajectory update (EWOK-style)
    bool UpdatePartialTrajectory(const Eigen::Vector3d& current_pose, 
                               int num_control_points_to_update = 4);
    
    // Get remaining waypoints from current position
    std::vector<Eigen::Vector3d> GetRemainingPath(const Eigen::Vector3d& current_pose) const;
    
    // Enable/disable automatic replanning
    void SetReplanningEnabled(bool enabled) { replanning_enabled_ = enabled; }
    bool IsReplanningEnabled() const { return replanning_enabled_; }
    
private:
    // B-spline basis function using Cox-de Boor recursion
    double BSplineBasis(int i, int p, double u, const std::vector<double>& knot_vector) const;
    
    // Evaluate B-spline at parameter u
    Eigen::Vector3d EvaluateBSpline(double u) const;
    
    // Evaluate B-spline derivative at parameter u
    Eigen::Vector3d EvaluateBSplineDerivative(double u, int derivative_order) const;
    
    // Generate control points from waypoints
    void GenerateControlPointsFromWaypoints();
    
    // Generate uniform knot vector
    void GenerateUniformKnotVector();
    
    // Generate centripetal knot vector (alternative)
    void GenerateCentripetalKnotVector(); 
    
    // Calculate arc length for velocity planning
    void CalculateArcLength();
    
    // Convert arc length to parameter u
    double ArcLengthToParameter(double arc_length) const;
    
    // Compute desired arc length at given time (velocity profile)
    double ComputeDesiredArcLength(double elapsed_time) const;
    
    // Compute desired speed at given time
    double ComputeDesiredSpeed(double elapsed_time) const;
    
    // Ensure control points stay within field boundaries
    void ApplyBoundaryConstraints();
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle) const;
    
    // Calculate time for parameter u
    double ParameterToTime(double u) const;
    
    // Find closest parameter on spline to a given position
    double FindClosestParameter(const Eigen::Vector2d& position) const;
    
private:
    // Robot description
    kin::RobotDescription robot_description_;
    
    // B-spline parameters
    static constexpr int SPLINE_DEGREE = 3;  // Cubic B-spline
    static constexpr int SPLINE_ORDER = SPLINE_DEGREE + 1;
    double dt_ = 0.1;  // Time interval between control points
    
    // Control points and knot vector
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<double> knot_vector_;
    
    // Original waypoints from RRT*
    std::vector<Eigen::Vector3d> waypoints_;
    
    // Arc length parameterization
    double total_arc_length_;
    std::vector<double> arc_length_samples_;
    std::vector<double> parameter_samples_;
    
    // Trajectory timing
    double trajectory_start_time_;
    double trajectory_duration_;
    
    // Trajectory state
    bool is_trajectory_active_;
    bool is_trajectory_finished_;
    
    // Velocity and acceleration limits
    double v_max_ = 0.8;        // m/s
    double a_max_ = 0.5;        // m/s²
    double omega_max_ = 2.5;    // rad/s
    double alpha_max_ = 3.0;    // rad/s²
    
    // Feedback control gains (simplified EWOK-style)
    double kp_ = 0.05;  // Proportional gain
    double kd_ = 0.3;   // Derivative gain (currently unused but kept for future extensions)

    // Boundary constraints for square field
    double field_min_x_ = 0.0;
    double field_max_x_ = 1.0;
    double field_min_y_ = 0.0;
    double field_max_y_ = 1.0;
    double boundary_margin_ = 0.005;  // 5mm margin from boundary
    
    // Number of collision checks per segment
    static constexpr int COLLISION_CHECKS_PER_SEGMENT = 10;
    
    // Replanning parameters
    bool replanning_enabled_ = true;
    int replan_count_ = 0;
    double last_replan_time_ = 0.0;
    double min_replan_interval_ = 0.5;  // Don't replan more than once per 0.5 seconds
    
    // For derivative control
    Eigen::Vector3d previous_pose_error_ = Eigen::Vector3d::Zero();
    double previous_update_time_ = 0.0;
    bool has_previous_update_ = false;
    
    // For longitudinal PID control
    double t_error_integral_ = 0.0;
    double t_error_prev_ = 0.0;
    double desired_parameter_rate_ = 0.0;  // desired rate of parameter change
    
    // Low-pass filter for smooth control
    Eigen::Vector3d previous_velocity_command_ = Eigen::Vector3d::Zero();
    bool has_previous_command_ = false;
    double velocity_filter_alpha_ = 0.3;  // Low-pass filter coefficient (0 = full filter, 1 = no filter)
    
    // PID control for precise tracking
    Eigen::Vector2d cross_track_error_integral_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d along_track_error_integral_ = Eigen::Vector2d::Zero();
    double max_integral_ = 0.5;  // Integral windup limit
    
public:
    // Additional methods for debugging and visualization
    Eigen::Vector3d GetCurrentDesiredPosition() const {
        if (!is_trajectory_active_) return Eigen::Vector3d::Zero();
        double elapsed = util::GetCurrentTime() - trajectory_start_time_;
        double arc_length = ComputeDesiredArcLength(elapsed);
        double u = ArcLengthToParameter(arc_length);
        return EvaluateBSpline(u);
    }
    
    Eigen::Vector3d GetDesiredPositionAtTime(double time) const {
        if (!is_trajectory_active_) return Eigen::Vector3d::Zero();
        double elapsed = time - trajectory_start_time_;
        double arc_length = ComputeDesiredArcLength(elapsed);
        double u = ArcLengthToParameter(arc_length);
        return EvaluateBSpline(u);
    }
    
    Eigen::Vector3d EvaluateBSplineAtParameter(double u) const {
        return EvaluateBSpline(u);
    }
    
    std::vector<Eigen::Vector3d> GetControlPoints() const {
        return control_points_;
    }
    
    double GetTotalArcLength() const {
        return total_arc_length_;
    }
    
    double GetTrajectoryDuration() const {
        return trajectory_duration_;
    }
    
    int GetReplanCount() const {
        return replan_count_;
    }
    
    double GetLastReplanTime() const {
        return last_replan_time_;
    }
};

} // namespace ctrl