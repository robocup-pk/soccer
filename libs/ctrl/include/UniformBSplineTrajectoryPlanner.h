#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include <mutex>
#include "RobotDescription.h"
#include "Utils.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

struct PlannerConstants {
    // =====================================================================================
    // Control Point Generation
    // These parameters control how the B-spline control points are generated from the waypoints.
    // Adjust these to change the shape of the trajectory around corners.
    // =====================================================================================

    // The angle (in radians) below which a corner is considered "sharp".
    // Smaller values will make more corners be treated as sharp corners.
    static constexpr double SHARP_CORNER_ANGLE_RAD = M_PI * 0.75; // 135 degrees

    // The angle (in radians) for a 90-degree corner.
    static constexpr double NINETY_DEG_CORNER_ANGLE_RAD = M_PI / 2.0;

    // The tolerance (in radians) for detecting a 90-degree corner.
    static constexpr double NINETY_DEG_CORNER_ANGLE_TOLERANCE_RAD = 0.1;

    // The distance to offset the control points from the waypoint for a 90-degree corner.
    // Increasing this value will make the robot cut the corner more, resulting in a wider turn.
    static constexpr double CORNER_OFFSET_M = 0.025;

    // The factor by which to pull the corner control point inwards.
    // This helps to tighten the turn. A value of 0.5 means the control point is placed halfway
    // between the waypoint and the intersection of the two segments.
    static constexpr double CORNER_INWARD_PULL_FACTOR = 0.5;

    // The distance to offset the control points from the waypoint for a general sharp corner.
    // Increasing this value will make the robot cut the corner more.
    static constexpr double GENERAL_CORNER_OFFSET_M = 0.08;

    // The inward pull factor for a general sharp corner.
    static constexpr double GENERAL_CORNER_INWARD_PULL_FACTOR = 0.4;

    // The distance to offset the control points from the waypoint for a smooth corner.
    // Increasing this value will make the robot cut the corner more.
    static constexpr double SMOOTH_CORNER_OFFSET_M = 0.1;


    // =====================================================================================
    // Boundary Constraints
    // These parameters control how the robot behaves near the field boundaries.
    // =====================================================================================

    // The margin (in meters) from the field boundary to a corner of the field.
    // If the robot is within this margin, the boundary constraints will be applied.
    static constexpr double BOUNDARY_CORNER_MARGIN_M = 0.05;

    // The distance (in meters) to pull the control point towards the center of the field
    // when it is near a corner. This helps to prevent the robot from getting too close to the corner.
    static constexpr double BOUNDARY_CORNER_PULL_M = 0.02;


    // =====================================================================================
    // Trajectory Following
    // These parameters control how the robot follows the trajectory.
    // =====================================================================================

    // The tolerance (in meters) for the position error when the trajectory is finished.
    // If the robot is within this distance of the final target, the trajectory is considered finished.
    static constexpr double POSITION_TOLERANCE_M = 0.02;

    // The tolerance (in radians) for the angle error when the trajectory is finished.
    static constexpr double ANGLE_TOLERANCE_RAD = 0.05;

    // The maximum velocity (in m/s) for the final approach to the target.
    static constexpr double FINAL_APPROACH_MAX_VEL_MS = 0.10;

    // The maximum angular velocity (in rad/s) for the final approach to the target.
    static constexpr double FINAL_APPROACH_MAX_OMEGA_RADS = 0.3;

    // The minimum time step (in seconds) for the update loop.
    static constexpr double MIN_DT_S = 0.001;

    // The step size for numerical differentiation.
    static constexpr double NUMERICAL_DIFF_H = 1e-6;

    // The lookahead time (in seconds) for corner detection.
    // Increasing this value will make the robot detect corners earlier and start turning sooner.
    static constexpr double LOOKAHEAD_TIME_S = 0.15;

    // The curvature threshold for detecting a sharp corner.
    // Higher values will make the corner detection less sensitive.
    static constexpr double SHARP_CORNER_CURVATURE = 1.5;

    // The curvature threshold for detecting a moderate corner.
    static constexpr double MODERATE_CORNER_CURVATURE = 1.0;

    // The factor by which to increase the cross-track gain for a sharp corner.
    // This makes the robot turn more aggressively in sharp corners.
    static constexpr double SHARP_CORNER_GAIN_FACTOR = 1.5;

    // The factor by which to increase the cross-track gain for a moderate corner.
    static constexpr double MODERATE_CORNER_GAIN_FACTOR = 1.2;

    // The factor by which to reduce the speed for a sharp corner.
    static constexpr double SHARP_CORNER_SPEED_FACTOR = 0.5;

    // The factor by which to reduce the speed for a moderate corner.
    static constexpr double MODERATE_CORNER_SPEED_FACTOR = 0.6;

    // The curvature threshold for detecting a very sharp corner.
    static constexpr double VERY_SHARP_CORNER_CURVATURE = 2.5;

    // The compensation distance (in meters) for a very sharp corner.
    // This adds a small perpendicular velocity to help the robot turn.
    static constexpr double VERY_SHARP_CORNER_COMPENSATION_M = 0.02;

    // The lookahead time (in seconds) for heading control.
    static constexpr double HEADING_LOOKAHEAD_S = 0.1;

    // The alpha value for the low-pass filter on the velocity command.
    // A value of 1.0 means no filtering, while a value of 0.0 means full filtering.
    static constexpr double VELOCITY_FILTER_ALPHA = 0.8;


    // =====================================================================================
    // Gain Scheduling
    // These parameters control the feedback gains for the trajectory controller.
    // =====================================================================================

    // The position error threshold (in meters) for using the large error gains.
    static constexpr double LARGE_ERROR_THRESHOLD_M = 0.05;

    // The position error threshold (in meters) for using the medium error gains.
    static constexpr double MEDIUM_ERROR_THRESHOLD_M = 0.02;

    // The base gain for cross-track error.
    // This is the gain used when the error is small.
    static constexpr double BASE_CROSS_TRACK_GAIN = 12.0;

    // The gain for cross-track error when the error is large.
    static constexpr double LARGE_ERROR_CROSS_TRACK_GAIN = 15.0;

    // The gain for cross-track error when the error is medium.
    static constexpr double MEDIUM_ERROR_CROSS_TRACK_GAIN = 12.0;

    // The gain for along-track error.
    static constexpr double BASE_ALONG_TRACK_GAIN = 3.0;

    // The base gain for heading error.
    static constexpr double BASE_HEADING_GAIN = 8.0;

    // The gain for heading error when the error is small.
    static constexpr double SMALL_HEADING_ERROR_GAIN = 5.0;

    // The gain for heading error when the error is large.
    static constexpr double LARGE_HEADING_ERROR_GAIN = 12.0;

    // The heading error threshold (in radians) for using the small error gain.
    static constexpr double SMALL_HEADING_ERROR_RAD = 0.1;

    // The heading error threshold (in radians) for using the large error gain.
    static constexpr double LARGE_HEADING_ERROR_RAD = 0.5;

    // The damping gain for the PD controller.
    static constexpr double DAMPING_GAIN = 0.1;


    // =====================================================================================
    // Arc Length Calculation
    // =====================================================================================

    // The number of samples to use for calculating the arc length of the spline.
    // More samples will give a more accurate result, but will be slower.
    static constexpr int ARC_LENGTH_NUM_SAMPLES = 200;

    // The weight to give to the angular distance when calculating the arc length.
    // This is used to account for the fact that the robot has to rotate as well as translate.
    static constexpr double ARC_LENGTH_ANGULAR_WEIGHT = 0.02;


    // =====================================================================================
    // Closest Point Search
    // =====================================================================================

    // The number of coarse samples to use when searching for the closest point on the spline.
    static constexpr int CLOSEST_POINT_COARSE_SAMPLES = 100;

    // The number of gradient descent iterations to use when refining the closest point.
    static constexpr int CLOSEST_POINT_GD_ITERATIONS = 10;

    // The step size for the gradient descent.
    static constexpr double CLOSEST_POINT_GD_STEP_SIZE = 0.01;

    // The step size for numerical differentiation when calculating the gradient.
    static constexpr double CLOSEST_POINT_GD_H = 0.001;


    // =====================================================================================
    // Replanning
    // =====================================================================================

    // The position error threshold (in meters) for triggering a replan.
    static constexpr double REPLAN_POSITION_THRESHOLD_M = 0.05;

    // The angle error threshold (in radians) for triggering a replan.
    static constexpr double REPLAN_ANGLE_THRESHOLD_RAD = 0.1;

    // The dot product threshold for determining if a waypoint is ahead of the robot.
    static constexpr double REMAINING_PATH_WAYPOINT_DOT_THRESHOLD = 0.1;

    // The minimum distance (in meters) between the robot and a waypoint for it to be included
    // in the remaining path.
    static constexpr double REMAINING_PATH_MIN_DIST_M = 0.05;
};

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

    // Toggle verbose logging (disable for runtime replanning)
    void SetVerbose(bool verbose) { verbose_ = verbose; }
    
    // Check if trajectory is active
    bool IsActive() const { return is_trajectory_active_; }
    
    // Check if trajectory is finished
    bool IsFinished() const { return is_trajectory_finished_; }
    
    // Reset trajectory
    void ResetTrajectory();
    
    // Get ideal position at current time (for ground truth tracking)
    Eigen::Vector3d GetIdealPosition(double current_time) const;
    
    // Replanning functionality - EWOK-style partial replanning
    bool CheckAndReplan(const Eigen::Vector3d& state_estimation_pose, 
                       const Eigen::Vector3d& current_commanded_pose,
                       double current_time,
                       double position_error_threshold = 0.05,  // 5cm default
                       double angle_error_threshold = 0.1);     // 0.1 rad default

    // Lightweight, rate-limited replanning using internal desired pose
    bool TryAutoReplan(const Eigen::Vector3d& state_estimation_pose,
                       double current_time,
                       double position_error_threshold = 0.05,
                       double angle_error_threshold = 0.1);
    
    // Partial trajectory update (EWOK-style)
    bool UpdatePartialTrajectory(const Eigen::Vector3d& current_pose, 
                               int num_control_points_to_update = 4);
    
    // Get remaining waypoints from current position
    // Monotonic: never returns waypoints earlier than the last chosen index
    std::vector<Eigen::Vector3d> GetRemainingPath(const Eigen::Vector3d& current_pose);
    
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

    // Evaluate B-spline with given parameters
    Eigen::Vector3d EvaluateBSpline(double u,
                                    int degree,
                                    const std::vector<Eigen::Vector3d>& control_points,
                                    const std::vector<double>& knot_vector) const;

    // Evaluate B-spline derivative analytically
    Eigen::Vector3d EvaluateBSplineDerivativeAnalytically(double u, int derivative_order) const;
    
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

    // Refactored Update() methods
    Eigen::Vector3d HandleFinishedTrajectory(const Eigen::Vector3d& current_pose);
    
    struct DesiredState {
        Eigen::Vector3d pose;
        Eigen::Vector3d velocity;
    };
    DesiredState CalculateDesiredState(double elapsed_time, const Eigen::Vector3d& current_pose);

    Eigen::Vector3d CalculateVelocityCommand(const Eigen::Vector3d& current_pose,
                                             const DesiredState& desired_state,
                                             double dt, double elapsed_time);

    void ApplyControlGains(Eigen::Vector3d& velocity_command,
                           const Eigen::Vector3d& pose_error,
                           const Eigen::Vector3d& desired_velocity_tangent,
                           double curvature,
                           double desired_speed);

    void ApplyVelocityLimitsAndFilter(Eigen::Vector3d& velocity_command);

    void FindClosestParameterRecursive(const Eigen::Vector2d& position,
                                       double u_min,
                                       double u_max,
                                       double& best_u,
                                       double& min_dist_sq) const;
    
private:
    // Guard planner internal state for thread-safety
    mutable std::mutex planner_mutex_;

    // Robot description
    kin::RobotDescription robot_description_;
    
    // B-spline parameters
    static constexpr int SPLINE_DEGREE = 3;  // Cubic B-spline
    static constexpr int SPLINE_ORDER = SPLINE_DEGREE + 1;
    double dt_ = 0.1;  // Time interval between control points
    
    // Control points and knot vector
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<double> knot_vector_;
    mutable std::vector<std::vector<Eigen::Vector3d>> derivative_control_points_;
    
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
    bool replanning_enabled_ = false;  // Disabled by default to avoid performance issues
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

    // Utility: desired pose at now (world frame)
    Eigen::Vector3d GetDesiredPoseAt(double current_time) const { return GetDesiredPositionAtTime(current_time); }

private:
    // Logging verbosity
    bool verbose_ = false;  // keep quiet by default to avoid UI stalls

    // Progress guard to prevent replanning from jumping back to earlier waypoints
    size_t min_waypoint_index_ = 0;
};

} // namespace ctrl
