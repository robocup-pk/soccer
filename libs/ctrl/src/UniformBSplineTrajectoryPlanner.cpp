#include "UniformBSplineTrajectoryPlanner.h"
#include "RobotDescription.h"
#include "RobotManager.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace ctrl {

UniformBSplineTrajectoryPlanner::UniformBSplineTrajectoryPlanner() 
    : robot_description_(kin::GetRobotDescription()),
      trajectory_start_time_(0.0),
      trajectory_duration_(0.0),
      is_trajectory_active_(false),
      is_trajectory_finished_(true) {
    
    std::cout << "[UniformBSplineTrajectoryPlanner] Initialized with uniform B-spline approach" << std::endl;
}

bool UniformBSplineTrajectoryPlanner::SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    if (path_fWorld.size() < 2) {
        std::cerr << "[UniformBSplineTrajectoryPlanner::SetPath] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[UniformBSplineTrajectoryPlanner::SetPath] Setting path with " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    try {
        // Store original waypoints
        waypoints_ = path_fWorld;
        
        // RoboCup SSL Division B field boundaries (9m x 6m)
        // Field is centered at origin, so boundaries are:
        // X: -4.5m to +4.5m
        // Y: -3.0m to +3.0m
        field_min_x_ = -4.5;
        field_max_x_ = 4.5;
        field_min_y_ = -3.0;
        field_max_y_ = 3.0;
        
        std::cout << "[UniformBSplineTrajectoryPlanner::SetPath] Field boundaries: ["
                  << field_min_x_ << ", " << field_max_x_ << "] x ["
                  << field_min_y_ << ", " << field_max_y_ << "]" << std::endl;
        
        // Generate control points from waypoints
        GenerateControlPointsFromWaypoints();
        
        // Apply boundary constraints to ensure robot stays within field
        ApplyBoundaryConstraints();
        
        // Generate uniform knot vector
        GenerateUniformKnotVector();
        
        // Calculate arc length for velocity planning
        CalculateArcLength();
        
        // Set trajectory timing
        trajectory_start_time_ = (t_start_s > 0) ? t_start_s : util::GetCurrentTime();
        
        // Calculate trajectory duration based on arc length and velocity limits
        double accel_time = v_max_ / a_max_;
        double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
        
        if (2 * accel_distance > total_arc_length_) {
            // Triangular velocity profile
            trajectory_duration_ = 2 * std::sqrt(total_arc_length_ / a_max_);
        } else {
            // Trapezoidal velocity profile
            double cruise_distance = total_arc_length_ - 2 * accel_distance;
            trajectory_duration_ = 2 * accel_time + cruise_distance / v_max_;
        }
        
        is_trajectory_active_ = true;
        is_trajectory_finished_ = false;
        
        std::cout << "[UniformBSplineTrajectoryPlanner::SetPath] Trajectory generated successfully. "
                  << "Control points: " << control_points_.size() 
                  << ", Arc length: " << total_arc_length_ << "m"
                  << ", Duration: " << trajectory_duration_ << "s" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[UniformBSplineTrajectoryPlanner::SetPath] Exception: " << e.what() << std::endl;
        return false;
    }
}

void UniformBSplineTrajectoryPlanner::GenerateControlPointsFromWaypoints() {
    control_points_.clear();
    
    // For uniform B-splines, we need to handle endpoints carefully
    // to ensure the spline passes through start and end points
    
    if (waypoints_.size() == 2) {
        // Simple case: straight line or rotation
        const Eigen::Vector3d& p0 = waypoints_[0];
        const Eigen::Vector3d& p1 = waypoints_[1];
        
        // For cubic B-spline, we need at least 4 control points
        // Repeat endpoints to ensure interpolation
        control_points_.push_back(p0);
        control_points_.push_back(p0);
        control_points_.push_back(p1);
        control_points_.push_back(p1);
        
    } else {
        // Multiple waypoints: handle corners and orientation properly
        
        // First, normalize orientation changes to minimize rotation
        std::vector<Eigen::Vector3d> normalized_waypoints = waypoints_;
        for (size_t i = 1; i < normalized_waypoints.size(); ++i) {
            double dtheta = normalized_waypoints[i][2] - normalized_waypoints[i-1][2];
            // Normalize to [-PI, PI]
            dtheta = NormalizeAngle(dtheta);
            normalized_waypoints[i][2] = normalized_waypoints[i-1][2] + dtheta;
        }
        
        // Start with repeated first point for endpoint interpolation
        control_points_.push_back(normalized_waypoints[0]);
        control_points_.push_back(normalized_waypoints[0]);
        
        // Process intermediate waypoints
        for (size_t i = 1; i < normalized_waypoints.size() - 1; ++i) {
            const Eigen::Vector3d& prev = normalized_waypoints[i-1];
            const Eigen::Vector3d& curr = normalized_waypoints[i];
            const Eigen::Vector3d& next = normalized_waypoints[i+1];
            
            // Calculate corner angle based on position change
            Eigen::Vector2d v1 = curr.head<2>() - prev.head<2>();
            Eigen::Vector2d v2 = next.head<2>() - curr.head<2>();
            
            // Check if we have significant position movement
            if (v1.norm() > 1e-6 && v2.norm() > 1e-6) {
                v1.normalize();
                v2.normalize();
                double angle = std::acos(std::clamp(v1.dot(v2), -1.0, 1.0));
                
                if (angle < M_PI * 0.75) { // Sharp corner (< 135 degrees)
                    // For sharp corners, we need to ensure the spline stays within bounds
                    // Key insight from EWOK: place control points to create a smooth arc inside the corner
                    
                    // Calculate bisector direction (points inward for convex corners)
                    Eigen::Vector2d bisector = -(v1 + v2).normalized();
                    
                    // For 90-degree corners on a square path
                    if (std::abs(angle - M_PI/2) < 0.1) {
                        // Place control points on an arc inside the corner
                        // This ensures the convex hull property keeps the trajectory inside
                        double corner_offset = 0.05; // 5cm inside corner
                        
                        // Before corner
                        Eigen::Vector3d before_corner = curr;
                        before_corner.head<2>() = curr.head<2>() - v1 * corner_offset;
                        control_points_.push_back(before_corner);
                        
                        // Corner point (pulled inward)
                        Eigen::Vector3d corner_point = curr;
                        corner_point.head<2>() = curr.head<2>() + bisector * corner_offset * 0.7;
                        control_points_.push_back(corner_point);
                        
                        // After corner
                        Eigen::Vector3d after_corner = curr;
                        after_corner.head<2>() = curr.head<2>() + v2 * corner_offset;
                        after_corner[2] = curr[2] + 0.5 * NormalizeAngle(next[2] - curr[2]);
                        control_points_.push_back(after_corner);
                        
                    } else {
                        // Other sharp corners
                        control_points_.push_back(curr);
                    }
                } else {
                    // Smooth corner or straight line
                    control_points_.push_back(curr);
                }
            } else {
                // No significant position movement, just add the waypoint
                control_points_.push_back(curr);
            }
        }
        
        // End with repeated last point for endpoint interpolation
        control_points_.push_back(normalized_waypoints.back());
        control_points_.push_back(normalized_waypoints.back());
    }
    
    // Ensure we have enough control points for cubic B-spline
    while (control_points_.size() < SPLINE_ORDER) {
        if (control_points_.size() >= 2) {
            control_points_.push_back(control_points_.back());
        } else {
            control_points_.push_back(Eigen::Vector3d::Zero());
        }
    }
}

void UniformBSplineTrajectoryPlanner::GenerateUniformKnotVector() {
    knot_vector_.clear();
    
    int n = control_points_.size() - 1;  // Number of control points - 1
    int p = SPLINE_DEGREE;              // Degree of B-spline
    int m = n + p + 1;                  // Total number of knots
    
    // Generate uniform knot vector
    // For clamped B-spline (interpolates endpoints):
    // - First p+1 knots are 0
    // - Last p+1 knots are 1
    // - Internal knots are uniformly spaced
    
    // First p+1 knots = 0
    for (int i = 0; i <= p; ++i) {
        knot_vector_.push_back(0.0);
    }
    
    // Internal knots uniformly spaced
    int num_internal = m - 2 * (p + 1);
    for (int i = 1; i <= num_internal; ++i) {
        knot_vector_.push_back(static_cast<double>(i) / (num_internal + 1));
    }
    
    // Last p+1 knots = 1
    for (int i = 0; i <= p; ++i) {
        knot_vector_.push_back(1.0);
    }
}

double UniformBSplineTrajectoryPlanner::BSplineBasis(int i, int p, double u, 
                                                    const std::vector<double>& knot_vector) const {
    // Cox-de Boor recursion formula
    if (p == 0) {
        // Special handling for the last knot span
        if (i == static_cast<int>(control_points_.size()) - 1 && u >= 1.0 - 1e-10) {
            return 1.0;
        }
        return (u >= knot_vector[i] && u < knot_vector[i + 1]) ? 1.0 : 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    double denomLeft = knot_vector[i + p] - knot_vector[i];
    if (denomLeft > 1e-10) {
        left = (u - knot_vector[i]) / denomLeft * BSplineBasis(i, p - 1, u, knot_vector);
    }
    
    double denomRight = knot_vector[i + p + 1] - knot_vector[i + 1];
    if (denomRight > 1e-10) {
        right = (knot_vector[i + p + 1] - u) / denomRight * BSplineBasis(i + 1, p - 1, u, knot_vector);
    }
    
    return left + right;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::EvaluateBSpline(double u) const {
    // Clamp parameter to valid range
    u = std::clamp(u, 0.0, 1.0);
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    // Evaluate using basis functions
    for (size_t i = 0; i < control_points_.size(); ++i) {
        double basis = BSplineBasis(i, SPLINE_DEGREE, u, knot_vector_);
        result += basis * control_points_[i];
    }
    
    return result;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::EvaluateBSplineDerivative(double u, int derivative_order) const {
    // Numerical differentiation with central differences
    const double h = 1e-6;
    
    if (derivative_order == 1) {
        if (u < h) {
            // Forward difference at start
            return (EvaluateBSpline(u + h) - EvaluateBSpline(u)) / h;
        } else if (u > 1.0 - h) {
            // Backward difference at end
            return (EvaluateBSpline(u) - EvaluateBSpline(u - h)) / h;
        } else {
            // Central difference
            Eigen::Vector3d forward = EvaluateBSpline(u + h);
            Eigen::Vector3d backward = EvaluateBSpline(u - h);
            Eigen::Vector3d deriv = (forward - backward) / (2.0 * h);
            
            // Handle angle wrapping for orientation
            deriv[2] = NormalizeAngle(forward[2] - backward[2]) / (2.0 * h);
            return deriv;
        }
    } else if (derivative_order == 2) {
        // Second derivative
        return (EvaluateBSpline(u + h) - 2.0 * EvaluateBSpline(u) + EvaluateBSpline(u - h)) / (h * h);
    }
    
    return Eigen::Vector3d::Zero();
}

void UniformBSplineTrajectoryPlanner::CalculateArcLength() {
    // Calculate arc length using adaptive sampling
    const int num_samples = 200;  // More samples for better accuracy
    total_arc_length_ = 0.0;
    arc_length_samples_.clear();
    parameter_samples_.clear();
    
    arc_length_samples_.push_back(0.0);
    parameter_samples_.push_back(0.0);
    
    Eigen::Vector3d prev_point = EvaluateBSpline(0.0);
    
    for (int i = 1; i <= num_samples; ++i) {
        double u = static_cast<double>(i) / num_samples;
        Eigen::Vector3d current_point = EvaluateBSpline(u);
        
        // Consider both position and orientation changes
        double linear_dist = (current_point.head<2>() - prev_point.head<2>()).norm();
        double angular_dist = std::abs(NormalizeAngle(current_point[2] - prev_point[2]));
        
        // Weight angular distance (robot radius ~0.09m)
        // Reduce weight for angular distance to avoid overestimating arc length
        double segment_length = linear_dist + 0.05 * angular_dist;
        
        total_arc_length_ += segment_length;
        arc_length_samples_.push_back(total_arc_length_);
        parameter_samples_.push_back(u);
        
        prev_point = current_point;
    }
}

double UniformBSplineTrajectoryPlanner::ArcLengthToParameter(double arc_length) const {
    // Binary search for parameter corresponding to arc length
    arc_length = std::clamp(arc_length, 0.0, total_arc_length_);
    
    // Find the segment
    auto it = std::lower_bound(arc_length_samples_.begin(), arc_length_samples_.end(), arc_length);
    if (it == arc_length_samples_.end()) {
        return 1.0;
    }
    
    size_t idx = std::distance(arc_length_samples_.begin(), it);
    if (idx == 0) {
        return 0.0;
    }
    
    // Linear interpolation
    double t = (arc_length - arc_length_samples_[idx - 1]) / 
               (arc_length_samples_[idx] - arc_length_samples_[idx - 1]);
    
    return parameter_samples_[idx - 1] + t * (parameter_samples_[idx] - parameter_samples_[idx - 1]);
}

double UniformBSplineTrajectoryPlanner::ComputeDesiredArcLength(double elapsed_time) const {
    // Trapezoidal velocity profile for smooth acceleration/deceleration
    double accel_time = v_max_ / a_max_;
    double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
    
    if (2 * accel_distance > total_arc_length_) {
        // Triangular profile (never reaches max velocity)
        double peak_time = std::sqrt(total_arc_length_ / a_max_);
        if (elapsed_time < peak_time) {
            return 0.5 * a_max_ * elapsed_time * elapsed_time;
        } else if (elapsed_time < 2 * peak_time) {
            double t_decel = elapsed_time - peak_time;
            return total_arc_length_ - 0.5 * a_max_ * (2 * peak_time - elapsed_time) * (2 * peak_time - elapsed_time);
        } else {
            return total_arc_length_;
        }
    } else {
        // Trapezoidal profile
        double cruise_distance = total_arc_length_ - 2 * accel_distance;
        double cruise_time = cruise_distance / v_max_;
        
        if (elapsed_time < accel_time) {
            // Acceleration phase
            return 0.5 * a_max_ * elapsed_time * elapsed_time;
        } else if (elapsed_time < accel_time + cruise_time) {
            // Cruise phase
            return accel_distance + v_max_ * (elapsed_time - accel_time);
        } else if (elapsed_time < 2 * accel_time + cruise_time) {
            // Deceleration phase
            double t_total = 2 * accel_time + cruise_time;
            double remaining = t_total - elapsed_time;
            return total_arc_length_ - 0.5 * a_max_ * remaining * remaining;
        } else {
            return total_arc_length_;
        }
    }
}

double UniformBSplineTrajectoryPlanner::ComputeDesiredSpeed(double elapsed_time) const {
    // Velocity from trapezoidal profile
    double accel_time = v_max_ / a_max_;
    double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
    
    if (2 * accel_distance > total_arc_length_) {
        // Triangular profile
        double peak_time = std::sqrt(total_arc_length_ / a_max_);
        if (elapsed_time < peak_time) {
            return a_max_ * elapsed_time;
        } else if (elapsed_time < 2 * peak_time) {
            return a_max_ * (2 * peak_time - elapsed_time);
        } else {
            return 0.0;
        }
    } else {
        // Trapezoidal profile
        double cruise_time = (total_arc_length_ - 2 * accel_distance) / v_max_;
        
        if (elapsed_time < accel_time) {
            return a_max_ * elapsed_time;
        } else if (elapsed_time < accel_time + cruise_time) {
            return v_max_;
        } else if (elapsed_time < 2 * accel_time + cruise_time) {
            return v_max_ - a_max_ * (elapsed_time - accel_time - cruise_time);
        } else {
            return 0.0;
        }
    }
}

void UniformBSplineTrajectoryPlanner::ApplyBoundaryConstraints() {
    // Ensure all control points stay within field boundaries
    // This leverages the convex hull property of B-splines
    
    for (auto& cp : control_points_) {
        cp[0] = std::clamp(cp[0], field_min_x_ + boundary_margin_, field_max_x_ - boundary_margin_);
        cp[1] = std::clamp(cp[1], field_min_y_ + boundary_margin_, field_max_y_ - boundary_margin_);
        // Note: Don't clamp orientation
    }
    
    // Additional check: ensure corner control points are pulled inward enough
    // This is critical for achieving <0.05% error at boundaries
    for (size_t i = 2; i < control_points_.size() - 2; ++i) {
        // Check if this might be a corner control point
        const Eigen::Vector3d& prev = control_points_[i-1];
        const Eigen::Vector3d& curr = control_points_[i];
        const Eigen::Vector3d& next = control_points_[i+1];
        
        // If near a corner of the field, pull it further inward
        bool near_corner = false;
        double corner_margin = 0.05; // 5cm from corner
        
        if ((std::abs(curr[0] - field_min_x_) < corner_margin || 
             std::abs(curr[0] - field_max_x_) < corner_margin) &&
            (std::abs(curr[1] - field_min_y_) < corner_margin || 
             std::abs(curr[1] - field_max_y_) < corner_margin)) {
            near_corner = true;
        }
        
        if (near_corner) {
            // Pull control point toward center
            Eigen::Vector2d to_center = Eigen::Vector2d(0.5, 0.5) - curr.head<2>();
            to_center.normalize();
            control_points_[i].head<2>() += to_center * 0.02; // Pull 2cm toward center
        }
    }
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    
    // Check if trajectory is finished
    if (elapsed_time >= trajectory_duration_) {
        is_trajectory_finished_ = true;
        is_trajectory_active_ = false;
        return Eigen::Vector3d::Zero();
    }
    
    // Compute desired position along trajectory
    double desired_arc_length = ComputeDesiredArcLength(elapsed_time);
    double u = ArcLengthToParameter(desired_arc_length);
    
    // Get desired pose and velocity
    Eigen::Vector3d desired_pose = EvaluateBSpline(u);
    Eigen::Vector3d spline_velocity = EvaluateBSplineDerivative(u, 1);
    
    // Compute tracking error
    Eigen::Vector3d pose_error = desired_pose - current_pose;
    pose_error[2] = NormalizeAngle(pose_error[2]);
    
    // Feedforward + feedback control
    double current_speed = ComputeDesiredSpeed(elapsed_time);
    Eigen::Vector3d velocity_command;
    
    if (spline_velocity.head<2>().norm() > 1e-6) {
        // Scale velocity to desired speed
        double scale = current_speed / spline_velocity.head<2>().norm();
        velocity_command = scale * spline_velocity + kp_ * pose_error;
    } else {
        // Pure rotation
        velocity_command = Eigen::Vector3d::Zero();
        velocity_command[2] = kp_ * pose_error[2];
    }
    
    // Apply velocity limits
    velocity_command[0] = std::clamp(velocity_command[0], -v_max_, v_max_);
    velocity_command[1] = std::clamp(velocity_command[1], -v_max_, v_max_);
    velocity_command[2] = std::clamp(velocity_command[2], -omega_max_, omega_max_);
    
    return velocity_command;
}

bool UniformBSplineTrajectoryPlanner::AddGoal(const Eigen::Vector3d& goal) {
    // Dynamic replanning: add new goal and regenerate trajectory
    if (!is_trajectory_active_) {
        return false;
    }
    
    // Get current position on trajectory
    Eigen::Vector3d current_pos = GetCurrentDesiredPosition();
    
    // Create new path from current position to new goal
    std::vector<Eigen::Vector3d> new_path;
    new_path.push_back(current_pos);
    new_path.push_back(goal);
    
    // Reset and regenerate trajectory
    return SetPath(new_path, util::GetCurrentTime());
}

double UniformBSplineTrajectoryPlanner::GetRemainingTime() const {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return 0.0;
    }
    
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    return std::max(0.0, trajectory_duration_ - elapsed);
}

void UniformBSplineTrajectoryPlanner::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
}

void UniformBSplineTrajectoryPlanner::SetFeedbackGains(double kp, double kd) {
    kp_ = kp;
    kd_ = kd;
}

void UniformBSplineTrajectoryPlanner::SetSplineDegree(int degree) {
    // For this implementation, we keep it fixed at cubic (degree 3)
    // as it provides C2 continuity which is ideal for robot motion
    if (degree != 3) {
        std::cerr << "[UniformBSplineTrajectoryPlanner] Warning: Only cubic B-splines (degree 3) are supported" << std::endl;
    }
}

void UniformBSplineTrajectoryPlanner::SetTimeInterval(double dt) {
    dt_ = dt;
}

void UniformBSplineTrajectoryPlanner::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    // Can be used to get current robot state if needed
}

double UniformBSplineTrajectoryPlanner::NormalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double UniformBSplineTrajectoryPlanner::ParameterToTime(double u) const {
    return u * trajectory_duration_;
}

void UniformBSplineTrajectoryPlanner::ResetTrajectory() {
    is_trajectory_active_ = false;
    is_trajectory_finished_ = true;
    trajectory_start_time_ = 0.0;
    trajectory_duration_ = 0.0;
    control_points_.clear();
    knot_vector_.clear();
    arc_length_samples_.clear();
    parameter_samples_.clear();
    waypoints_.clear();
}

bool UniformBSplineTrajectoryPlanner::CheckAndReplan(const Eigen::Vector3d& state_estimation_pose, 
                                                    const Eigen::Vector3d& current_commanded_pose,
                                                    double current_time,
                                                    double position_error_threshold,
                                                    double angle_error_threshold) {
    if (!replanning_enabled_ || !is_trajectory_active_ || is_trajectory_finished_) {
        return false;
    }
    
    // Don't replan too frequently
    if (current_time - last_replan_time_ < min_replan_interval_) {
        return false;
    }
    
    // Calculate position error between state estimation and commanded position
    double position_error = (state_estimation_pose.head<2>() - current_commanded_pose.head<2>()).norm();
    double angle_error = std::abs(NormalizeAngle(state_estimation_pose[2] - current_commanded_pose[2]));
    
    // Check if error exceeds threshold
    if (position_error > position_error_threshold || angle_error > angle_error_threshold) {
        std::cout << "[UniformBSplineTrajectoryPlanner] Replanning triggered! "
                  << "Position error: " << position_error << "m (threshold: " << position_error_threshold << "m), "
                  << "Angle error: " << angle_error << " rad (threshold: " << angle_error_threshold << " rad)"
                  << std::endl;
        
        // Get remaining path from state estimation position
        std::vector<Eigen::Vector3d> remaining_path = GetRemainingPath(state_estimation_pose);
        
        if (remaining_path.size() < 2) {
            std::cout << "[UniformBSplineTrajectoryPlanner] Not enough waypoints remaining for replanning" << std::endl;
            return false;
        }
        
        // Replan from current state estimation position
        bool success = SetPath(remaining_path, current_time);
        
        if (success) {
            replan_count_++;
            last_replan_time_ = current_time;
            std::cout << "[UniformBSplineTrajectoryPlanner] Replanning successful. Total replans: " << replan_count_ << std::endl;
        } else {
            std::cout << "[UniformBSplineTrajectoryPlanner] Replanning failed!" << std::endl;
        }
        
        return success;
    }
    
    return false;
}

std::vector<Eigen::Vector3d> UniformBSplineTrajectoryPlanner::GetRemainingPath(const Eigen::Vector3d& current_pose) const {
    std::vector<Eigen::Vector3d> remaining_path;
    
    // Always start from current position
    remaining_path.push_back(current_pose);
    
    if (waypoints_.empty()) {
        return remaining_path;
    }
    
    // Find closest waypoint ahead
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    // Get current progress along trajectory
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    double progress = elapsed / trajectory_duration_;
    
    // Estimate which waypoint we should be near based on progress
    size_t expected_idx = static_cast<size_t>(progress * (waypoints_.size() - 1));
    
    // Search for closest waypoint starting from expected position
    for (size_t i = std::max(size_t(0), expected_idx); i < waypoints_.size(); ++i) {
        double dist = (waypoints_[i].head<2>() - current_pose.head<2>()).norm();
        
        // Check if this waypoint is ahead of us (not behind)
        if (i < waypoints_.size() - 1) {
            Eigen::Vector2d to_waypoint = waypoints_[i].head<2>() - current_pose.head<2>();
            Eigen::Vector2d to_next = waypoints_[i+1].head<2>() - waypoints_[i].head<2>();
            
            // If dot product is positive, waypoint is generally in forward direction
            if (to_waypoint.dot(to_next) >= 0 || dist < 0.1) {  // Within 10cm or ahead
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_idx = i;
                }
            }
        }
    }
    
    // Add remaining waypoints
    for (size_t i = closest_idx; i < waypoints_.size(); ++i) {
        // Skip waypoints that are too close to current position (avoid loops)
        if (i == closest_idx && min_distance < 0.05) {  // Within 5cm
            continue;
        }
        remaining_path.push_back(waypoints_[i]);
    }
    
    // Ensure we have at least the final goal
    if (remaining_path.size() == 1 && waypoints_.size() > 0) {
        remaining_path.push_back(waypoints_.back());
    }
    
    return remaining_path;
}

} // namespace ctrl