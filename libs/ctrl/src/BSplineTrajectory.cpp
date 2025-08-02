#include "BSplineTrajectory.h"
#include "RobotDescription.h"
#include "RobotManager.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace ctrl {

BSplineTrajectory::BSplineTrajectory() 
    : robot_description_(kin::GetRobotDescription()),
      trajectory_start_time_(0.0),
      is_trajectory_active_(false),
      is_trajectory_finished_(true),
      current_segment_progress_(0.0),
      spline_degree_(3) {  // Cubic B-spline for C2 continuity
    
    std::cout << "[BSplineTrajectory] Initialized with " 
              << robot_description_.wheel_angles_rad.size() << " wheels" << std::endl;
}

bool BSplineTrajectory::SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    if (path_fWorld.size() < 2) {
        std::cerr << "[BSplineTrajectory::SetPath] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[BSplineTrajectory::SetPath] Generating B-spline trajectory for " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    try {
        // Store original waypoints
        waypoints_ = path_fWorld;
        
        // Check if this is a pure rotation trajectory (all positions are the same)
        bool is_pure_rotation = true;
        const double position_tolerance = 0.001;  // 1mm tolerance
        for (size_t i = 1; i < path_fWorld.size(); ++i) {
            if ((path_fWorld[i].head<2>() - path_fWorld[0].head<2>()).norm() > position_tolerance) {
                is_pure_rotation = false;
                break;
            }
        }
        
        // Generate B-spline control points
        GenerateBSplineControlPoints();
        
        // Generate knot vector
        GenerateKnotVector();
        
        // Calculate total arc length for velocity planning
        CalculateArcLength();
        
        // For pure rotation, ensure arc length is based on angular distance
        if (is_pure_rotation && total_arc_length_ < 0.1) {
            // Calculate total angular distance
            double total_angle_change = std::abs(path_fWorld.back()[2] - path_fWorld.front()[2]);
            total_arc_length_ = total_angle_change * 0.09;  // Convert to equivalent linear distance
            std::cout << "  Pure rotation detected. Angular distance: " << total_angle_change 
                      << " rad, equivalent arc length: " << total_arc_length_ << "m" << std::endl;
        }
        
        trajectory_start_time_ = (t_start_s > 0) ? t_start_s : util::GetCurrentTime();
        is_trajectory_active_ = true;
        is_trajectory_finished_ = false;
        current_segment_progress_ = 0.0;
        
        // Debug: Check start and end positions
        Eigen::Vector3d start_pos = EvaluateBSpline(0.0);
        Eigen::Vector3d end_pos = EvaluateBSpline(1.0);
        
        std::cout << "[BSplineTrajectory::SetPath] B-spline trajectory generated successfully. "
                  << "Control points: " << control_points_.size() 
                  << ", Arc length: " << total_arc_length_ << "m" << std::endl;
        std::cout << "  Start: (" << start_pos[0] << ", " << start_pos[1] << ", " << start_pos[2] << " rad)" << std::endl;
        std::cout << "  End: (" << end_pos[0] << ", " << end_pos[1] << ", " << end_pos[2] << " rad = " 
                  << end_pos[2] * 180.0 / M_PI << " deg)" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[BSplineTrajectory::SetPath] Exception: " << e.what() << std::endl;
        return false;
    }
}

double BSplineTrajectory::CalculateAngleBetween(const Eigen::Vector3d& p1, 
                                               const Eigen::Vector3d& p2, 
                                               const Eigen::Vector3d& p3) const {
    // Calculate angle between vectors (p1-p2) and (p3-p2)
    Eigen::Vector2d v1 = p1.head<2>() - p2.head<2>();
    Eigen::Vector2d v2 = p3.head<2>() - p2.head<2>();
    
    double norm_v1 = v1.norm();
    double norm_v2 = v2.norm();
    
    if (norm_v1 < 1e-6 || norm_v2 < 1e-6) {
        return 0.0;  // Treat as sharpest possible corner
    }
    
    double cos_angle = v1.dot(v2) / (norm_v1 * norm_v2);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    
    return std::acos(cos_angle);
}

std::vector<Eigen::Vector3d> BSplineTrajectory::SmartRepetition(
    const std::vector<Eigen::Vector3d>& points, 
    int min_repeats, 
    int max_repeats) const {
    
    std::vector<Eigen::Vector3d> new_points;
    
    // Start with the first point
    new_points.push_back(points[0]);
    
    // Process intermediate points
    for (size_t i = 1; i < points.size() - 1; ++i) {
        const Eigen::Vector3d& p_prev = points[i - 1];
        const Eigen::Vector3d& p_curr = points[i];
        const Eigen::Vector3d& p_next = points[i + 1];
        
        // Calculate angle at this corner
        double angle = CalculateAngleBetween(p_prev, p_curr, p_next);
        
        // Linear mapping: sharp corner (0 rad) = max_repeats, straight (pi rad) = min_repeats
        double weight = 1.0 - angle / M_PI;
        weight = std::clamp(weight, 0.0, 1.0);
        
        int repeats = static_cast<int>(std::round(min_repeats + weight * (max_repeats - min_repeats)));
        
        // Add repeated points
        for (int j = 0; j < repeats; ++j) {
            new_points.push_back(p_curr);
        }
    }
    
    // Add the last point
    new_points.push_back(points.back());
    
    return new_points;
}

std::vector<Eigen::Vector3d> BSplineTrajectory::LinearEntryExitClosedLoop(
    const std::vector<Eigen::Vector3d>& points, 
    int repeats, 
    double tolerance) const {
    
    if (points.empty()) return points;
    
    const Eigen::Vector3d& start = points.front();
    const Eigen::Vector3d& end = points.back();
    
    // Check if this is a closed loop
    if ((start.head<2>() - end.head<2>()).norm() < tolerance) {
        std::vector<Eigen::Vector3d> new_points;
        
        // Add repeated start points for linear entry
        for (int i = 0; i < repeats; ++i) {
            new_points.push_back(start);
        }
        
        // Add all points except the last one (to avoid duplication)
        for (size_t i = 0; i < points.size() - 1; ++i) {
            new_points.push_back(points[i]);
        }
        
        // Add repeated end points for linear exit
        for (int i = 0; i < repeats; ++i) {
            new_points.push_back(end);
        }
        
        return new_points;
    }
    
    return points;
}

std::vector<Eigen::Vector3d> BSplineTrajectory::InsertCornerControlPoints(
    const std::vector<Eigen::Vector3d>& waypoints,
    double corner_offset) const {
    
    if (waypoints.size() < 3) {
        return waypoints;
    }
    
    std::vector<Eigen::Vector3d> control_points;
    
    // Add the first waypoint
    control_points.push_back(waypoints[0]);
    
    // Process interior waypoints
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        const Eigen::Vector3d& prev = waypoints[i - 1];
        const Eigen::Vector3d& curr = waypoints[i];
        const Eigen::Vector3d& next = waypoints[i + 1];
        
        // Calculate direction vectors
        Eigen::Vector2d v1 = (curr.head<2>() - prev.head<2>());
        Eigen::Vector2d v2 = (next.head<2>() - curr.head<2>());
        
        double len1 = v1.norm();
        double len2 = v2.norm();
        
        if (len1 > 2 * corner_offset && len2 > 2 * corner_offset) {
            // Normalize directions
            v1 /= len1;
            v2 /= len2;
            
            // Calculate angle between segments
            double angle = CalculateAngleBetween(prev, curr, next);
            
            // If sharp corner (angle < 120 degrees), add control points before and after
            if (angle < 2.0 * M_PI / 3.0) {
                // Add control point before corner
                Eigen::Vector3d before = curr;
                before.head<2>() = curr.head<2>() - v1 * corner_offset;
                control_points.push_back(before);
                
                // Add the corner point itself
                control_points.push_back(curr);
                
                // Add control point after corner
                Eigen::Vector3d after = curr;
                after.head<2>() = curr.head<2>() + v2 * corner_offset;
                control_points.push_back(after);
            } else {
                // For smooth curves, just add the waypoint
                control_points.push_back(curr);
            }
        } else {
            // Not enough space for corner handling
            control_points.push_back(curr);
        }
    }
    
    // Add the last waypoint
    control_points.push_back(waypoints.back());
    
    return control_points;
}

void BSplineTrajectory::GenerateBSplineControlPoints() {
    control_points_.clear();
    
    if (waypoints_.size() == 2) {
        // For only 2 waypoints, add intermediate control points
        Eigen::Vector3d p0 = waypoints_[0];
        Eigen::Vector3d p1 = waypoints_[1];
        
        // Check if this is primarily a rotation (small position change, large angle change)
        double pos_change = (p1.head<2>() - p0.head<2>()).norm();
        double angle_change = std::abs(NormalizeAngle(p1[2] - p0[2]));
        
        if (pos_change < 0.1 && angle_change > M_PI/2) {
            // For large rotations, we need to overshoot the control points
            // to compensate for B-spline's smoothing effect
            
            // Calculate the overshoot factor based on the angle change
            // For 180 degrees, we need about 1.33x overshoot to achieve full rotation
            double overshoot_factor = 1.0;
            if (angle_change >= M_PI) {
                overshoot_factor = 1.33;  // 33% overshoot for 180 degrees
            } else if (angle_change >= 3*M_PI/4) {
                overshoot_factor = 1.25;  // 25% overshoot for 135 degrees
            } else if (angle_change >= M_PI/2) {
                overshoot_factor = 1.15;  // 15% overshoot for 90 degrees
            }
            
            // Create control points with proper angle interpolation
            control_points_.push_back(p0);
            
            // Add intermediate control points with overshoot
            for (double t = 0.25; t <= 0.75; t += 0.25) {
                Eigen::Vector3d cp;
                cp.head<2>() = p0.head<2>() + t * (p1.head<2>() - p0.head<2>());
                
                // Interpolate angle with overshoot
                double angle_diff = p1[2] - p0[2];
                // Normalize to [-pi, pi] for proper interpolation
                while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
                
                // Apply overshoot to intermediate control points
                cp[2] = p0[2] + t * angle_diff * overshoot_factor;
                
                control_points_.push_back(cp);
            }
            
            // Final control point - use exact target angle
            control_points_.push_back(p1);
        } else {
            // For normal trajectories, use original method
            control_points_.push_back(p0);
            control_points_.push_back(p0 + 0.25 * (p1 - p0));
            control_points_.push_back(p0 + 0.5 * (p1 - p0));
            control_points_.push_back(p0 + 0.75 * (p1 - p0));
            control_points_.push_back(p1);
        }
    } else {
        // For multiple waypoints, add control points to prevent overshooting at corners
        // This approach adds "pull-in" points before sharp corners
        
        // Add first point multiple times for clamped B-spline
        for (int i = 0; i <= spline_degree_; ++i) {
            control_points_.push_back(waypoints_[0]);
        }
        
        // Process waypoints with corner constraint points
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            if (i > 0 && i < waypoints_.size() - 1) {
                const Eigen::Vector3d& prev = waypoints_[i - 1];
                const Eigen::Vector3d& curr = waypoints_[i];
                const Eigen::Vector3d& next = waypoints_[i + 1];
                
                // Calculate angle at corner
                double angle = CalculateAngleBetween(prev, curr, next);
                
                // For sharp corners, add multiple constraint points to maintain shape
                if (angle < M_PI * 0.75) {  // Sharp corner (< 135 degrees)
                    // Direction vectors
                    Eigen::Vector2d dir_in = (curr.head<2>() - prev.head<2>()).normalized();
                    Eigen::Vector2d dir_out = (next.head<2>() - curr.head<2>()).normalized();
                    
                    // Add points to create tighter corner control
                    // Point 10cm before corner
                    Eigen::Vector3d before = curr;
                    before.head<2>() = curr.head<2>() - dir_in * 0.10;
                    control_points_.push_back(before);
                    
                    // Point 3cm before corner (closer)
                    Eigen::Vector3d close_before = curr;
                    close_before.head<2>() = curr.head<2>() - dir_in * 0.03;
                    control_points_.push_back(close_before);
                    
                    // Add the corner point twice for more weight
                    control_points_.push_back(curr);
                    control_points_.push_back(curr);
                    
                    // Point 3cm after corner
                    Eigen::Vector3d close_after = curr;
                    close_after.head<2>() = curr.head<2>() + dir_out * 0.03;
                    control_points_.push_back(close_after);
                    
                    // Point 10cm after corner
                    Eigen::Vector3d after = curr;
                    after.head<2>() = curr.head<2>() + dir_out * 0.10;
                    control_points_.push_back(after);
                } else {
                    // For smoother corners, just add the waypoint
                    control_points_.push_back(waypoints_[i]);
                }
            } else if (i == 0 || i == waypoints_.size() - 1) {
                // Skip first and last (already added for clamping)
                continue;
            }
        }
        
        // Add last point multiple times for clamped B-spline
        for (int i = 0; i <= spline_degree_; ++i) {
            control_points_.push_back(waypoints_.back());
        }
        
        std::cout << "  B-spline control points with corner constraints: " << waypoints_.size() 
                  << " waypoints -> " << control_points_.size() << " control points" << std::endl;
    }
}

void BSplineTrajectory::GenerateKnotVector() {
    knot_vector_.clear();
    int n = control_points_.size() - 1;  // Number of control points - 1
    int p = spline_degree_;
    int m = n + p + 1;  // Number of knots
    
    // Generate clamped (open) B-spline knot vector
    // This ensures the curve passes through the first and last control points
    
    // Add p+1 zeros at the beginning
    for (int i = 0; i <= p; ++i) {
        knot_vector_.push_back(0.0);
    }
    
    // Add internal knots with uniform spacing
    int internal_knots = m - 2 * (p + 1);
    for (int i = 1; i <= internal_knots; ++i) {
        knot_vector_.push_back(static_cast<double>(i) / (internal_knots + 1));
    }
    
    // Add p+1 ones at the end
    for (int i = 0; i <= p; ++i) {
        knot_vector_.push_back(1.0);
    }
}

double BSplineTrajectory::BSplineBasis(int i, int p, double u) const {
    // Cox-de Boor recursion formula for B-spline basis functions
    
    // Bounds check
    if (i < 0 || i + p + 1 >= static_cast<int>(knot_vector_.size())) {
        return 0.0;
    }
    
    if (p == 0) {
        // Special handling for the last knot span
        if (i == static_cast<int>(knot_vector_.size()) - 2 && u >= knot_vector_[i] && u <= knot_vector_[i + 1]) {
            return 1.0;
        }
        return (u >= knot_vector_[i] && u < knot_vector_[i + 1]) ? 1.0 : 0.0;
    }
    
    double left_term = 0.0;
    double right_term = 0.0;
    
    double left_denom = knot_vector_[i + p] - knot_vector_[i];
    if (left_denom > 1e-10) {
        left_term = ((u - knot_vector_[i]) / left_denom) * BSplineBasis(i, p - 1, u);
    }
    
    double right_denom = knot_vector_[i + p + 1] - knot_vector_[i + 1];
    if (right_denom > 1e-10) {
        right_term = ((knot_vector_[i + p + 1] - u) / right_denom) * BSplineBasis(i + 1, p - 1, u);
    }
    
    return left_term + right_term;
}

Eigen::Vector3d BSplineTrajectory::EvaluateBSpline(double u) const {
    // Clamp u to valid range
    u = std::clamp(u, 0.0, 1.0);
    
    // Handle edge case at u = 1.0
    if (u >= 1.0 - 1e-10) {
        u = 1.0 - 1e-10;
    }
    
    // Sanity check
    if (control_points_.empty() || knot_vector_.empty()) {
        std::cerr << "[BSplineTrajectory::EvaluateBSpline] Error: No control points or knot vector" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    double basis_sum = 0.0;
    
    // Only evaluate basis functions for valid indices
    int n = control_points_.size() - 1;
    for (int i = 0; i <= n; ++i) {
        double basis = BSplineBasis(i, spline_degree_, u);
        if (!std::isnan(basis) && !std::isinf(basis)) {
            result += basis * control_points_[i];
            basis_sum += basis;
        }
    }
    
    // Normalize if basis functions don't sum to 1 (can happen due to numerical issues)
    if (basis_sum > 1e-10 && std::abs(basis_sum - 1.0) > 1e-6) {
        result /= basis_sum;
    }
    
    // Check for NaN and return last valid position if found
    if (result.hasNaN()) {
        std::cerr << "[BSplineTrajectory::EvaluateBSpline] Warning: NaN detected at u=" << u << std::endl;
        if (!control_points_.empty()) {
            return control_points_.back();
        }
        return Eigen::Vector3d::Zero();
    }
    
    return result;
}

Eigen::Vector3d BSplineTrajectory::EvaluateBSplineDerivative(double u, int derivative_order) const {
    // Numerical differentiation for B-spline derivatives
    const double h = 1e-6;  // Small step for numerical differentiation
    
    if (derivative_order == 1) {
        // First derivative (velocity)
        Eigen::Vector3d p_plus = EvaluateBSpline(u + h);
        Eigen::Vector3d p_minus = EvaluateBSpline(u - h);
        Eigen::Vector3d derivative = (p_plus - p_minus) / (2.0 * h);
        
        // Handle angle wrapping for angular velocity
        derivative[2] = NormalizeAngle(p_plus[2] - p_minus[2]) / (2.0 * h);
        
        return derivative;
    } else if (derivative_order == 2) {
        // Second derivative (acceleration)
        Eigen::Vector3d p_plus = EvaluateBSpline(u + h);
        Eigen::Vector3d p_center = EvaluateBSpline(u);
        Eigen::Vector3d p_minus = EvaluateBSpline(u - h);
        return (p_plus - 2.0 * p_center + p_minus) / (h * h);
    }
    
    return Eigen::Vector3d::Zero();
}

void BSplineTrajectory::CalculateArcLength() {
    // Calculate arc length using numerical integration
    // Use more samples for accurate arc length calculation, especially for rotation-heavy trajectories
    const int num_samples = 500;
    total_arc_length_ = 0.0;
    arc_length_samples_.clear();
    arc_length_samples_.push_back(0.0);
    
    Eigen::Vector3d prev_point = EvaluateBSpline(0.0);
    
    for (int i = 1; i <= num_samples; ++i) {
        double u = static_cast<double>(i) / num_samples;
        Eigen::Vector3d current_point = EvaluateBSpline(u);
        
        // Consider both linear and angular distance
        double linear_distance = (current_point.head<2>() - prev_point.head<2>()).norm();
        double angular_distance = std::abs(NormalizeAngle(current_point[2] - prev_point[2]));
        
        // Weight angular distance to make it comparable to linear distance
        // Using robot radius of ~0.09m, so full rotation = 2*pi*0.09 ≈ 0.565m
        double angular_weight = 0.09;  // Convert radians to equivalent linear distance
        double segment_length = linear_distance + angular_weight * angular_distance;
        total_arc_length_ += segment_length;
        arc_length_samples_.push_back(total_arc_length_);
        
        prev_point = current_point;
    }
}

double BSplineTrajectory::ArcLengthToParameter(double arc_length) const {
    // Convert arc length to parameter u using linear interpolation
    arc_length = std::clamp(arc_length, 0.0, total_arc_length_);
    
    // Find the segment containing this arc length
    for (size_t i = 1; i < arc_length_samples_.size(); ++i) {
        if (arc_length <= arc_length_samples_[i]) {
            // Linear interpolation
            double t = (arc_length - arc_length_samples_[i-1]) / 
                      (arc_length_samples_[i] - arc_length_samples_[i-1]);
            double u_start = static_cast<double>(i-1) / (arc_length_samples_.size() - 1);
            double u_end = static_cast<double>(i) / (arc_length_samples_.size() - 1);
            return u_start + t * (u_end - u_start);
        }
    }
    
    return 1.0;
}

Eigen::Vector3d BSplineTrajectory::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    
    // Use trapezoidal velocity profile for smooth motion
    double desired_arc_length = ComputeDesiredArcLength(elapsed_time);
    
    // Check if trajectory is finished
    if (desired_arc_length >= total_arc_length_) {
        is_trajectory_finished_ = true;
        is_trajectory_active_ = false;
        std::cout << "[BSplineTrajectory::Update] Trajectory execution completed at t=" 
                  << elapsed_time << "s, arc=" << desired_arc_length << "/" << total_arc_length_ << "m" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    // Convert arc length to parameter
    double u = ArcLengthToParameter(desired_arc_length);
    
    // Evaluate B-spline and its derivatives
    Eigen::Vector3d desired_position = EvaluateBSpline(u);
    Eigen::Vector3d spline_velocity = EvaluateBSplineDerivative(u, 1);
    
    // Scale velocity by desired speed
    double current_speed = ComputeDesiredSpeed(elapsed_time);
    
    // Handle both translation and rotation
    double linear_vel_norm = spline_velocity.head<2>().norm();
    double angular_vel = spline_velocity[2];
    
    Eigen::Vector3d feedforward_velocity;
    if (linear_vel_norm > 1e-6) {
        // Normal case: scale linear velocity
        double scale = current_speed / linear_vel_norm;
        feedforward_velocity[0] = spline_velocity[0] * scale;
        feedforward_velocity[1] = spline_velocity[1] * scale;
        feedforward_velocity[2] = spline_velocity[2] * scale;
    } else {
        // Pure rotation case
        feedforward_velocity[0] = 0.0;
        feedforward_velocity[1] = 0.0;
        // For pure rotation, use the angular velocity directly scaled by speed ratio
        // Don't convert linear speed to angular - just use proportional scaling
        double speed_ratio = current_speed / v_max_;  // Ratio of current to max speed
        feedforward_velocity[2] = angular_vel * speed_ratio;
    }
    
    // Apply feedback control for trajectory tracking
    Eigen::Vector3d position_error = desired_position - current_pose;
    position_error[2] = NormalizeAngle(position_error[2]);
    
    // PD controller with feedforward
    // For pure rotation, increase angular gain
    Eigen::Vector3d gains(kp_, kp_, kp_ * 2.0);  // Double gain for angular control
    Eigen::Vector3d velocity_command = feedforward_velocity + gains.cwiseProduct(position_error);
    
    // Apply velocity limits
    velocity_command[0] = std::clamp(velocity_command[0], -v_max_, v_max_);
    velocity_command[1] = std::clamp(velocity_command[1], -v_max_, v_max_);
    velocity_command[2] = std::clamp(velocity_command[2], -omega_max_, omega_max_);
    
    return velocity_command;
}

double BSplineTrajectory::ComputeDesiredArcLength(double elapsed_time) const {
    // Trapezoidal velocity profile for smooth acceleration and deceleration
    double accel_time = v_max_ / a_max_;
    double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
    
    // Check if we need to use triangular profile (short distance)
    if (2 * accel_distance > total_arc_length_) {
        // Triangular profile
        double peak_time = std::sqrt(total_arc_length_ / a_max_);
        if (elapsed_time < peak_time) {
            // Acceleration phase
            return 0.5 * a_max_ * elapsed_time * elapsed_time;
        } else if (elapsed_time < 2 * peak_time) {
            // Deceleration phase
            double t_decel = elapsed_time - peak_time;
            double peak_velocity = a_max_ * peak_time;
            double peak_distance = 0.5 * a_max_ * peak_time * peak_time;  // Distance at peak
            return peak_distance + peak_velocity * t_decel - 0.5 * a_max_ * t_decel * t_decel;
        } else {
            return total_arc_length_;
        }
    } else {
        // Trapezoidal profile
        double cruise_distance = total_arc_length_ - 2 * accel_distance;
        double cruise_time = cruise_distance / v_max_;
        double total_time = 2 * accel_time + cruise_time;
        
        if (elapsed_time < accel_time) {
            // Acceleration phase
            return 0.5 * a_max_ * elapsed_time * elapsed_time;
        } else if (elapsed_time < accel_time + cruise_time) {
            // Cruise phase
            double t_cruise = elapsed_time - accel_time;
            return accel_distance + v_max_ * t_cruise;
        } else if (elapsed_time < total_time) {
            // Deceleration phase
            double t_decel = elapsed_time - accel_time - cruise_time;
            return accel_distance + cruise_distance + v_max_ * t_decel - 0.5 * a_max_ * t_decel * t_decel;
        } else {
            return total_arc_length_;
        }
    }
}

double BSplineTrajectory::ComputeDesiredSpeed(double elapsed_time) const {
    // Compute speed from trapezoidal velocity profile
    double accel_time = v_max_ / a_max_;
    double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
    
    if (2 * accel_distance > total_arc_length_) {
        // Triangular profile
        double peak_time = std::sqrt(total_arc_length_ / a_max_);
        if (elapsed_time < peak_time) {
            return a_max_ * elapsed_time;
        } else if (elapsed_time < 2 * peak_time) {
            double t_decel = elapsed_time - peak_time;
            return a_max_ * peak_time - a_max_ * t_decel;
        } else {
            return 0.0;
        }
    } else {
        // Trapezoidal profile
        double cruise_distance = total_arc_length_ - 2 * accel_distance;
        double cruise_time = cruise_distance / v_max_;
        
        if (elapsed_time < accel_time) {
            return a_max_ * elapsed_time;
        } else if (elapsed_time < accel_time + cruise_time) {
            return v_max_;
        } else if (elapsed_time < 2 * accel_time + cruise_time) {
            double t_decel = elapsed_time - accel_time - cruise_time;
            return v_max_ - a_max_ * t_decel;
        } else {
            return 0.0;
        }
    }
}

bool BSplineTrajectory::AddGoal(const Eigen::Vector3d& goal) {
    std::cout << "[BSplineTrajectory::AddGoal] Dynamic goal addition not yet supported" << std::endl;
    return false;
}

double BSplineTrajectory::GetRemainingTime() const {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return 0.0;
    }
    
    // Calculate total time based on trapezoidal profile
    double accel_time = v_max_ / a_max_;
    double accel_distance = 0.5 * a_max_ * accel_time * accel_time;
    
    double total_time;
    if (2 * accel_distance > total_arc_length_) {
        total_time = 2 * std::sqrt(total_arc_length_ / a_max_);
    } else {
        double cruise_distance = total_arc_length_ - 2 * accel_distance;
        double cruise_time = cruise_distance / v_max_;
        total_time = 2 * accel_time + cruise_time;
    }
    
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    return std::max(0.0, total_time - elapsed);
}

void BSplineTrajectory::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
    
    std::cout << "[BSplineTrajectory] Updated limits: v_max=" << v_max_ 
              << ", a_max=" << a_max_ << ", ω_max=" << omega_max_ 
              << ", α_max=" << alpha_max_ << std::endl;
}

void BSplineTrajectory::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    if (!robot_manager) {
        std::cerr << "[BSplineTrajectory] Error: RobotManager is null" << std::endl;
        return;
    }
    
    Eigen::Vector3d current_pose = robot_manager->GetPoseInWorldFrame();
    Eigen::Vector3d current_velocity = robot_manager->GetVelocityInWorldFrame();
    
    std::cout << "[BSplineTrajectory] Initialized from RobotManager:" << std::endl;
    std::cout << "  Current pose: " << current_pose.transpose() << std::endl;
    std::cout << "  Current velocity: " << current_velocity.transpose() << std::endl;
}

double BSplineTrajectory::NormalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void BSplineTrajectory::ResetTrajectory() {
    is_trajectory_active_ = false;
    is_trajectory_finished_ = true;
    trajectory_start_time_ = 0.0;
    current_segment_progress_ = 0.0;
    control_points_.clear();
    knot_vector_.clear();
    arc_length_samples_.clear();
    waypoints_.clear();
}

void BSplineTrajectory::SetFeedbackGains(double kp, double kd) {
    kp_ = kp;
    kd_ = kd;
    std::cout << "[BSplineTrajectory] Updated feedback gains: kp=" << kp_ << ", kd=" << kd_ << std::endl;
}

void BSplineTrajectory::SetSplineDegree(int degree) {
    spline_degree_ = std::max(1, std::min(degree, 5));  // Limit to reasonable range
    std::cout << "[BSplineTrajectory] Set spline degree to " << spline_degree_ << std::endl;
}

} // namespace ctrl