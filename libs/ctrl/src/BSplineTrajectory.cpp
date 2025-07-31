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

void BSplineTrajectory::GenerateBSplineControlPoints() {
    control_points_.clear();
    
    if (waypoints_.size() == 2) {
        // For only 2 waypoints, add intermediate control points
        Eigen::Vector3d p0 = waypoints_[0];
        Eigen::Vector3d p1 = waypoints_[1];
        Eigen::Vector3d dir = (p1 - p0).normalized();
        
        control_points_.push_back(p0);
        control_points_.push_back(p0 + 0.25 * (p1 - p0));
        control_points_.push_back(p0 + 0.5 * (p1 - p0));
        control_points_.push_back(p0 + 0.75 * (p1 - p0));
        control_points_.push_back(p1);
    } else {
        // For multiple waypoints, use centripetal parameterization
        // This creates smoother curves through the waypoints
        
        // Add first point multiple times for clamped B-spline
        for (int i = 0; i <= spline_degree_; ++i) {
            control_points_.push_back(waypoints_[0]);
        }
        
        // Add intermediate waypoints
        for (size_t i = 1; i < waypoints_.size() - 1; ++i) {
            control_points_.push_back(waypoints_[i]);
        }
        
        // Add last point multiple times for clamped B-spline
        for (int i = 0; i <= spline_degree_; ++i) {
            control_points_.push_back(waypoints_.back());
        }
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

double BSplineTrajectory::BSplineBasis(int i, int p, double u) {
    // Cox-de Boor recursion formula for B-spline basis functions
    if (p == 0) {
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

Eigen::Vector3d BSplineTrajectory::EvaluateBSpline(double u) {
    // Clamp u to valid range
    u = std::clamp(u, 0.0, 1.0);
    
    // Handle edge case at u = 1.0
    if (u >= 1.0 - 1e-10) {
        u = 1.0 - 1e-10;
    }
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    for (size_t i = 0; i < control_points_.size(); ++i) {
        double basis = BSplineBasis(i, spline_degree_, u);
        result += basis * control_points_[i];
    }
    
    // Don't normalize angle here - we want to support continuous rotations beyond ±π
    // result[2] = NormalizeAngle(result[2]);
    
    return result;
}

Eigen::Vector3d BSplineTrajectory::EvaluateBSplineDerivative(double u, int derivative_order) {
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

double BSplineTrajectory::ArcLengthToParameter(double arc_length) {
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

double BSplineTrajectory::ComputeDesiredArcLength(double elapsed_time) {
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

double BSplineTrajectory::ComputeDesiredSpeed(double elapsed_time) {
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

double BSplineTrajectory::NormalizeAngle(double angle) {
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