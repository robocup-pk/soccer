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
        
        // Clear derivative cache
        derivative_control_points_.clear();
        
        // Apply boundary constraints to ensure robot stays within field
        ApplyBoundaryConstraints();
        
        // Generate uniform knot vector
        //GenerateUniformKnotVector();
        GenerateCentripetalKnotVector();

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
        
        // Reset control state
        has_previous_update_ = false;
        has_previous_command_ = false;  // Reset velocity filter
        previous_velocity_command_ = Eigen::Vector3d::Zero();
        previous_pose_error_ = Eigen::Vector3d::Zero();
        previous_update_time_ = 0.0;
        t_error_integral_ = 0.0;
        t_error_prev_ = 0.0;
        
        std::cout << "[UniformBSplineTrajectoryPlanner::SetPath] Trajectory generated successfully. "
                  << "Control points: " << control_points_.size() 
                  << ", Arc length: " << total_arc_length_ << "m"
                  << ", Duration: " << trajectory_duration_ << "s" << std::endl;
        
        // Debug: check endpoint interpolation
        Eigen::Vector3d start_point = EvaluateBSpline(0.0);
        Eigen::Vector3d end_point = EvaluateBSpline(1.0);
        std::cout << "[DEBUG] Spline start: (" << start_point[0] << ", " << start_point[1] << ", " << start_point[2] << ")" << std::endl;
        std::cout << "[DEBUG] Spline end: (" << end_point[0] << ", " << end_point[1] << ", " << end_point[2] << ")" << std::endl;
        std::cout << "[DEBUG] Expected end: (" << waypoints_.back()[0] << ", " << waypoints_.back()[1] << ", " << waypoints_.back()[2] << ")" << std::endl;
        
        // Debug control points
        std::cout << "[DEBUG] Control points:" << std::endl;
        for (size_t i = 0; i < control_points_.size(); ++i) {
            std::cout << "  CP[" << i << "]: (" << control_points_[i][0] << ", " 
                      << control_points_[i][1] << ", " << control_points_[i][2] << ")" << std::endl;
        }
        
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

                // Always apply cornering logic, but vary the offset based on the angle
                double corner_offset;
                double inward_pull_factor;

                if (angle < PlannerConstants::SHARP_CORNER_ANGLE_RAD) { // Sharp corner
                    if (std::abs(angle - PlannerConstants::NINETY_DEG_CORNER_ANGLE_RAD) < PlannerConstants::NINETY_DEG_CORNER_ANGLE_TOLERANCE_RAD) {
                        corner_offset = PlannerConstants::CORNER_OFFSET_M;
                        inward_pull_factor = PlannerConstants::CORNER_INWARD_PULL_FACTOR;
                    } else {
                        corner_offset = PlannerConstants::GENERAL_CORNER_OFFSET_M;
                        inward_pull_factor = PlannerConstants::GENERAL_CORNER_INWARD_PULL_FACTOR;
                    }
                } else { // Smooth corner
                    corner_offset = PlannerConstants::SMOOTH_CORNER_OFFSET_M;
                    inward_pull_factor = PlannerConstants::CORNER_INWARD_PULL_FACTOR; // Use the same pull factor for consistency
                }

                Eigen::Vector2d bisector = -(v1 + v2).normalized();

                Eigen::Vector3d before_corner = curr;
                before_corner.head<2>() = curr.head<2>() - v1 * corner_offset;
                control_points_.push_back(before_corner);

                Eigen::Vector3d corner_point = curr;
                corner_point.head<2>() = curr.head<2>() + bisector * corner_offset * inward_pull_factor;
                control_points_.push_back(corner_point);

                Eigen::Vector3d after_corner = curr;
                after_corner.head<2>() = curr.head<2>() + v2 * corner_offset;
                control_points_.push_back(after_corner);

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

// ---------------------------------------------------------------------------
//  Centripetal knot vector (open, clamped) –
//  internal knots are spaced according to the √(chord-length) between control
//  points, which prevents tight corners from getting too little “time”.
// ---------------------------------------------------------------------------
void UniformBSplineTrajectoryPlanner::GenerateCentripetalKnotVector()
{
    knot_vector_.clear();

    const int n = static_cast<int>(control_points_.size()) - 1; // last index
    const int p = SPLINE_DEGREE;                                // spline degree
    const int m = n + p + 1;                                    // last knot index

    // 1.  Parameter-values u[i] using centripetal (√chord-length) measure
    std::vector<double> u(n + 1, 0.0);          // u[0] = 0
    double total = 0.0;
    for (int i = 1; i <= n; ++i)
    {
        // chord length in XY plane (orientation does not affect spacing)
        double chord =  (control_points_[i].head<2>() -
                         control_points_[i - 1].head<2>()).norm();
        total       += std::sqrt(chord);        // centripetal → √-distance
        u[i]         = total;
    }
    for (double &val : u) val /= total;         // normalise to [0,1]

    // 2.  Build open (clamped) knot vector
    //     first (p+1) zeros, last (p+1) ones
    for (int i = 0; i <= p; ++i) knot_vector_.push_back(0.0);

    const int num_internal = m - 2 * (p + 1);   // how many interior knots?
    for (int j = 1; j <= num_internal; ++j)
    {
        // each interior knot is the average of p consecutive u’s
        double sum = 0.0;
        for (int k = j; k < j + p; ++k) sum += u[k];
        knot_vector_.push_back(sum / p);
    }

    for (int i = 0; i <= p; ++i) knot_vector_.push_back(1.0);
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
    
    // Special case for endpoints to ensure exact interpolation
    if (u >= 1.0 - 1e-10) {
        // Return last control point for u=1
        return control_points_.back();
    }
    if (u <= 1e-10) {
        // Return first control point for u=0
        return control_points_.front();
    }
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    // Evaluate using basis functions
    for (size_t i = 0; i < control_points_.size(); ++i) {
        double basis = BSplineBasis(i, SPLINE_DEGREE, u, knot_vector_);
        result += basis * control_points_[i];
    }
    
    return result;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::EvaluateBSpline(double u,
                                                                 int degree,
                                                                 const std::vector<Eigen::Vector3d>& control_points,
                                                                 const std::vector<double>& knot_vector) const {
    u = std::clamp(u, 0.0, 1.0);

    if (control_points.empty()) {
        return Eigen::Vector3d::Zero();
    }

    if (u >= 1.0 - 1e-10) {
        return control_points.back();
    }
    if (u <= 1e-10) {
        return control_points.front();
    }

    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < control_points.size(); ++i) {
        double basis = BSplineBasis(i, degree, u, knot_vector);
        result += basis * control_points[i];
    }
    return result;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::EvaluateBSplineDerivative(double u, int derivative_order) const {
    return EvaluateBSplineDerivativeAnalytically(u, derivative_order);
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::EvaluateBSplineDerivativeAnalytically(double u, int derivative_order) const {
    if (derivative_order < 1) {
        return Eigen::Vector3d::Zero();
    }

    if (derivative_control_points_.size() < static_cast<size_t>(derivative_order)) {
        derivative_control_points_.resize(derivative_order);
    }

    // First derivative
    if (derivative_control_points_[0].empty()) {
        if (control_points_.size() < 2) {
            return Eigen::Vector3d::Zero();
        }
        derivative_control_points_[0].resize(control_points_.size() - 1);
        for (size_t i = 0; i < control_points_.size() - 1; ++i) {
            double denom = knot_vector_[i + SPLINE_DEGREE + 1] - knot_vector_[i + 1];
            if (std::abs(denom) > 1e-9) {
                derivative_control_points_[0][i] = SPLINE_DEGREE * (control_points_[i + 1] - control_points_[i]) / denom;
            } else {
                derivative_control_points_[0][i] = Eigen::Vector3d::Zero();
            }
        }
    }

    // Higher order derivatives
    for (int d = 2; d <= derivative_order; ++d) {
        if (derivative_control_points_[d - 1].empty()) {
            const auto& prev_control_points = derivative_control_points_[d - 2];
            if (prev_control_points.size() < 2) {
                return Eigen::Vector3d::Zero();
            }
            derivative_control_points_[d - 1].resize(prev_control_points.size() - 1);
            for (size_t i = 0; i < prev_control_points.size() - 1; ++i) {
                double denom = knot_vector_[i + SPLINE_DEGREE - d + 2] - knot_vector_[i + 1];
                if (std::abs(denom) > 1e-9) {
                    derivative_control_points_[d - 1][i] = (SPLINE_DEGREE - d + 1) * (prev_control_points[i + 1] - prev_control_points[i]) / denom;
                } else {
                    derivative_control_points_[d - 1][i] = Eigen::Vector3d::Zero();
                }
            }
        }
    }

    std::vector<double> derivative_knot_vector(knot_vector_.begin() + derivative_order, knot_vector_.end() - derivative_order);

    return EvaluateBSpline(u, SPLINE_DEGREE - derivative_order, derivative_control_points_[derivative_order - 1], derivative_knot_vector);
}


void UniformBSplineTrajectoryPlanner::CalculateArcLength() {
    // Calculate arc length using adaptive sampling
    const int num_samples = PlannerConstants::ARC_LENGTH_NUM_SAMPLES;  // More samples for better accuracy
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
        // Use smaller weight for angular distance to avoid overestimating arc length
        // For straight paths, angular changes should have minimal impact
        double segment_length = linear_dist + PlannerConstants::ARC_LENGTH_ANGULAR_WEIGHT * angular_dist;
        
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
        double total_time = 2 * accel_time + cruise_time;
        
        if (elapsed_time < accel_time) {
            return a_max_ * elapsed_time;
        } else if (elapsed_time < accel_time + cruise_time) {
            return v_max_;
        } else if (elapsed_time < total_time) {
            double decel_time = elapsed_time - accel_time - cruise_time;
            return std::max(0.0, v_max_ - a_max_ * decel_time);
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
        double corner_margin = PlannerConstants::BOUNDARY_CORNER_MARGIN_M; // 5cm from corner
        
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
            control_points_[i].head<2>() += to_center * PlannerConstants::BOUNDARY_CORNER_PULL_M; // Pull 2cm toward center
        }
    }
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }

    double elapsed_time = current_time - trajectory_start_time_;

    if (elapsed_time >= trajectory_duration_) {
        return HandleFinishedTrajectory(current_pose);
    }

    double dt = has_previous_update_ ? (current_time - previous_update_time_) : 0.01;
    dt = std::max(dt, PlannerConstants::MIN_DT_S);

    DesiredState desired_state = CalculateDesiredState(elapsed_time);

    Eigen::Vector3d velocity_command = CalculateVelocityCommand(current_pose, desired_state, dt, elapsed_time);

    ApplyVelocityLimitsAndFilter(velocity_command);

    previous_pose_error_ = desired_state.pose - current_pose;
    previous_update_time_ = current_time;
    has_previous_update_ = true;
    previous_velocity_command_ = velocity_command;
    has_previous_command_ = true;

    return velocity_command;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::HandleFinishedTrajectory(const Eigen::Vector3d& current_pose) {
    Eigen::Vector3d final_target = EvaluateBSpline(1.0);
    Eigen::Vector3d position_error = final_target - current_pose;

    static bool debug_printed = false;
    if (!debug_printed) {
        std::cout << "[DEBUG] Trajectory ended. Current pose: (" << current_pose[0] << ", "
                  << current_pose[1] << ", " << current_pose[2] << ")" << std::endl;
        std::cout << "[DEBUG] Final target: (" << final_target[0] << ", "
                  << final_target[1] << ", " << final_target[2] << ")" << std::endl;
        std::cout << "[DEBUG] Position error: " << position_error.head<2>().norm() << "m" << std::endl;
        debug_printed = true;
    }

    if (position_error.head<2>().norm() < PlannerConstants::POSITION_TOLERANCE_M &&
        std::abs(NormalizeAngle(position_error[2])) < PlannerConstants::ANGLE_TOLERANCE_RAD) {
        is_trajectory_finished_ = true;
        is_trajectory_active_ = false;
        t_error_integral_ = 0.0;
        debug_printed = false;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d velocity_command;
    velocity_command.head<2>() = kp_ * 10.0 * position_error.head<2>();
    velocity_command[2] = kp_ * 10.0 * NormalizeAngle(position_error[2]);

    velocity_command[0] = std::clamp(velocity_command[0], -PlannerConstants::FINAL_APPROACH_MAX_VEL_MS, PlannerConstants::FINAL_APPROACH_MAX_VEL_MS);
    velocity_command[1] = std::clamp(velocity_command[1], -PlannerConstants::FINAL_APPROACH_MAX_VEL_MS, PlannerConstants::FINAL_APPROACH_MAX_VEL_MS);
    velocity_command[2] = std::clamp(velocity_command[2], -PlannerConstants::FINAL_APPROACH_MAX_OMEGA_RADS, PlannerConstants::FINAL_APPROACH_MAX_OMEGA_RADS);

    return velocity_command;
}

UniformBSplineTrajectoryPlanner::DesiredState UniformBSplineTrajectoryPlanner::CalculateDesiredState(double elapsed_time) {
    DesiredState desired_state;
    double desired_arc_length = ComputeDesiredArcLength(elapsed_time);
    double u_desired = ArcLengthToParameter(desired_arc_length);

    desired_state.pose = EvaluateBSpline(u_desired);
    desired_state.velocity = EvaluateBSplineDerivative(u_desired, 1);

    return desired_state;
}

Eigen::Vector3d UniformBSplineTrajectoryPlanner::CalculateVelocityCommand(const Eigen::Vector3d& current_pose,
                                                                        const DesiredState& desired_state,
                                                                        double dt, double elapsed_time) {
    Eigen::Vector3d pose_error = desired_state.pose - current_pose;
    double desired_speed = ComputeDesiredSpeed(elapsed_time);

    Eigen::Vector3d desired_velocity_tangent = desired_state.velocity;
    if (desired_velocity_tangent.head<2>().norm() > 1e-6) {
        desired_velocity_tangent.head<2>().normalize();
        desired_velocity_tangent.head<2>() *= desired_speed;
    }

    Eigen::Vector3d velocity_command;
    double curvature = 0.0;
    Eigen::Vector2d tangent_2d = desired_velocity_tangent.head<2>();
    if (tangent_2d.norm() > 1e-6) {
        tangent_2d.normalize();
        Eigen::Vector2d perpendicular(-tangent_2d[1], tangent_2d[0]);
        double cross_track_error = pose_error.head<2>().dot(perpendicular);
        double along_track_error = pose_error.head<2>().dot(tangent_2d);

        double lookahead_time = PlannerConstants::LOOKAHEAD_TIME_S;
        double future_u = std::clamp(ArcLengthToParameter(ComputeDesiredArcLength(ParameterToTime(ArcLengthToParameter(total_arc_length_)) + lookahead_time)), 0.0, 1.0);
        Eigen::Vector3d future_tangent = EvaluateBSplineDerivative(future_u, 1);

        if (future_tangent.head<2>().norm() > 1e-6) {
            future_tangent.head<2>().normalize();
            double angle_change = std::acos(std::clamp(tangent_2d.dot(future_tangent.head<2>()), -1.0, 1.0));
            curvature = angle_change / lookahead_time;
        }

        ApplyControlGains(velocity_command, pose_error, desired_velocity_tangent, curvature, desired_speed);
    } else {
        velocity_command.head<2>() = kp_ * 2.0 * pose_error.head<2>();
    }

    if (has_previous_update_ && dt > 0.001) {
        Eigen::Vector3d error_derivative = (pose_error - previous_pose_error_) / dt;
        velocity_command.head<2>() -= PlannerConstants::DAMPING_GAIN * error_derivative.head<2>();
    }

    double desired_heading = 0.0;
    double heading_lookahead = PlannerConstants::HEADING_LOOKAHEAD_S;
    double heading_u = std::clamp(ArcLengthToParameter(ComputeDesiredArcLength(ParameterToTime(ArcLengthToParameter(total_arc_length_)) + heading_lookahead)), 0.0, 1.0);
    Eigen::Vector3d heading_tangent = EvaluateBSplineDerivative(heading_u, 1);

    if (heading_tangent.head<2>().norm() > 0.1) {
        desired_heading = std::atan2(heading_tangent[1], heading_tangent[0]);
    } else if (velocity_command.head<2>().norm() > 0.1) {
        desired_heading = std::atan2(velocity_command[1], velocity_command[0]);
    } else {
        desired_heading = current_pose[2];
    }

    double heading_error = NormalizeAngle(desired_heading - current_pose[2]);
    double heading_gain = PlannerConstants::BASE_HEADING_GAIN;
    if (std::abs(heading_error) < PlannerConstants::SMALL_HEADING_ERROR_RAD) {
        heading_gain = PlannerConstants::SMALL_HEADING_ERROR_GAIN;
    } else if (std::abs(heading_error) > PlannerConstants::LARGE_HEADING_ERROR_RAD) {
        heading_gain = PlannerConstants::LARGE_HEADING_ERROR_GAIN;
    }
    velocity_command[2] = heading_gain * heading_error;

    return velocity_command;
}

void UniformBSplineTrajectoryPlanner::ApplyControlGains(Eigen::Vector3d& velocity_command,
                                                      const Eigen::Vector3d& pose_error,
                                                      const Eigen::Vector3d& desired_velocity_tangent,
                                                      double curvature,
                                                      double desired_speed) {
    Eigen::Vector2d tangent_2d = desired_velocity_tangent.head<2>().normalized();
    Eigen::Vector2d perpendicular(-tangent_2d[1], tangent_2d[0]);
    double cross_track_error = pose_error.head<2>().dot(perpendicular);
    double along_track_error = pose_error.head<2>().dot(tangent_2d);

    double cross_track_gain = PlannerConstants::BASE_CROSS_TRACK_GAIN;
    double error_magnitude = std::abs(cross_track_error);
    if (error_magnitude > PlannerConstants::LARGE_ERROR_THRESHOLD_M) {
        cross_track_gain = PlannerConstants::LARGE_ERROR_CROSS_TRACK_GAIN;
    } else if (error_magnitude > PlannerConstants::MEDIUM_ERROR_THRESHOLD_M) {
        cross_track_gain = PlannerConstants::MEDIUM_ERROR_CROSS_TRACK_GAIN;
    }

    double speed_factor = 1.0;
    if (curvature > PlannerConstants::SHARP_CORNER_CURVATURE) {
        cross_track_gain *= PlannerConstants::SHARP_CORNER_GAIN_FACTOR;
        speed_factor = PlannerConstants::SHARP_CORNER_SPEED_FACTOR;
    } else if (curvature > PlannerConstants::MODERATE_CORNER_CURVATURE) {
        cross_track_gain *= PlannerConstants::MODERATE_CORNER_GAIN_FACTOR;
        speed_factor = PlannerConstants::MODERATE_CORNER_SPEED_FACTOR;
    }

    velocity_command.head<2>() = tangent_2d * (desired_speed * speed_factor);
    velocity_command.head<2>() += perpendicular * (cross_track_gain * cross_track_error);
    velocity_command.head<2>() += tangent_2d * (PlannerConstants::BASE_ALONG_TRACK_GAIN * along_track_error);

    if (curvature > PlannerConstants::VERY_SHARP_CORNER_CURVATURE) {
        velocity_command.head<2>() += perpendicular * PlannerConstants::VERY_SHARP_CORNER_COMPENSATION_M;
    }
}

void UniformBSplineTrajectoryPlanner::ApplyVelocityLimitsAndFilter(Eigen::Vector3d& velocity_command) {
    double vel_magnitude = velocity_command.head<2>().norm();
    if (vel_magnitude > v_max_) {
        velocity_command.head<2>() = velocity_command.head<2>().normalized() * v_max_;
    }

    velocity_command[2] = std::clamp(velocity_command[2], -omega_max_, omega_max_);

    if (has_previous_command_) {
        velocity_command = PlannerConstants::VELOCITY_FILTER_ALPHA * velocity_command +
                         (1.0 - PlannerConstants::VELOCITY_FILTER_ALPHA) * previous_velocity_command_;
    }
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
    
    // Reset control state when gains change
    has_previous_update_ = false;
    previous_pose_error_ = Eigen::Vector3d::Zero();
    t_error_integral_ = 0.0;
    t_error_prev_ = 0.0;
    
    std::cout << "[UniformBSplineTrajectoryPlanner] Feedback gains set to: kp=" << kp_ << ", kd=" << kd_ << std::endl;
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

Eigen::Vector3d UniformBSplineTrajectoryPlanner::GetIdealPosition(double current_time) const {
    if (!is_trajectory_active_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    
    // If before trajectory start, return start position
    if (elapsed_time <= 0) {
        return EvaluateBSpline(0.0);
    }
    
    // If past trajectory duration, return end position
    if (elapsed_time >= trajectory_duration_) {
        return EvaluateBSpline(1.0);
    }
    
    // Calculate the ideal position along the trajectory
    double desired_arc_length = ComputeDesiredArcLength(elapsed_time);
    double u = ArcLengthToParameter(desired_arc_length);
    
    // Return the ideal pose at this point on the spline
    return EvaluateBSpline(u);
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
            if (to_waypoint.dot(to_next) >= 0 || dist < PlannerConstants::REMAINING_PATH_WAYPOINT_DOT_THRESHOLD) {  // Within 10cm or ahead
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
        if (i == closest_idx && min_distance < PlannerConstants::REMAINING_PATH_MIN_DIST_M) {  // Within 5cm
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

double UniformBSplineTrajectoryPlanner::FindClosestParameter(const Eigen::Vector2d& position) const {
    double best_u = 0.0;
    double min_dist_sq = std::numeric_limits<double>::max();
    FindClosestParameterRecursive(position, 0.0, 1.0, best_u, min_dist_sq);
    return best_u;
}

void UniformBSplineTrajectoryPlanner::FindClosestParameterRecursive(const Eigen::Vector2d& position,
                                                                    double u_min,
                                                                    double u_max,
                                                                    double& best_u,
                                                                    double& min_dist_sq) const {
    // Base case: if the interval is small enough, sample points and find the closest one
    if (u_max - u_min < 1e-3) {
        const int num_samples = 10;
        for (int i = 0; i <= num_samples; ++i) {
            double u = u_min + (u_max - u_min) * static_cast<double>(i) / num_samples;
            Eigen::Vector3d point = EvaluateBSpline(u);
            double dist_sq = (point.head<2>() - position).squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_u = u;
            }
        }
        return;
    }

    // Recursive step: subdivide the interval and search in the most promising sub-interval
    double u_mid = (u_min + u_max) / 2.0;
    Eigen::Vector3d point_min = EvaluateBSpline(u_min);
    Eigen::Vector3d point_mid = EvaluateBSpline(u_mid);
    Eigen::Vector3d point_max = EvaluateBSpline(u_max);

    double dist_sq_min = (point_min.head<2>() - position).squaredNorm();
    double dist_sq_mid = (point_mid.head<2>() - position).squaredNorm();
    double dist_sq_max = (point_max.head<2>() - position).squaredNorm();

    if (dist_sq_min < min_dist_sq) {
        min_dist_sq = dist_sq_min;
        best_u = u_min;
    }
    if (dist_sq_mid < min_dist_sq) {
        min_dist_sq = dist_sq_mid;
        best_u = u_mid;
    }
    if (dist_sq_max < min_dist_sq) {
        min_dist_sq = dist_sq_max;
        best_u = u_max;
    }

    // Prune the search space by checking the bounding boxes of the sub-intervals
    Eigen::Vector2d min_p1, max_p1, min_p2, max_p2;
    min_p1 = point_min.head<2>().cwiseMin(point_mid.head<2>());
    max_p1 = point_min.head<2>().cwiseMax(point_mid.head<2>());
    min_p2 = point_mid.head<2>().cwiseMin(point_max.head<2>());
    max_p2 = point_mid.head<2>().cwiseMax(point_max.head<2>());

    double dist_sq_bb1 = 0.0;
    for (int i = 0; i < 2; ++i) {
        if (position[i] < min_p1[i]) {
            dist_sq_bb1 += std::pow(position[i] - min_p1[i], 2);
        } else if (position[i] > max_p1[i]) {
            dist_sq_bb1 += std::pow(position[i] - max_p1[i], 2);
        }
    }

    double dist_sq_bb2 = 0.0;
    for (int i = 0; i < 2; ++i) {
        if (position[i] < min_p2[i]) {
            dist_sq_bb2 += std::pow(position[i] - min_p2[i], 2);
        } else if (position[i] > max_p2[i]) {
            dist_sq_bb2 += std::pow(position[i] - max_p2[i], 2);
        }
    }

    if (dist_sq_bb1 < dist_sq_bb2) {
        if (dist_sq_bb1 < min_dist_sq) {
            FindClosestParameterRecursive(position, u_min, u_mid, best_u, min_dist_sq);
        }
        if (dist_sq_bb2 < min_dist_sq) {
            FindClosestParameterRecursive(position, u_mid, u_max, best_u, min_dist_sq);
        }
    } else {
        if (dist_sq_bb2 < min_dist_sq) {
            FindClosestParameterRecursive(position, u_mid, u_max, best_u, min_dist_sq);
        }
        if (dist_sq_bb1 < min_dist_sq) {
            FindClosestParameterRecursive(position, u_min, u_mid, best_u, min_dist_sq);
        }
    }
}

} // namespace ctrl