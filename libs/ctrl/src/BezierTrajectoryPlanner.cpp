#include "BezierTrajectoryPlanner.h"
#include "RobotDescription.h"
#include "RobotManager.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace ctrl {

BezierTrajectoryPlanner::BezierTrajectoryPlanner() 
    : robot_description_(kin::GetRobotDescription()),
      trajectory_start_time_(0.0),
      trajectory_duration_(0.0),
      is_trajectory_active_(false),
      is_trajectory_finished_(true) {
    
    std::cout << "[BezierTrajectoryPlanner] Initialized with RoboJackets-style Bezier approach" << std::endl;
}

bool BezierTrajectoryPlanner::SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    if (path_fWorld.size() < 2) {
        std::cerr << "[BezierTrajectoryPlanner::SetPath] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[BezierTrajectoryPlanner::SetPath] Setting path with " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    try {
        // Store original waypoints
        waypoints_ = path_fWorld;
        
        // Clear previous trajectory
        bezier_segments_.clear();
        trajectory_instants_.clear();
        path_points_.clear();
        
        // Preprocess waypoints to handle sharp corners
        path_points_ = PreprocessWaypoints(waypoints_);
        
        std::cout << "[BezierTrajectoryPlanner] After preprocessing: " 
                  << path_points_.size() << " path points" << std::endl;
        
        // Initial and final velocities (start and end at rest)
        Eigen::Vector2d v_initial = Eigen::Vector2d::Zero();
        Eigen::Vector2d v_final = Eigen::Vector2d::Zero();
        
        // Fit cubic Bezier curves through points
        FitCubicBezier(path_points_, v_initial, v_final);
        
        // Profile velocity along the path
        ProfileVelocity(0.0, 0.0);
        
        // Plan angles separately
        PlanAngles(waypoints_);
        
        // Set trajectory timing
        trajectory_start_time_ = (t_start_s > 0) ? t_start_s : util::GetCurrentTime();
        
        if (!trajectory_instants_.empty()) {
            trajectory_duration_ = trajectory_instants_.back().timestamp;
        }
        
        is_trajectory_active_ = true;
        is_trajectory_finished_ = false;
        
        std::cout << "[BezierTrajectoryPlanner::SetPath] Trajectory generated successfully. "
                  << "Instants: " << trajectory_instants_.size() 
                  << ", Duration: " << trajectory_duration_ << "s" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[BezierTrajectoryPlanner::SetPath] Exception: " << e.what() << std::endl;
        return false;
    }
}

std::vector<Eigen::Vector2d> BezierTrajectoryPlanner::PreprocessWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints) {
    
    std::vector<Eigen::Vector2d> points;
    
    // Always add the first point
    points.push_back(waypoints[0].head<2>());
    
    // Process intermediate waypoints
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        Eigen::Vector2d prev = waypoints[i-1].head<2>();
        Eigen::Vector2d curr = waypoints[i].head<2>();
        Eigen::Vector2d next = waypoints[i+1].head<2>();
        
        if (IsSharpCorner(prev, curr, next)) {
            // For sharp corners, add intermediate points to help the robot navigate
            Eigen::Vector2d v1 = (curr - prev).normalized();
            Eigen::Vector2d v2 = (next - curr).normalized();
            
            // Add point before corner
            if ((curr - prev).norm() > corner_cut_distance_) {
                points.push_back(curr - v1 * corner_cut_distance_);
            }
            
            // Add the corner point
            points.push_back(curr);
            
            // Add point after corner
            if ((next - curr).norm() > corner_cut_distance_) {
                points.push_back(curr + v2 * corner_cut_distance_);
            }
        } else {
            // For smooth corners, just add the waypoint
            points.push_back(curr);
        }
    }
    
    // Always add the last point
    points.push_back(waypoints.back().head<2>());
    
    return points;
}

bool BezierTrajectoryPlanner::IsSharpCorner(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                                           const Eigen::Vector2d& p2) const {
    Eigen::Vector2d v1 = (p1 - p0).normalized();
    Eigen::Vector2d v2 = (p2 - p1).normalized();
    
    // Calculate angle between vectors
    double dot_product = v1.dot(v2);
    double angle = std::acos(std::clamp(dot_product, -1.0, 1.0));
    
    // Check if angle is sharp (less than 135 degrees)
    return angle < sharp_corner_angle_;
}

void BezierTrajectoryPlanner::FitCubicBezier(const std::vector<Eigen::Vector2d>& points,
                                            const Eigen::Vector2d& vi, const Eigen::Vector2d& vf) {
    if (points.size() < 2) {
        throw std::invalid_argument("Must have at least 2 points to fit cubic Bezier");
    }
    
    int num_curves = static_cast<int>(points.size()) - 1;
    bezier_segments_.resize(num_curves);
    
    // Calculate time estimates for each segment
    std::vector<double> ks(num_curves);
    double total_path_length = 0.0;
    
    for (size_t i = 0; i < points.size() - 1; i++) {
        total_path_length += (points[i+1] - points[i]).norm();
    }
    
    double path_length_so_far = 0.0;
    for (int i = 0; i < num_curves; i++) {
        double time_before = GetTrapezoidalTime(path_length_so_far, v_max_, a_max_, 
                                               vi.norm(), vf.norm());
        path_length_so_far += (points[i+1] - points[i]).norm();
        double time_after = GetTrapezoidalTime(path_length_so_far, v_max_, a_max_,
                                              vi.norm(), vf.norm());
        
        ks[i] = 1.0 / (time_after - time_before);
        
        if (!std::isfinite(ks[i])) {
            ks[i] = 1.0; // Default to 1 second per segment
        }
    }
    
    // Set endpoints for each curve
    for (int i = 0; i < num_curves; i++) {
        bezier_segments_[i].p0 = points[i];
        bezier_segments_[i].p3 = points[i + 1];
    }
    
    // RoboJackets-style control point calculation
    if (num_curves == 1) {
        // Single curve case
        bezier_segments_[0].p1 = points[0] + vi / (3.0 * ks[0]);
        bezier_segments_[0].p2 = points[1] - vf / (3.0 * ks[0]);
    } else {
        // Multiple curves - solve system of equations for smooth transitions
        using Eigen::MatrixXd;
        using Eigen::VectorXd;
        
        int matrix_size = num_curves * 2;
        MatrixXd equations = MatrixXd::Zero(matrix_size, matrix_size);
        VectorXd answer_x(matrix_size);
        VectorXd answer_y(matrix_size);
        
        // Boundary conditions
        equations(0, 0) = 1;
        answer_x(0) = vi.x() / (3.0 * ks[0]) + points[0].x();
        answer_y(0) = vi.y() / (3.0 * ks[0]) + points[0].y();
        
        equations(1, matrix_size - 1) = 1;
        answer_x(1) = points[num_curves].x() - vf.x() / (3 * ks[num_curves - 1]);
        answer_y(1) = points[num_curves].y() - vf.y() / (3 * ks[num_curves - 1]);
        
        int eq_idx = 2;
        
        // Continuity of first derivative at junctions
        for (int n = 0; n < num_curves - 1; n++) {
            equations(eq_idx, n * 2 + 1) = ks[n];
            equations(eq_idx, n * 2 + 2) = ks[n + 1];
            answer_x(eq_idx) = (ks[n] + ks[n + 1]) * points[n + 1].x();
            answer_y(eq_idx) = (ks[n] + ks[n + 1]) * points[n + 1].y();
            eq_idx++;
        }
        
        // Continuity of second derivative at junctions
        for (int n = 0; n < num_curves - 1; n++) {
            equations(eq_idx, n * 2) = ks[n] * ks[n];
            equations(eq_idx, n * 2 + 1) = -2 * ks[n] * ks[n];
            equations(eq_idx, n * 2 + 2) = 2 * ks[n + 1] * ks[n + 1];
            equations(eq_idx, n * 2 + 3) = -ks[n + 1] * ks[n + 1];
            answer_x(eq_idx) = points[n + 1].x() * (ks[n + 1] * ks[n + 1] - ks[n] * ks[n]);
            answer_y(eq_idx) = points[n + 1].y() * (ks[n + 1] * ks[n + 1] - ks[n] * ks[n]);
            eq_idx++;
        }
        
        // Solve the system
        Eigen::HouseholderQR<MatrixXd> solver(equations);
        VectorXd solution_x = solver.solve(answer_x);
        VectorXd solution_y = solver.solve(answer_y);
        
        // Extract control points
        for (int n = 0; n < num_curves; n++) {
            bezier_segments_[n].p1 = Eigen::Vector2d(solution_x[n * 2], solution_y[n * 2]);
            bezier_segments_[n].p2 = Eigen::Vector2d(solution_x[n * 2 + 1], solution_y[n * 2 + 1]);
        }
    }
}

void BezierTrajectoryPlanner::EvaluateBezier(double s, Eigen::Vector2d* position,
                                            Eigen::Vector2d* tangent, double* curvature) const {
    if (s < 0 || s > 1) {
        s = std::clamp(s, 0.0, 1.0);
    }
    
    size_t num_curves = bezier_segments_.size();
    if (num_curves == 0) return;
    
    // Find which curve to use
    int index = static_cast<int>(s * num_curves);
    if (index == num_curves) {
        index--;
    }
    
    // Local parameter within the curve
    double t = s * num_curves - index;
    
    // Get control points
    const auto& seg = bezier_segments_[index];
    
    // Cubic Bezier formulas
    double t2 = t * t;
    double t3 = t2 * t;
    
    // Position: B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
    if (position) {
        *position = seg.p0 * (-t3 + 3*t2 - 3*t + 1) +
                   seg.p1 * (3*t3 - 6*t2 + 3*t) +
                   seg.p2 * (-3*t3 + 3*t2) +
                   seg.p3 * t3;
    }
    
    // First derivative (tangent)
    if (tangent) {
        *tangent = seg.p0 * (-3*t2 + 6*t - 3) +
                  seg.p1 * (9*t2 - 12*t + 3) +
                  seg.p2 * (-9*t2 + 6*t) +
                  seg.p3 * (3*t2);
        *tangent *= num_curves; // Scale by number of segments
    }
    
    // Curvature
    if (curvature && tangent) {
        // Second derivative
        Eigen::Vector2d d2 = seg.p0 * (-6*t + 6) +
                            seg.p1 * (18*t - 12) +
                            seg.p2 * (-18*t + 6) +
                            seg.p3 * (6*t);
        d2 *= num_curves * num_curves;
        
        // Curvature: κ = |v × a| / |v|³
        double vel_mag = tangent->norm();
        if (vel_mag > 1e-6) {
            double cross = tangent->x() * d2.y() - tangent->y() * d2.x();
            *curvature = std::abs(cross) / (vel_mag * vel_mag * vel_mag);
        } else {
            *curvature = 0.0;
        }
    }
}

void BezierTrajectoryPlanner::ProfileVelocity(double initial_speed, double final_speed) {
    if (bezier_segments_.empty()) {
        return;
    }
    
    const int num_segments = bezier_segments_.size();
    const int points_per_segment = kInterpolationsPerBezier;
    const int num_points = num_segments * points_per_segment + 1;
    
    // Sample the path
    std::vector<Eigen::Vector2d> positions(num_points);
    std::vector<Eigen::Vector2d> tangents(num_points);
    std::vector<double> curvatures(num_points);
    std::vector<double> speeds(num_points, v_max_);
    
    // Set initial and final speeds
    speeds[0] = initial_speed;
    speeds[num_points - 1] = std::min(v_max_, final_speed);
    
    // Sample path and apply curvature constraints
    for (int n = 0; n < num_points; n++) {
        double s = static_cast<double>(n) / (num_points - 1);
        EvaluateBezier(s, &positions[n], &tangents[n], &curvatures[n]);
        
        // Apply curvature constraint: v = sqrt(a_centripetal / curvature)
        if (curvatures[n] > 1e-6) {
            double max_speed_curvature = std::sqrt(a_max_ / curvatures[n]);
            speeds[n] = std::min(speeds[n], max_speed_curvature);
        }
    }
    
    // Forward pass: apply acceleration constraints
    for (int n = 0; n < num_points - 1; n++) {
        double distance = (positions[n + 1] - positions[n]).norm();
        if (distance > 1e-6) {
            // v²_f = v²_i + 2*a*d
            double max_speed_sq = speeds[n] * speeds[n] + 2 * a_max_ * distance;
            speeds[n + 1] = std::min(speeds[n + 1], std::sqrt(max_speed_sq));
        }
    }
    
    // Backward pass: apply deceleration constraints
    for (int n = num_points - 1; n > 0; n--) {
        double distance = (positions[n] - positions[n - 1]).norm();
        if (distance > 1e-6) {
            // v²_i = v²_f + 2*a*d
            double max_speed_sq = speeds[n] * speeds[n] + 2 * a_max_ * distance;
            speeds[n - 1] = std::min(speeds[n - 1], std::sqrt(max_speed_sq));
        }
    }
    
    // Build trajectory instants (position only, angles will be added later)
    trajectory_instants_.clear();
    trajectory_instants_.reserve(num_points);
    
    double time = 0.0;
    for (int n = 0; n < num_points; n++) {
        // Create instant with 2D position (angle will be set later)
        Eigen::Vector3d pose(positions[n].x(), positions[n].y(), 0.0);
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        
        if (tangents[n].norm() > 1e-6 && speeds[n] > 1e-6) {
            Eigen::Vector2d unit_tangent = tangents[n].normalized();
            velocity.head<2>() = unit_tangent * speeds[n];
        }
        
        trajectory_instants_.push_back(RobotInstant(pose, velocity, time));
        
        // Calculate time to next point
        if (n < num_points - 1) {
            double distance = (positions[n + 1] - positions[n]).norm();
            double avg_speed = (speeds[n] + speeds[n + 1]) / 2.0;
            if (avg_speed > 1e-6) {
                time += distance / avg_speed;
            } else {
                time += 0.1; // Default 100ms if stationary
            }
        }
    }
}

void BezierTrajectoryPlanner::PlanAngles(const std::vector<Eigen::Vector3d>& waypoints) {
    if (trajectory_instants_.empty() || waypoints.empty()) {
        return;
    }
    
    // Simple angle planning: interpolate between waypoint angles
    // For each instant, find which waypoint segment it belongs to
    
    for (auto& instant : trajectory_instants_) {
        // Find closest waypoints
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            double dist = (instant.pose.head<2>() - waypoints[i].head<2>()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Simple approach: use the angle from the closest waypoint
        instant.pose[2] = waypoints[closest_idx][2];
        
        // Calculate angular velocity (simplified)
        // This could be improved with proper angle interpolation
        instant.velocity[2] = 0.0;
    }
    
    // Smooth angular velocities
    for (size_t i = 1; i < trajectory_instants_.size(); ++i) {
        double dt = trajectory_instants_[i].timestamp - trajectory_instants_[i-1].timestamp;
        if (dt > 1e-6) {
            double dtheta = NormalizeAngle(trajectory_instants_[i].pose[2] - 
                                         trajectory_instants_[i-1].pose[2]);
            trajectory_instants_[i].velocity[2] = dtheta / dt;
            
            // Apply angular velocity limit
            trajectory_instants_[i].velocity[2] = std::clamp(trajectory_instants_[i].velocity[2],
                                                            -omega_max_, omega_max_);
        }
    }
}

double BezierTrajectoryPlanner::GetTrapezoidalTime(double distance, double v_max, double a_max,
                                                  double v_start, double v_end) const {
    // Simplified trapezoidal profile calculation
    double v_avg = (v_start + v_end) / 2.0;
    
    // Time to accelerate from v_start to v_max
    double t_accel = (v_max - v_start) / a_max;
    double d_accel = v_start * t_accel + 0.5 * a_max * t_accel * t_accel;
    
    // Time to decelerate from v_max to v_end
    double t_decel = (v_max - v_end) / a_max;
    double d_decel = v_max * t_decel - 0.5 * a_max * t_decel * t_decel;
    
    if (d_accel + d_decel > distance) {
        // Triangular profile (never reaches v_max)
        return 2.0 * std::sqrt(distance / a_max);
    } else {
        // Trapezoidal profile
        double d_cruise = distance - d_accel - d_decel;
        return t_accel + d_cruise / v_max + t_decel;
    }
}

Eigen::Vector3d BezierTrajectoryPlanner::Update(const Eigen::Vector3d& current_pose, double current_time) {
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
    
    // Find current instant
    int idx = FindInstantIndex(elapsed_time);
    if (idx < 0 || idx >= static_cast<int>(trajectory_instants_.size())) {
        return Eigen::Vector3d::Zero();
    }
    
    // Get desired state
    RobotInstant desired;
    if (idx == static_cast<int>(trajectory_instants_.size()) - 1) {
        desired = trajectory_instants_[idx];
    } else {
        // Linear interpolation between instants
        const RobotInstant& prev = trajectory_instants_[idx];
        const RobotInstant& next = trajectory_instants_[idx + 1];
        double dt = next.timestamp - prev.timestamp;
        double s = (elapsed_time - prev.timestamp) / dt;
        
        desired.pose = prev.pose + s * (next.pose - prev.pose);
        desired.pose[2] = prev.pose[2] + s * NormalizeAngle(next.pose[2] - prev.pose[2]);
        desired.velocity = prev.velocity + s * (next.velocity - prev.velocity);
    }
    
    // Compute tracking error
    Eigen::Vector3d pose_error = desired.pose - current_pose;
    pose_error[2] = NormalizeAngle(pose_error[2]);
    
    // Feedforward + feedback control
    Eigen::Vector3d velocity_command = desired.velocity + kp_ * pose_error;
    
    // Apply velocity limits
    velocity_command[0] = std::clamp(velocity_command[0], -v_max_, v_max_);
    velocity_command[1] = std::clamp(velocity_command[1], -v_max_, v_max_);
    velocity_command[2] = std::clamp(velocity_command[2], -omega_max_, omega_max_);
    
    return velocity_command;
}

int BezierTrajectoryPlanner::FindInstantIndex(double time) const {
    // Binary search for the instant just before the given time
    int left = 0;
    int right = trajectory_instants_.size() - 1;
    
    while (left < right) {
        int mid = (left + right + 1) / 2;
        if (trajectory_instants_[mid].timestamp <= time) {
            left = mid;
        } else {
            right = mid - 1;
        }
    }
    
    return left;
}

void BezierTrajectoryPlanner::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
}

void BezierTrajectoryPlanner::SetFeedbackGains(double kp, double kd) {
    kp_ = kp;
    kd_ = kd;
}

void BezierTrajectoryPlanner::SetFieldBoundaries(double min_x, double max_x, double min_y, double max_y) {
    field_min_x_ = min_x;
    field_max_x_ = max_x;
    field_min_y_ = min_y;
    field_max_y_ = max_y;
}

void BezierTrajectoryPlanner::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    // Can be used to get current robot state if needed
}

void BezierTrajectoryPlanner::ResetTrajectory() {
    is_trajectory_active_ = false;
    is_trajectory_finished_ = true;
    trajectory_start_time_ = 0.0;
    trajectory_duration_ = 0.0;
    bezier_segments_.clear();
    trajectory_instants_.clear();
    waypoints_.clear();
    path_points_.clear();
}

Eigen::Vector3d BezierTrajectoryPlanner::GetCurrentDesiredPosition() const {
    return GetDesiredPositionAtTime(util::GetCurrentTime());
}

Eigen::Vector3d BezierTrajectoryPlanner::GetDesiredPositionAtTime(double time) const {
    if (!is_trajectory_active_ || trajectory_instants_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed = time - trajectory_start_time_;
    elapsed = std::clamp(elapsed, 0.0, trajectory_duration_);
    
    int idx = FindInstantIndex(elapsed);
    if (idx < 0) {
        return trajectory_instants_.front().pose;
    }
    if (idx >= static_cast<int>(trajectory_instants_.size()) - 1) {
        return trajectory_instants_.back().pose;
    }
    
    // Interpolate
    const RobotInstant& prev = trajectory_instants_[idx];
    const RobotInstant& next = trajectory_instants_[idx + 1];
    double dt = next.timestamp - prev.timestamp;
    double s = (elapsed - prev.timestamp) / dt;
    
    Eigen::Vector3d pose = prev.pose + s * (next.pose - prev.pose);
    pose[2] = prev.pose[2] + s * NormalizeAngle(next.pose[2] - prev.pose[2]);
    
    return pose;
}

double BezierTrajectoryPlanner::NormalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace ctrl