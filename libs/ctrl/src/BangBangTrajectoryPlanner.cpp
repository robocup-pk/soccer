#include "BangBangTrajectoryPlanner.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace ctrl {

// ============================================================================
// BangBangTrajectory1D Implementation
// ============================================================================

void BangBangTrajectory1D::Generate(double initial_pos, double target_pos,
                                   double initial_vel, double max_vel, double max_acc) {
    segments_.clear();
    
    double distance = target_pos - initial_pos;
    double direction = (distance >= 0) ? 1.0 : -1.0;
    
    // Use absolute values for calculations
    double abs_distance = std::abs(distance);
    double abs_initial_vel = initial_vel * direction;
    double abs_max_vel = max_vel;
    double abs_max_acc = max_acc;
    
    // Calculate braking distance from current velocity
    double braking_distance = ComputeBrakingDistance(abs_initial_vel, abs_max_acc);
    
    if (braking_distance >= abs_distance) {
        // Need to brake immediately - two segment profile (brake only)
        double t_brake = std::abs(abs_initial_vel) / abs_max_acc;
        
        BangBangSegment brake_segment;
        brake_segment.duration = t_brake;
        brake_segment.acceleration = -direction * abs_max_acc * (abs_initial_vel >= 0 ? 1.0 : -1.0);
        brake_segment.initial_vel = initial_vel;
        brake_segment.initial_pos = initial_pos;
        segments_.push_back(brake_segment);
        
        total_time_ = t_brake;
    } else {
        // Can accelerate first - three segment profile
        GenerateThreeSegmentProfile(initial_pos, target_pos, initial_vel, max_vel, max_acc);
    }
}

double BangBangTrajectory1D::ComputeBrakingDistance(double vel, double max_acc) const {
    return (vel * vel) / (2.0 * max_acc);
}

void BangBangTrajectory1D::GenerateThreeSegmentProfile(double initial_pos, double target_pos,
                                                       double initial_vel, double max_vel, double max_acc) {
    double distance = target_pos - initial_pos;
    double direction = (distance >= 0) ? 1.0 : -1.0;
    
    // Work with signed values
    double signed_max_vel = direction * max_vel;
    double signed_max_acc = direction * max_acc;
    
    // Time to reach max velocity from current velocity
    double t_accel = (signed_max_vel - initial_vel) / signed_max_acc;
    
    // Distance covered during acceleration
    double d_accel = initial_vel * t_accel + 0.5 * signed_max_acc * t_accel * t_accel;
    
    // Distance needed to brake from max velocity
    double d_brake = (signed_max_vel * signed_max_vel) / (2.0 * max_acc);
    
    // Check if we can reach max velocity
    if (d_accel + d_brake <= std::abs(distance)) {
        // Full three-segment profile: accelerate, cruise, brake
        
        // Acceleration segment
        BangBangSegment accel_segment;
        accel_segment.duration = t_accel;
        accel_segment.acceleration = signed_max_acc;
        accel_segment.initial_vel = initial_vel;
        accel_segment.initial_pos = initial_pos;
        segments_.push_back(accel_segment);
        
        // Cruise segment
        double d_cruise = std::abs(distance) - d_accel - d_brake;
        double t_cruise = d_cruise / max_vel;
        
        BangBangSegment cruise_segment;
        cruise_segment.duration = t_cruise;
        cruise_segment.acceleration = 0.0;
        cruise_segment.initial_vel = signed_max_vel;
        cruise_segment.initial_pos = initial_pos + d_accel * direction;
        segments_.push_back(cruise_segment);
        
        // Braking segment
        double t_brake = max_vel / max_acc;
        
        BangBangSegment brake_segment;
        brake_segment.duration = t_brake;
        brake_segment.acceleration = -signed_max_acc;
        brake_segment.initial_vel = signed_max_vel;
        brake_segment.initial_pos = initial_pos + (d_accel + d_cruise) * direction;
        segments_.push_back(brake_segment);
        
        total_time_ = t_accel + t_cruise + t_brake;
    } else {
        // Triangular profile: accelerate then brake without reaching max velocity
        
        // Solve for peak velocity
        double v_peak_squared = initial_vel * initial_vel + 2 * max_acc * std::abs(distance);
        double v_peak = direction * std::sqrt(v_peak_squared);
        
        // Time to reach peak velocity
        double t_accel = (v_peak - initial_vel) / signed_max_acc;
        
        // Time to brake from peak velocity
        double t_brake = std::abs(v_peak) / max_acc;
        
        // Acceleration segment
        BangBangSegment accel_segment;
        accel_segment.duration = t_accel;
        accel_segment.acceleration = signed_max_acc;
        accel_segment.initial_vel = initial_vel;
        accel_segment.initial_pos = initial_pos;
        segments_.push_back(accel_segment);
        
        // Braking segment
        double d_accel = initial_vel * t_accel + 0.5 * signed_max_acc * t_accel * t_accel;
        
        BangBangSegment brake_segment;
        brake_segment.duration = t_brake;
        brake_segment.acceleration = -signed_max_acc;
        brake_segment.initial_vel = v_peak;
        brake_segment.initial_pos = initial_pos + d_accel;
        segments_.push_back(brake_segment);
        
        total_time_ = t_accel + t_brake;
    }
}

double BangBangTrajectory1D::GetPosition(double t) const {
    if (segments_.empty()) return 0.0;
    if (t <= 0) return segments_[0].initial_pos;
    if (t >= total_time_) {
        // Return final position
        const auto& last_segment = segments_.back();
        return last_segment.GetPosition(last_segment.duration);
    }
    
    double accumulated_time = 0.0;
    for (const auto& segment : segments_) {
        if (t <= accumulated_time + segment.duration) {
            double segment_time = t - accumulated_time;
            return segment.GetPosition(segment_time);
        }
        accumulated_time += segment.duration;
    }
    
    return segments_.back().GetPosition(segments_.back().duration);
}

double BangBangTrajectory1D::GetVelocity(double t) const {
    if (segments_.empty()) return 0.0;
    if (t <= 0) return segments_[0].initial_vel;
    if (t >= total_time_) return 0.0;
    
    double accumulated_time = 0.0;
    for (const auto& segment : segments_) {
        if (t <= accumulated_time + segment.duration) {
            double segment_time = t - accumulated_time;
            return segment.GetVelocity(segment_time);
        }
        accumulated_time += segment.duration;
    }
    
    return 0.0;
}

double BangBangTrajectory1D::GetAcceleration(double t) const {
    if (segments_.empty()) return 0.0;
    if (t < 0 || t >= total_time_) return 0.0;
    
    double accumulated_time = 0.0;
    for (const auto& segment : segments_) {
        if (t < accumulated_time + segment.duration) {
            return segment.acceleration;
        }
        accumulated_time += segment.duration;
    }
    
    return 0.0;
}

double BangBangTrajectory1D::GetTotalTime() const {
    return total_time_;
}

int BangBangTrajectory1D::FindSegmentIndex(double t) const {
    if (segments_.empty()) return -1;
    
    double accumulated_time = 0.0;
    for (size_t i = 0; i < segments_.size(); ++i) {
        accumulated_time += segments_[i].duration;
        if (t < accumulated_time) {
            return static_cast<int>(i);
        }
    }
    
    return static_cast<int>(segments_.size() - 1);
}

// ============================================================================
// BangBangTrajectory2D Implementation
// ============================================================================

void BangBangTrajectory2D::Generate(const Eigen::Vector2d& initial_pos,
                                   const Eigen::Vector2d& target_pos,
                                   const Eigen::Vector2d& initial_vel,
                                   double max_vel,
                                   double max_acc) {
    // Generate independent trajectories for X and Y
    x_trajectory_.Generate(initial_pos.x(), target_pos.x(), initial_vel.x(), max_vel, max_acc);
    y_trajectory_.Generate(initial_pos.y(), target_pos.y(), initial_vel.y(), max_vel, max_acc);
    
    // Synchronize to finish at the same time
    SynchronizeTrajectories(max_vel, max_acc);
}

void BangBangTrajectory2D::SynchronizeTrajectories(double max_vel, double max_acc) {
    double t_x = x_trajectory_.GetTotalTime();
    double t_y = y_trajectory_.GetTotalTime();
    
    if (std::abs(t_x - t_y) < 1e-6) {
        return;  // Already synchronized
    }
    
    // Find the slower trajectory
    double max_time = std::max(t_x, t_y);
    
    // Rescale the faster trajectory to match the slower one
    if (t_x < t_y) {
        // X is faster, need to slow it down
        double scale_factor = t_y / t_x;
        double scaled_max_vel = max_vel / scale_factor;
        double scaled_max_acc = max_acc / (scale_factor * scale_factor);
        
        double initial_pos = x_trajectory_.GetPosition(0);
        double target_pos = x_trajectory_.GetPosition(t_x);
        double initial_vel = x_trajectory_.GetVelocity(0) / scale_factor;
        
        x_trajectory_.Generate(initial_pos, target_pos, initial_vel, scaled_max_vel, scaled_max_acc);
    } else {
        // Y is faster, need to slow it down
        double scale_factor = t_x / t_y;
        double scaled_max_vel = max_vel / scale_factor;
        double scaled_max_acc = max_acc / (scale_factor * scale_factor);
        
        double initial_pos = y_trajectory_.GetPosition(0);
        double target_pos = y_trajectory_.GetPosition(t_y);
        double initial_vel = y_trajectory_.GetVelocity(0) / scale_factor;
        
        y_trajectory_.Generate(initial_pos, target_pos, initial_vel, scaled_max_vel, scaled_max_acc);
    }
}

Eigen::Vector2d BangBangTrajectory2D::GetPosition(double t) const {
    return Eigen::Vector2d(x_trajectory_.GetPosition(t), y_trajectory_.GetPosition(t));
}

Eigen::Vector2d BangBangTrajectory2D::GetVelocity(double t) const {
    return Eigen::Vector2d(x_trajectory_.GetVelocity(t), y_trajectory_.GetVelocity(t));
}

Eigen::Vector2d BangBangTrajectory2D::GetAcceleration(double t) const {
    return Eigen::Vector2d(x_trajectory_.GetAcceleration(t), y_trajectory_.GetAcceleration(t));
}

double BangBangTrajectory2D::GetTotalTime() const {
    return std::max(x_trajectory_.GetTotalTime(), y_trajectory_.GetTotalTime());
}

// ============================================================================
// BangBangTrajectoryPlanner Implementation
// ============================================================================

BangBangTrajectoryPlanner::BangBangTrajectoryPlanner() {
    // Constructor - parameters initialized in header
}

void BangBangTrajectoryPlanner::SetLimits(double max_vel, double max_acc) {
    max_vel_ = max_vel;
    max_acc_ = max_acc;
}

void BangBangTrajectoryPlanner::SetFieldBoundaries(double min_x, double max_x, double min_y, double max_y) {
    field_min_x_ = min_x;
    field_max_x_ = max_x;
    field_min_y_ = min_y;
    field_max_y_ = max_y;
}

bool BangBangTrajectoryPlanner::SetPath(const std::vector<Eigen::Vector3d>& waypoints, double start_time) {
    if (waypoints.size() < 2) {
        std::cerr << "[BangBangTrajectoryPlanner] Need at least 2 waypoints" << std::endl;
        return false;
    }
    
    waypoints_ = waypoints;
    current_waypoint_index_ = 0;
    trajectory_start_time_ = start_time;
    is_finished_ = false;
    has_trajectory_ = false;
    
    // Plan trajectory to first waypoint
    Eigen::Vector2d start_pos = waypoints[0].head<2>();
    Eigen::Vector2d goal_pos = waypoints[1].head<2>();
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    
    bool success = PlanTrajectory(start_pos, start_vel, goal_pos);
    if (success) {
        current_waypoint_index_ = 1;
        has_trajectory_ = true;
        std::cout << "[BangBangTrajectoryPlanner] Initial trajectory planned successfully" << std::endl;
    }
    
    return success;
}

Eigen::Vector3d BangBangTrajectoryPlanner::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (is_finished_ || !has_trajectory_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    double trajectory_time = current_trajectory_.GetTotalTime();
    
    // Check if current trajectory segment is complete
    if (elapsed_time >= trajectory_time) {
        // Move to next waypoint
        current_waypoint_index_++;
        
        if (current_waypoint_index_ >= waypoints_.size()) {
            // Reached final waypoint
            is_finished_ = true;
            std::cout << "[BangBangTrajectoryPlanner] Trajectory complete" << std::endl;
            return Eigen::Vector3d::Zero();
        }
        
        // Plan trajectory to next waypoint
        Eigen::Vector2d current_vel = current_trajectory_.GetVelocity(trajectory_time);
        Eigen::Vector2d next_goal = waypoints_[current_waypoint_index_].head<2>();
        
        if (PlanTrajectory(current_pose.head<2>(), current_vel, next_goal)) {
            trajectory_start_time_ = current_time;
            elapsed_time = 0;
            std::cout << "[BangBangTrajectoryPlanner] Moving to waypoint " << current_waypoint_index_ << std::endl;
        } else {
            std::cerr << "[BangBangTrajectoryPlanner] Failed to plan trajectory to next waypoint" << std::endl;
            is_finished_ = true;
            return Eigen::Vector3d::Zero();
        }
    }
    
    // Get desired position and velocity from trajectory
    Eigen::Vector2d desired_pos = current_trajectory_.GetPosition(elapsed_time);
    Eigen::Vector2d desired_vel = current_trajectory_.GetVelocity(elapsed_time);
    
    // Compute feedback control
    return ComputeFeedbackControl(current_pose, desired_pos, desired_vel);
}

bool BangBangTrajectoryPlanner::CheckTrajectoryCollision(const BangBangTrajectory2D& trajectory,
                                                        double time_offset) const {
    double dt = 0.01;  // 10ms time step for collision checking
    double total_time = trajectory.GetTotalTime();
    
    for (double t = 0; t <= total_time; t += dt) {
        Eigen::Vector2d robot_pos = trajectory.GetPosition(t);
        
        // Check field boundaries
        if (!IsWithinField(robot_pos)) {
            return true;  // Collision with field boundary
        }
        
        // Check obstacles
        for (const auto& obstacle : obstacles_) {
            Eigen::Vector2d obs_pos = obstacle.GetPositionAtTime(t + time_offset);
            double distance = (robot_pos - obs_pos).norm();
            double min_distance = robot_radius_ + obstacle.radius;
            
            if (distance < min_distance) {
                return true;  // Collision detected
            }
        }
    }
    
    return false;  // No collision
}

double BangBangTrajectoryPlanner::GetFirstCollisionTime(const BangBangTrajectory2D& trajectory,
                                                       double time_offset) const {
    double dt = 0.01;  // 10ms time step
    double total_time = trajectory.GetTotalTime();
    
    for (double t = 0; t <= total_time; t += dt) {
        Eigen::Vector2d robot_pos = trajectory.GetPosition(t);
        
        if (!IsWithinField(robot_pos)) {
            return t;
        }
        
        for (const auto& obstacle : obstacles_) {
            Eigen::Vector2d obs_pos = obstacle.GetPositionAtTime(t + time_offset);
            double distance = (robot_pos - obs_pos).norm();
            double min_distance = robot_radius_ + obstacle.radius;
            
            if (distance < min_distance) {
                return t;
            }
        }
    }
    
    return total_time;  // No collision found
}

std::vector<Eigen::Vector2d> BangBangTrajectoryPlanner::GenerateSubDestinations(
    const Eigen::Vector2d& start, const Eigen::Vector2d& goal) const {
    
    std::vector<Eigen::Vector2d> sub_destinations;
    
    // Generate intermediate points around obstacles
    double angle_step = M_PI / 4;  // 45 degree steps
    double offset_distance = 0.3;  // 30cm offset from obstacles
    
    // Calculate path direction (will be used later as well)
    Eigen::Vector2d to_goal = goal - start;
    double path_length = to_goal.norm();
    Eigen::Vector2d path_dir = to_goal / path_length;
    
    for (const auto& obstacle : obstacles_) {
        // Check if obstacle is near the direct path
        
        // Project obstacle onto path
        Eigen::Vector2d to_obstacle = obstacle.position - start;
        double projection = to_obstacle.dot(path_dir);
        
        if (projection > 0 && projection < path_length) {
            // Obstacle is along the path
            Eigen::Vector2d closest_point = start + path_dir * projection;
            double distance_to_path = (obstacle.position - closest_point).norm();
            
            if (distance_to_path < robot_radius_ + obstacle.radius + 0.1) {
                // Generate avoidance points around obstacle
                for (double angle = 0; angle < 2 * M_PI; angle += angle_step) {
                    double avoid_radius = obstacle.radius + robot_radius_ + offset_distance;
                    Eigen::Vector2d avoid_point(
                        obstacle.position.x() + avoid_radius * cos(angle),
                        obstacle.position.y() + avoid_radius * sin(angle)
                    );
                    
                    if (IsWithinField(avoid_point)) {
                        sub_destinations.push_back(avoid_point);
                    }
                }
            }
        }
    }
    
    // Also add some intermediate points along edges if direct path is blocked
    if (!sub_destinations.empty()) {
        // Add points at 25%, 50%, 75% of the way
        for (double fraction : {0.25, 0.5, 0.75}) {
            Eigen::Vector2d intermediate = start + (goal - start) * fraction;
            
            // Offset perpendicular to path
            Eigen::Vector2d perpendicular(-path_dir.y(), path_dir.x());
            sub_destinations.push_back(intermediate + perpendicular * 0.2);
            sub_destinations.push_back(intermediate - perpendicular * 0.2);
        }
    }
    
    return sub_destinations;
}

bool BangBangTrajectoryPlanner::PlanTrajectory(const Eigen::Vector2d& start_pos,
                                              const Eigen::Vector2d& start_vel,
                                              const Eigen::Vector2d& goal_pos) {
    // First try direct path
    current_trajectory_.Generate(start_pos, goal_pos, start_vel, max_vel_, max_acc_);
    
    if (!CheckTrajectoryCollision(current_trajectory_, 0.0)) {
        return true;  // Direct path is collision-free
    }
    
    std::cout << "[BangBangTrajectoryPlanner] Direct path has collision, trying sub-destinations" << std::endl;
    
    // Try sub-destinations
    auto sub_destinations = GenerateSubDestinations(start_pos, goal_pos);
    
    for (const auto& sub_dest : sub_destinations) {
        // Plan to sub-destination
        BangBangTrajectory2D sub_trajectory;
        sub_trajectory.Generate(start_pos, sub_dest, start_vel, max_vel_, max_acc_);
        
        if (!CheckTrajectoryCollision(sub_trajectory, 0.0)) {
            // Found collision-free path to sub-destination
            // Now check if we can reach goal from there
            Eigen::Vector2d sub_vel = sub_trajectory.GetVelocity(sub_trajectory.GetTotalTime());
            BangBangTrajectory2D goal_trajectory;
            goal_trajectory.Generate(sub_dest, goal_pos, sub_vel, max_vel_, max_acc_);
            
            if (!CheckTrajectoryCollision(goal_trajectory, sub_trajectory.GetTotalTime())) {
                // Success! Use the sub-destination path
                current_trajectory_ = sub_trajectory;
                std::cout << "[BangBangTrajectoryPlanner] Found path via sub-destination" << std::endl;
                return true;
            }
        }
    }
    
    std::cerr << "[BangBangTrajectoryPlanner] Could not find collision-free path" << std::endl;
    return false;
}

bool BangBangTrajectoryPlanner::IsWithinField(const Eigen::Vector2d& pos) const {
    double margin = robot_radius_;
    return pos.x() >= field_min_x_ + margin &&
           pos.x() <= field_max_x_ - margin &&
           pos.y() >= field_min_y_ + margin &&
           pos.y() <= field_max_y_ - margin;
}

Eigen::Vector3d BangBangTrajectoryPlanner::ComputeFeedbackControl(
    const Eigen::Vector3d& current_pose,
    const Eigen::Vector2d& desired_pos,
    const Eigen::Vector2d& desired_vel) const {
    
    // Position error (convert to meters since trajectory is in meters)
    Eigen::Vector2d pos_error = desired_pos - current_pose.head<2>();
    
    // Limit position error to prevent excessive correction
    double max_error = 0.5;  // 50cm max error for stability
    if (pos_error.norm() > max_error) {
        pos_error = pos_error.normalized() * max_error;
    }
    
    // Proportional control with feedforward
    Eigen::Vector2d velocity_command = desired_vel + kp_ * pos_error;
    
    // Apply velocity limits (robot can only move at 1 m/s max)
    double max_robot_vel = 0.8;  // Use 80% of max to have some margin
    double vel_magnitude = velocity_command.norm();
    if (vel_magnitude > max_robot_vel) {
        velocity_command = velocity_command.normalized() * max_robot_vel;
    }
    
    // Compute desired heading from velocity direction
    double desired_heading = 0.0;
    if (velocity_command.norm() > 0.05) {  // Reduced threshold
        desired_heading = std::atan2(velocity_command.y(), velocity_command.x());
    } else {
        // Keep current heading if velocity is small
        desired_heading = current_pose.z();
    }
    
    // Angular control with limited gain
    double heading_error = desired_heading - current_pose.z();
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    
    // Use smaller angular gain to prevent oscillation
    double angular_gain = 3.0;  // Much smaller than kp_
    double omega = angular_gain * heading_error;
    
    // Limit angular velocity (max 5 rad/s, use 4 for safety)
    double max_omega = 4.0;
    omega = std::clamp(omega, -max_omega, max_omega);
    
    // Transform to body frame
    double cos_theta = cos(current_pose.z());
    double sin_theta = sin(current_pose.z());
    double vx_body = velocity_command.x() * cos_theta + velocity_command.y() * sin_theta;
    double vy_body = -velocity_command.x() * sin_theta + velocity_command.y() * cos_theta;
    
    // Final safety check on body velocities
    vx_body = std::clamp(vx_body, -0.9, 0.9);
    vy_body = std::clamp(vy_body, -0.9, 0.9);
    
    return Eigen::Vector3d(vx_body, vy_body, omega);
}

} // namespace ctrl