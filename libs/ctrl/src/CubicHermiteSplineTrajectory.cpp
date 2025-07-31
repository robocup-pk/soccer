#include "CubicHermiteSplineTrajectory.h"
#include "RobotDescription.h"
#include "RobotManager.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace ctrl {

CubicHermiteSplineTrajectory::CubicHermiteSplineTrajectory() 
    : robot_description_(kin::GetRobotDescription()),
      trajectory_start_time_(0.0),
      is_trajectory_active_(false),
      is_trajectory_finished_(true),
      current_segment_index_(0) {
    
    std::cout << "[CubicHermiteSplineTrajectory] Initialized with " 
              << robot_description_.wheel_angles_rad.size() << " wheels" << std::endl;
}

bool CubicHermiteSplineTrajectory::SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    if (path_fWorld.size() < 2) {
        std::cerr << "[CubicHermiteSplineTrajectory::SetPath] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[CubicHermiteSplineTrajectory::SetPath] Generating trajectory for " << path_fWorld.size() << " waypoints" << std::endl;
    
    // Print the path
    for (size_t i = 0; i < path_fWorld.size() - 1; ++i) {
        std::cout << "  " << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld.back().transpose() << std::endl;
    
    try {
        // Clear previous trajectory
        segments_.clear();
        waypoints_ = path_fWorld;
        
        // Generate cubic Hermite spline segments
        GenerateHermiteSplineSegments();
        
        if (segments_.empty()) {
            std::cerr << "[CubicHermiteSplineTrajectory::SetPath] Error: Failed to generate trajectory segments" << std::endl;
            return false;
        }
        
        trajectory_start_time_ = (t_start_s > 0) ? t_start_s : util::GetCurrentTime();
        is_trajectory_active_ = true;
        is_trajectory_finished_ = false;
        current_segment_index_ = 0;
        
        // Calculate total trajectory time
        total_trajectory_time_ = 0.0;
        for (const auto& segment : segments_) {
            total_trajectory_time_ += segment.duration;
        }
        
        std::cout << "[CubicHermiteSplineTrajectory::SetPath] Trajectory generated successfully. "
                  << "Segments: " << segments_.size() 
                  << ", Total duration: " << total_trajectory_time_ << "s" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[CubicHermiteSplineTrajectory::SetPath] Exception: " << e.what() << std::endl;
        return false;
    }
}

void CubicHermiteSplineTrajectory::GenerateHermiteSplineSegments() {
    if (waypoints_.size() < 2) return;
    
    // Calculate tangents at each waypoint using finite differences
    std::vector<Eigen::Vector3d> tangents;
    tangents.reserve(waypoints_.size());
    
    // First waypoint tangent
    if (waypoints_.size() == 2) {
        // Only two points: use straight line direction
        Eigen::Vector3d dir = waypoints_[1] - waypoints_[0];
        tangents.push_back(dir.normalized() * v_max_ * 0.5);
    } else {
        // Use forward difference
        Eigen::Vector3d t0 = ComputeTangent(waypoints_[0], waypoints_[1], waypoints_[2]);
        tangents.push_back(t0);
    }
    
    // Interior waypoints tangents
    for (size_t i = 1; i < waypoints_.size() - 1; ++i) {
        Eigen::Vector3d t = ComputeTangent(waypoints_[i-1], waypoints_[i], waypoints_[i+1]);
        tangents.push_back(t);
    }
    
    // Last waypoint tangent
    if (waypoints_.size() == 2) {
        // Only two points: use straight line direction
        Eigen::Vector3d dir = waypoints_[1] - waypoints_[0];
        tangents.push_back(dir.normalized() * v_max_ * 0.5);
    } else {
        // Use backward difference
        size_t n = waypoints_.size();
        Eigen::Vector3d tn = ComputeTangent(waypoints_[n-3], waypoints_[n-2], waypoints_[n-1]);
        tangents.push_back(tn);
    }
    
    // Create Hermite spline segments between consecutive waypoints
    for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
        HermiteSegment segment;
        segment.p0 = waypoints_[i];
        segment.p1 = waypoints_[i+1];
        segment.m0 = tangents[i];
        segment.m1 = tangents[i+1];
        
        // Compute segment duration based on arc length approximation
        double distance = (segment.p1 - segment.p0).norm();
        double avg_velocity = v_max_ * 0.7; // Use 70% of max velocity as average
        segment.duration = std::max(0.1, distance / avg_velocity);
        
        // Pre-compute Hermite coefficients for efficiency
        ComputeHermiteCoefficients(segment);
        
        segments_.push_back(segment);
    }
}

Eigen::Vector3d CubicHermiteSplineTrajectory::ComputeTangent(
    const Eigen::Vector3d& p_prev, 
    const Eigen::Vector3d& p_curr, 
    const Eigen::Vector3d& p_next) {
    
    // Catmull-Rom tangent calculation
    Eigen::Vector3d v1 = p_curr - p_prev;
    Eigen::Vector3d v2 = p_next - p_curr;
    
    // Normalize direction vectors
    double d1 = v1.norm();
    double d2 = v2.norm();
    
    if (d1 < 1e-6 || d2 < 1e-6) {
        // Degenerate case: use average direction
        return (v1 + v2) * 0.5;
    }
    
    // Weighted average based on distances
    Eigen::Vector3d tangent = (v1/d1 + v2/d2).normalized();
    
    // Scale tangent by velocity
    double speed = std::min(v_max_, (d1 + d2) / 2.0);
    
    // Special handling for angular component
    tangent[2] = NormalizeAngle(p_next[2] - p_prev[2]) / 2.0;
    
    return tangent * speed * 0.5; // Scale factor for smoother curves
}

void CubicHermiteSplineTrajectory::ComputeHermiteCoefficients(HermiteSegment& segment) {
    // Hermite basis functions coefficients
    // h(t) = c0 + c1*t + c2*t^2 + c3*t^3
    // where t is normalized to [0, 1]
    
    segment.c0 = segment.p0;
    segment.c1 = segment.m0;
    segment.c2 = 3.0 * (segment.p1 - segment.p0) - 2.0 * segment.m0 - segment.m1;
    segment.c3 = 2.0 * (segment.p0 - segment.p1) + segment.m0 + segment.m1;
}

Eigen::Vector3d CubicHermiteSplineTrajectory::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    
    // Check if trajectory is finished
    if (elapsed_time >= total_trajectory_time_) {
        is_trajectory_finished_ = true;
        is_trajectory_active_ = false;
        std::cout << "[CubicHermiteSplineTrajectory::Update] Trajectory execution completed" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    // Find current segment
    double segment_start_time = 0.0;
    size_t segment_idx = 0;
    for (size_t i = 0; i < segments_.size(); ++i) {
        if (elapsed_time < segment_start_time + segments_[i].duration) {
            segment_idx = i;
            break;
        }
        segment_start_time += segments_[i].duration;
    }
    
    current_segment_index_ = segment_idx;
    
    // Evaluate spline at current time
    double segment_time = elapsed_time - segment_start_time;
    double t = segment_time / segments_[segment_idx].duration; // Normalize to [0, 1]
    t = std::clamp(t, 0.0, 1.0);
    
    auto [position, velocity, acceleration] = EvaluateHermiteSegment(segments_[segment_idx], t);
    
    // Apply feedback control for trajectory tracking
    Eigen::Vector3d position_error = position - current_pose;
    position_error[2] = NormalizeAngle(position_error[2]); // Normalize angular error
    
    // PD controller for trajectory tracking
    Eigen::Vector3d velocity_command = velocity + kp_ * position_error;
    
    // Apply velocity limits
    velocity_command[0] = std::clamp(velocity_command[0], -v_max_, v_max_);
    velocity_command[1] = std::clamp(velocity_command[1], -v_max_, v_max_);
    velocity_command[2] = std::clamp(velocity_command[2], -omega_max_, omega_max_);
    
    return velocity_command;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> 
CubicHermiteSplineTrajectory::EvaluateHermiteSegment(const HermiteSegment& segment, double t) {
    // Evaluate position using Hermite polynomial
    double t2 = t * t;
    double t3 = t2 * t;
    
    Eigen::Vector3d position = segment.c0 + segment.c1 * t + segment.c2 * t2 + segment.c3 * t3;
    
    // Evaluate velocity (first derivative)
    double dt = 1.0 / segment.duration; // Convert from normalized time to actual time
    Eigen::Vector3d velocity = (segment.c1 + 2.0 * segment.c2 * t + 3.0 * segment.c3 * t2) * dt;
    
    // Evaluate acceleration (second derivative)
    double dt2 = dt * dt;
    Eigen::Vector3d acceleration = (2.0 * segment.c2 + 6.0 * segment.c3 * t) * dt2;
    
    // Normalize angular position
    position[2] = NormalizeAngle(position[2]);
    
    return std::make_tuple(position, velocity, acceleration);
}

bool CubicHermiteSplineTrajectory::AddGoal(const Eigen::Vector3d& goal) {
    std::cout << "[CubicHermiteSplineTrajectory::AddGoal] Adding goal: " << goal.transpose() << std::endl;
    
    if (!is_trajectory_active_) {
        // Start new trajectory from current position to goal
        std::vector<Eigen::Vector3d> path = {Eigen::Vector3d::Zero(), goal};
        return SetPath(path);
    } else {
        // For now, we don't support dynamic goal addition
        // This would require re-planning the trajectory
        std::cout << "[CubicHermiteSplineTrajectory::AddGoal] Dynamic goal addition not yet supported" << std::endl;
        return false;
    }
}

double CubicHermiteSplineTrajectory::GetRemainingTime() const {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return 0.0;
    }
    
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    return std::max(0.0, total_trajectory_time_ - elapsed);
}

void CubicHermiteSplineTrajectory::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
    
    std::cout << "[CubicHermiteSplineTrajectory] Updated limits: v_max=" << v_max_ 
              << ", a_max=" << a_max_ << ", ω_max=" << omega_max_ 
              << ", α_max=" << alpha_max_ << std::endl;
}

void CubicHermiteSplineTrajectory::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    if (!robot_manager) {
        std::cerr << "[CubicHermiteSplineTrajectory] Error: RobotManager is null" << std::endl;
        return;
    }
    
    // Get current robot state
    Eigen::Vector3d current_pose = robot_manager->GetPoseInWorldFrame();
    Eigen::Vector3d current_velocity = robot_manager->GetVelocityInWorldFrame();
    
    std::cout << "[CubicHermiteSplineTrajectory] Initialized from RobotManager:" << std::endl;
    std::cout << "  Current pose: " << current_pose.transpose() << std::endl;
    std::cout << "  Current velocity: " << current_velocity.transpose() << std::endl;
}

double CubicHermiteSplineTrajectory::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void CubicHermiteSplineTrajectory::ResetTrajectory() {
    is_trajectory_active_ = false;
    is_trajectory_finished_ = true;
    trajectory_start_time_ = 0.0;
    current_segment_index_ = 0;
    segments_.clear();
    waypoints_.clear();
}

void CubicHermiteSplineTrajectory::SetFeedbackGains(double kp, double kd) {
    kp_ = kp;
    kd_ = kd;
    std::cout << "[CubicHermiteSplineTrajectory] Updated feedback gains: kp=" << kp_ << ", kd=" << kd_ << std::endl;
}

} // namespace ctrl