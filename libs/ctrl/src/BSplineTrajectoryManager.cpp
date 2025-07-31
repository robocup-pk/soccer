#include "BSplineTrajectoryManager.h"
#include "RobotManager.h"
#include "Utils.h"
#include <iostream>
#include <cassert>

namespace ctrl {

BSplineTrajectoryManager::BSplineTrajectoryManager() 
    : trajectory_active_(false),
      trajectory_finished_(true),
      smoothing_enabled_(true),
      min_waypoint_distance_(0.05) {  // 5cm minimum distance between waypoints
    
    bspline_trajectory_ = std::make_unique<BSplineTrajectory>();
    std::cout << "[BSplineTrajectoryManager] Initialized with B-spline trajectory planner" << std::endl;
}

bool BSplineTrajectoryManager::CreateTrajectoriesFromPath(
    const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    
    if (path_fWorld.size() < 2) {
        std::cerr << "[BSplineTrajectoryManager] Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[BSplineTrajectoryManager] Creating B-spline trajectory from " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    // Preprocess waypoints if smoothing is enabled
    std::vector<Eigen::Vector3d> processed_waypoints;
    if (smoothing_enabled_) {
        processed_waypoints = PreprocessWaypoints(path_fWorld);
    } else {
        processed_waypoints = path_fWorld;
    }
    
    // Set the path in the B-spline trajectory planner
    bool success = bspline_trajectory_->SetPath(processed_waypoints, t_start_s);
    
    if (success) {
        trajectory_active_ = true;
        trajectory_finished_ = false;
        current_waypoints_ = processed_waypoints;
    }
    
    return success;
}

std::vector<Eigen::Vector3d> BSplineTrajectoryManager::PreprocessWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints) {
    
    std::vector<Eigen::Vector3d> processed;
    
    // Always keep the first waypoint
    processed.push_back(waypoints[0]);
    
    // Filter out waypoints that are too close to previous ones
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double distance = (waypoints[i].head<2>() - processed.back().head<2>()).norm();
        
        if (distance >= min_waypoint_distance_ || i == waypoints.size() - 1) {
            // Keep waypoint if it's far enough or if it's the last one
            processed.push_back(waypoints[i]);
        } else {
            std::cout << "[BSplineTrajectoryManager] Filtering out waypoint " << i 
                      << " (too close: " << distance << "m)" << std::endl;
        }
    }
    
    // Add intermediate waypoints for very long segments
    std::vector<Eigen::Vector3d> final_waypoints;
    for (size_t i = 0; i < processed.size() - 1; ++i) {
        final_waypoints.push_back(processed[i]);
        
        Eigen::Vector3d segment = processed[i+1] - processed[i];
        double segment_length = segment.head<2>().norm();
        
        // If segment is too long, add intermediate points
        if (segment_length > 0.5) {  // 50cm max segment length
            int num_intermediate = static_cast<int>(segment_length / 0.3);  // One point every 30cm
            for (int j = 1; j <= num_intermediate; ++j) {
                double t = static_cast<double>(j) / (num_intermediate + 1);
                Eigen::Vector3d intermediate = processed[i] + t * segment;
                // Properly interpolate angle
                intermediate[2] = processed[i][2] + t * NormalizeAngle(processed[i+1][2] - processed[i][2]);
                final_waypoints.push_back(intermediate);
            }
        }
    }
    final_waypoints.push_back(processed.back());
    
    std::cout << "[BSplineTrajectoryManager] Preprocessed " << waypoints.size() 
              << " waypoints to " << final_waypoints.size() << std::endl;
    
    return final_waypoints;
}

std::pair<bool, Eigen::Vector3d> BSplineTrajectoryManager::Update(const Eigen::Vector3d& pose_est) {
    p_fworld_ = pose_est;
    
    if (!trajectory_active_ || trajectory_finished_) {
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Check if trajectory is finished
    if (bspline_trajectory_->IsFinished()) {
        trajectory_finished_ = true;
        trajectory_active_ = false;
        std::cout << "[BSplineTrajectoryManager] Trajectory completed" << std::endl;
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Get current time
    double current_time = util::GetCurrentTime();
    
    // Get velocity command from B-spline trajectory
    Eigen::Vector3d velocity_fWorld = bspline_trajectory_->Update(pose_est, current_time);
    
    // Apply additional smoothing filter if enabled
    if (smoothing_enabled_) {
        ApplySmoothingFilter(velocity_fWorld);
    }
    
    // Apply safety limits
    ApplySafetyLimits(velocity_fWorld);
    
    // Convert to body frame
    Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
    
    return std::make_pair(false, velocity_fBody);
}

void BSplineTrajectoryManager::ApplySmoothingFilter(Eigen::Vector3d& velocity) {
    // Apply exponential moving average filter for additional smoothness
    const double alpha = 0.3;  // Filter coefficient (0 = no change, 1 = no filter)
    
    if (!previous_velocity_initialized_) {
        previous_velocity_ = velocity;
        previous_velocity_initialized_ = true;
    } else {
        velocity = alpha * velocity + (1.0 - alpha) * previous_velocity_;
        previous_velocity_ = velocity;
    }
}

void BSplineTrajectoryManager::ApplySafetyLimits(Eigen::Vector3d& velocity) {
    // Conservative limits for smooth motion
    const double max_linear = 0.7;   // m/s
    const double max_angular = 2.5;  // rad/s
    
    // Apply per-component limits
    velocity[0] = std::clamp(velocity[0], -max_linear, max_linear);
    velocity[1] = std::clamp(velocity[1], -max_linear, max_linear);
    velocity[2] = std::clamp(velocity[2], -max_angular, max_angular);
    
    // Apply linear velocity magnitude limit
    double linear_speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    if (linear_speed > max_linear) {
        double scale = max_linear / linear_speed;
        velocity[0] *= scale;
        velocity[1] *= scale;
    }
}

void BSplineTrajectoryManager::Reset() {
    bspline_trajectory_->ResetTrajectory();
    trajectory_active_ = false;
    trajectory_finished_ = true;
    current_waypoints_.clear();
    previous_velocity_initialized_ = false;
}

void BSplineTrajectoryManager::SetSmoothingEnabled(bool enabled) {
    smoothing_enabled_ = enabled;
    std::cout << "[BSplineTrajectoryManager] Smoothing " 
              << (enabled ? "enabled" : "disabled") << std::endl;
}

void BSplineTrajectoryManager::SetSplineDegree(int degree) {
    bspline_trajectory_->SetSplineDegree(degree);
}

void BSplineTrajectoryManager::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    if (robot_manager) {
        bspline_trajectory_->InitializeFromRobotManager(robot_manager);
    }
}

void BSplineTrajectoryManager::SetVelocityLimits(double v_max, double omega_max) {
    // Pass conservative limits to B-spline trajectory
    bspline_trajectory_->SetLimits(v_max, 1.0, omega_max, 3.0);
    
    std::cout << "[BSplineTrajectoryManager] Updated velocity limits: v_max=" 
              << v_max << ", omega_max=" << omega_max << std::endl;
}

double BSplineTrajectoryManager::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Vector3d BSplineTrajectoryManager::GetVelocityAtT(double current_time_s) {
    if (!trajectory_active_ || trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    return bspline_trajectory_->Update(p_fworld_, current_time_s);
}

Eigen::Vector3d BSplineTrajectoryManager::GetPositionAtT(double current_time_s) {
    // This would require implementing position query in BSplineTrajectory
    return p_fworld_;
}

bool BSplineTrajectoryManager::IsTrajectoryActive() const {
    return trajectory_active_ && !trajectory_finished_;
}

double BSplineTrajectoryManager::GetRemainingTime() const {
    if (!trajectory_active_ || trajectory_finished_) {
        return 0.0;
    }
    return bspline_trajectory_->GetRemainingTime();
}

void BSplineTrajectoryManager::SetFeedbackGains(double kp, double kd) {
    bspline_trajectory_->SetFeedbackGains(kp, kd);
}

void BSplineTrajectoryManager::SetMinWaypointDistance(double distance) {
    min_waypoint_distance_ = std::max(0.01, distance);  // At least 1cm
    std::cout << "[BSplineTrajectoryManager] Set minimum waypoint distance to " 
              << min_waypoint_distance_ << "m" << std::endl;
}

} // namespace ctrl