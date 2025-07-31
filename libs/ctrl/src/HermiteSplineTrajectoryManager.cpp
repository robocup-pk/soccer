#include "HermiteSplineTrajectoryManager.h"
#include "RobotManager.h"
#include "Utils.h"
#include <iostream>
#include <cassert>

namespace ctrl {

HermiteSplineTrajectoryManager::HermiteSplineTrajectoryManager() 
    : lookahead_distance_(0.5),
      min_lookahead_distance_(0.1),
      max_lookahead_distance_(2.0),
      speed_scaling_factor_(0.8),
      trajectory_active_(false),
      trajectory_finished_(true) {
    
    hermite_trajectory_ = std::make_unique<CubicHermiteSplineTrajectory>();
    std::cout << "[HermiteSplineTrajectoryManager] Initialized" << std::endl;
}

bool HermiteSplineTrajectoryManager::CreateTrajectoriesFromPath(
    const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    
    if (path_fWorld.size() < 2) {
        std::cerr << "[HermiteSplineTrajectoryManager] Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[HermiteSplineTrajectoryManager] Creating trajectory from " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    // Set the path in the Hermite spline trajectory planner
    bool success = hermite_trajectory_->SetPath(path_fWorld, t_start_s);
    
    if (success) {
        trajectory_active_ = true;
        trajectory_finished_ = false;
        current_waypoints_ = path_fWorld;
    }
    
    return success;
}

std::pair<bool, Eigen::Vector3d> HermiteSplineTrajectoryManager::Update(const Eigen::Vector3d& pose_est) {
    p_fworld_ = pose_est;
    
    if (!trajectory_active_ || trajectory_finished_) {
        std::cout << "[HermiteSplineTrajectoryManager] No active trajectory" << std::endl;
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Check if trajectory is finished
    if (hermite_trajectory_->IsFinished()) {
        trajectory_finished_ = true;
        trajectory_active_ = false;
        std::cout << "[HermiteSplineTrajectoryManager] Trajectory completed" << std::endl;
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Get current time
    double current_time = util::GetCurrentTime();
    
    // Get velocity command from Hermite spline trajectory
    Eigen::Vector3d velocity_fWorld = hermite_trajectory_->Update(pose_est, current_time);
    
    // Apply safety limits
    ApplySafetyLimits(velocity_fWorld);
    
    // Convert to body frame
    Eigen::Vector3d velocity_fBody = util::RotateAboutZ(velocity_fWorld, -pose_est[2]);
    
    std::cout << "[HermiteSplineTrajectoryManager] Velocity command (body): " 
              << velocity_fBody.transpose() << std::endl;
    
    return std::make_pair(false, velocity_fBody);
}

void HermiteSplineTrajectoryManager::Reset() {
    hermite_trajectory_->ResetTrajectory();
    trajectory_active_ = false;
    trajectory_finished_ = true;
    current_waypoints_.clear();
}

void HermiteSplineTrajectoryManager::SetParameters(double lookahead_distance, double speed_scaling) {
    lookahead_distance_ = std::clamp(lookahead_distance, min_lookahead_distance_, max_lookahead_distance_);
    speed_scaling_factor_ = std::clamp(speed_scaling, 0.1, 1.0);
    
    std::cout << "[HermiteSplineTrajectoryManager] Updated parameters: lookahead=" 
              << lookahead_distance_ << ", speed_scaling=" << speed_scaling_factor_ << std::endl;
}

void HermiteSplineTrajectoryManager::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    if (robot_manager) {
        hermite_trajectory_->InitializeFromRobotManager(robot_manager);
    }
}

void HermiteSplineTrajectoryManager::SetVelocityLimits(double v_max, double omega_max) {
    max_linear_velocity_ = v_max;
    max_angular_velocity_ = omega_max;
    
    // Also update the Hermite trajectory planner limits
    hermite_trajectory_->SetLimits(v_max, 3.0, omega_max, 10.0); // Using default accelerations
    
    std::cout << "[HermiteSplineTrajectoryManager] Updated velocity limits: v_max=" 
              << v_max << ", omega_max=" << omega_max << std::endl;
}

void HermiteSplineTrajectoryManager::ApplySafetyLimits(Eigen::Vector3d& velocity) {
    // Apply linear velocity limits
    double linear_speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    if (linear_speed > max_linear_velocity_) {
        double scale = max_linear_velocity_ / linear_speed;
        velocity[0] *= scale;
        velocity[1] *= scale;
    }
    
    // Apply angular velocity limit
    velocity[2] = std::clamp(velocity[2], -max_angular_velocity_, max_angular_velocity_);
    
    // Apply speed scaling factor
    velocity *= speed_scaling_factor_;
}

Eigen::Vector3d HermiteSplineTrajectoryManager::GetVelocityAtT(double current_time_s) {
    if (!trajectory_active_ || trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    // Get velocity from Hermite trajectory
    return hermite_trajectory_->Update(p_fworld_, current_time_s);
}

Eigen::Vector3d HermiteSplineTrajectoryManager::GetPositionAtT(double current_time_s) {
    // This would require implementing position query in CubicHermiteSplineTrajectory
    // For now, return current position
    return p_fworld_;
}

bool HermiteSplineTrajectoryManager::IsTrajectoryActive() const {
    return trajectory_active_ && !trajectory_finished_;
}

double HermiteSplineTrajectoryManager::GetRemainingTime() const {
    if (!trajectory_active_ || trajectory_finished_) {
        return 0.0;
    }
    return hermite_trajectory_->GetRemainingTime();
}

void HermiteSplineTrajectoryManager::SetFeedbackGains(double kp, double kd) {
    hermite_trajectory_->SetFeedbackGains(kp, kd);
}

} // namespace ctrl