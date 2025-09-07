#include "TrajectoryTracker.h"
#include "AdvancedMotionPlanner.h"  // For AdvancedMotionPlanner
#include "Utils.h"
#include <iostream>

namespace ctrl {

TrajectoryTracker::TrajectoryTracker()
    : motion_planner_(nullptr),
      start_time_(0.0),
      last_update_time_(0.0),
      is_finished_(true) {
    
    // PID gains tuned for SSL robots (more conservative than original Team)
    pos_pid_.kp = 8.0;   // Moderate proportional gain for position
    pos_pid_.ki = 0.5;   // Small integral to handle steady-state errors
    pos_pid_.kd = 0.1;   // Small derivative to dampen oscillations
    pos_pid_.integral_clamp = 0.3;
    
    angle_pid_.kp = 6.0;  // Moderate proportional gain for orientation
    angle_pid_.ki = 0.1;
    angle_pid_.kd = 0.1;
    angle_pid_.integral_clamp = 0.2;
}

void TrajectoryTracker::setTrajectory(std::shared_ptr<AdvancedMotionPlanner> planner) {
    motion_planner_ = planner;
    start_time_ = util::GetCurrentTime();
    last_update_time_ = start_time_;  // Initialize to start time
    is_finished_ = false;
    
    // Reset PID controllers
    pos_pid_.reset();
    angle_pid_.reset();
}

bool TrajectoryTracker::isFinished() const {
    if (!motion_planner_) return true;
    
    double elapsed_time = util::GetCurrentTime() - start_time_;
    return elapsed_time >= motion_planner_->getTotalTime();
}

Eigen::Vector3d TrajectoryTracker::update(const Eigen::Vector3d& current_pose) {
    if (isFinished() || !motion_planner_) {
        return Eigen::Vector3d::Zero();
    }
    
    double current_time = util::GetCurrentTime();
    double elapsed_time = current_time - start_time_;
    
    // Calculate dt properly - track previous update time
    double dt = current_time - last_update_time_;
    last_update_time_ = current_time;
    
    if (dt < 1e-6 || dt > 0.1) dt = 0.02; // Default 50Hz, clamp large dt
    
    // --- Step 1: Get feedforward commands from AdvancedMotionPlanner ---
    // This matches exactly how Advanced's MoveBangBangSkill works (lines 92, 111)
    Eigen::Vector3d desired_position = motion_planner_->getPosition(elapsed_time);
    Eigen::Vector3d desired_velocity = motion_planner_->getVelocity(elapsed_time);
    
    // Debug: Check raw trajectory velocity
    if (desired_velocity.head<2>().norm() > 1.4) {
        std::cout << "[TrajectoryTracker] WARNING: Raw trajectory velocity too high at t=" << elapsed_time 
                  << "s: " << desired_velocity.transpose() << std::endl;
    }
    
    // --- Step 2: Calculate position and orientation errors ---
    Eigen::Vector2d pos_error = desired_position.head<2>() - current_pose.head<2>();
    double angle_error = util::WrapAngle(desired_position.z() - current_pose.z());
    
    // --- Step 3: Calculate PID corrections (feedback) ---
    Eigen::Vector2d pos_correction = pos_pid_.calculate(pos_error, dt);
    double angle_correction = angle_pid_.calculate(Eigen::Vector2d(angle_error, 0), dt).x();
    
    // --- Step 4: Combine feedforward + feedback ---
    Eigen::Vector3d world_velocity;
    world_velocity.head<2>() = desired_velocity.head<2>() + pos_correction;
    world_velocity.z() = desired_velocity.z() + angle_correction;
    
    // --- Step 5: Convert to body frame ---
    Eigen::Vector3d body_velocity = util::RotateAboutZ(world_velocity, -current_pose.z());
    
    // --- Step 6: Clamp velocities to robot limits ---
    // Use SystemConfig limits: (1.5, 1.5, 5.0) m/s
    body_velocity.x() = std::clamp(body_velocity.x(), -1.4, 1.4);  // Slightly under limit for safety
    body_velocity.y() = std::clamp(body_velocity.y(), -1.4, 1.4);
    body_velocity.z() = std::clamp(body_velocity.z(), -4.8, 4.8);
    
    return body_velocity;
}

} // namespace ctrl