#include "TrajectoryTracker.h"
#include "AdvancedMotionPlanner.h"  // For AdvancedMotionPlanner
#include "Utils.h"
#include <iostream>

namespace ctrl {

TrajectoryTracker::TrajectoryTracker()
    : motion_planner_(nullptr),
      start_time_(0.0),
      is_finished_(true) {
    
    // PID gains from TIGERs Mannheim (converted from Java)
    pos_pid_.kp = 8.0;   // Strong proportional gain for position
    pos_pid_.ki = 0.5;   // Small integral to handle steady-state errors
    pos_pid_.kd = 0.3;   // Derivative to dampen oscillations
    pos_pid_.integral_clamp = 0.5;
    
    angle_pid_.kp = 6.0;  // Strong proportional gain for orientation
    angle_pid_.ki = 0.2;
    angle_pid_.kd = 0.2;
    angle_pid_.integral_clamp = 0.4;
}

void TrajectoryTracker::setTrajectory(std::shared_ptr<AdvancedMotionPlanner> planner) {
    motion_planner_ = planner;
    start_time_ = util::GetCurrentTime();
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
    
    double elapsed_time = util::GetCurrentTime() - start_time_;
    static double last_time = elapsed_time;
    double dt = elapsed_time - last_time;
    last_time = elapsed_time;
    
    if (dt < 1e-6) dt = 0.02; // Default 50Hz
    
    // --- Step 1: Get feedforward commands from AdvancedMotionPlanner ---
    Eigen::Vector3d desired_position = motion_planner_->getPosition(elapsed_time);
    Eigen::Vector3d desired_velocity = motion_planner_->getVelocity(elapsed_time);
    
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
    return util::RotateAboutZ(world_velocity, -current_pose.z());
}

} // namespace ctrl