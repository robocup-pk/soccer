#include "M_TrajectoryController.h"
#include "Utils.h"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace ctrl {

//=============================================================================
// M_TrajectoryController Implementation
//=============================================================================

M_TrajectoryController::M_TrajectoryController() 
    : current_state_(M_TrajectoryState::IDLE)
    , trajectory_start_time_(0.0)
    , current_time_(0.0)
    , robot_position_(Eigen::Vector3d::Zero())
    , robot_velocity_(Eigen::Vector3d::Zero())
    , desired_velocity_(Eigen::Vector3d::Zero())
    , desired_position_(Eigen::Vector3d::Zero())
    , desired_acceleration_(Eigen::Vector3d::Zero())
    , position_error_(Eigen::Vector3d::Zero())
    , velocity_error_(Eigen::Vector3d::Zero())
    , emergency_stop_(false)
    , logging_enabled_(false) {
    
    // Set default configuration
    config_.position_tolerance = 0.01;
    config_.velocity_tolerance = 0.1;
    config_.angle_tolerance = 0.05;
    config_.lookahead_time = 0.05;
    config_.max_velocity_correction = 0.5;
    config_.proportional_gain = 0.8;
    config_.enable_velocity_feedforward = true;
    config_.enable_position_correction = true;
}

M_TrajectoryController::M_TrajectoryController(const M_ControllerConfig& config)
    : M_TrajectoryController() {
    config_ = config;
}

void M_TrajectoryController::setConfig(const M_ControllerConfig& config) {
    config_ = config;
}

void M_TrajectoryController::setTrajectory(std::shared_ptr<M_Trajectory> trajectory, bool immediate) {
    if (!trajectory) {
        std::cerr << "[M_TrajectoryController] Error: Null trajectory provided" << std::endl;
        return;
    }
    
    if (immediate) {
        stopCurrentTrajectory();
        startTrajectory(trajectory);
    } else {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        M_TrajectoryQueueEntry entry;
        entry.trajectory = trajectory;
        entry.start_time = current_time_;
        entry.immediate_execution = false;
        trajectory_queue_.push(entry);
    }
}

void M_TrajectoryController::addTrajectoryToQueue(
    std::shared_ptr<M_Trajectory> trajectory, double start_time, const std::string& id) {
    
    if (!trajectory) {
        std::cerr << "[M_TrajectoryController] Error: Null trajectory provided to queue" << std::endl;
        return;
    }
    
    std::lock_guard<std::mutex> lock(queue_mutex_);
    M_TrajectoryQueueEntry entry;
    entry.trajectory = trajectory;
    entry.start_time = (start_time < 0) ? current_time_ : start_time;
    entry.immediate_execution = false;
    entry.id = id;
    trajectory_queue_.push(entry);
    
    if (logging_enabled_) {
        std::cout << "[M_TrajectoryController] Added trajectory to queue: " << id 
                  << " (start_time: " << entry.start_time << "s)" << std::endl;
    }
}

void M_TrajectoryController::clearTrajectoryQueue() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!trajectory_queue_.empty()) {
        trajectory_queue_.pop();
    }
    
    if (logging_enabled_) {
        std::cout << "[M_TrajectoryController] Trajectory queue cleared" << std::endl;
    }
}

void M_TrajectoryController::stopCurrentTrajectory() {
    if (current_state_ == M_TrajectoryState::EXECUTING) {
        current_state_ = M_TrajectoryState::FINISHED;
        current_trajectory_.reset();
        
        if (logging_enabled_) {
            std::cout << "[M_TrajectoryController] Current trajectory stopped" << std::endl;
        }
    }
}

void M_TrajectoryController::updateRobotState(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    robot_position_ = position;
    robot_velocity_ = velocity;
}

void M_TrajectoryController::updateTime(double current_time) {
    current_time_ = current_time;
    
    if (emergency_stop_) {
        desired_velocity_ = Eigen::Vector3d::Zero();
        desired_acceleration_ = Eigen::Vector3d::Zero();
        return;
    }
    
    processTrajectoryQueue();
    updateTrajectoryExecution();
    calculateDesiredOutputs();
    
    if (logging_enabled_) {
        logTrajectoryInfo();
    }
}

void M_TrajectoryController::processTrajectoryQueue() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // Check if we need to start a new trajectory
    if (current_state_ != M_TrajectoryState::EXECUTING && !trajectory_queue_.empty()) {
        const auto& next_entry = trajectory_queue_.front();
        
        if (next_entry.start_time <= current_time_ || next_entry.immediate_execution) {
            startTrajectory(next_entry.trajectory);
            trajectory_queue_.pop();
        }
    }
}

void M_TrajectoryController::updateTrajectoryExecution() {
    if (current_state_ != M_TrajectoryState::EXECUTING || !current_trajectory_) {
        return;
    }
    
    double trajectory_time = current_time_ - trajectory_start_time_;
    
    if (trajectory_time >= current_trajectory_->getTotalTime()) {
        finishTrajectory();
        return;
    }
    
    // Calculate errors
    calculatePositionError();
    calculateVelocityError();
    
    // Check for trajectory completion based on tolerances
    bool position_reached = position_error_.head<2>().norm() < config_.position_tolerance &&
                           std::abs(position_error_[2]) < config_.angle_tolerance;
    bool velocity_reached = velocity_error_.norm() < config_.velocity_tolerance;
    
    if (trajectory_time > current_trajectory_->getTotalTime() * 0.9 && 
        position_reached && velocity_reached) {
        finishTrajectory();
    }
}

void M_TrajectoryController::calculateDesiredOutputs() {
    if (current_state_ != M_TrajectoryState::EXECUTING || !current_trajectory_) {
        desired_velocity_ = Eigen::Vector3d::Zero();
        desired_position_ = robot_position_;
        desired_acceleration_ = Eigen::Vector3d::Zero();
        return;
    }
    
    double trajectory_time = current_time_ - trajectory_start_time_;
    double lookahead_time = trajectory_time + config_.lookahead_time;
    
    // Clamp lookahead time to trajectory bounds
    lookahead_time = std::min(lookahead_time, current_trajectory_->getTotalTime());
    
    // Get desired state from trajectory
    desired_position_ = current_trajectory_->getPosition(trajectory_time);
    
    Eigen::Vector3d feedforward_velocity = Eigen::Vector3d::Zero();
    if (config_.enable_velocity_feedforward) {
        feedforward_velocity = calculateVelocityFeedforward(lookahead_time);
    }
    
    Eigen::Vector3d position_correction = Eigen::Vector3d::Zero();
    if (config_.enable_position_correction) {
        position_correction = calculatePositionCorrection();
        position_correction = limitVelocityCorrection(position_correction);
    }
    
    desired_velocity_ = feedforward_velocity + position_correction;
    desired_acceleration_ = current_trajectory_->getAcceleration(trajectory_time);
}

void M_TrajectoryController::calculatePositionError() {
    if (!current_trajectory_) {
        position_error_ = Eigen::Vector3d::Zero();
        return;
    }
    
    double trajectory_time = current_time_ - trajectory_start_time_;
    Eigen::Vector3d desired_pos = current_trajectory_->getPosition(trajectory_time);
    
    position_error_ = desired_pos - robot_position_;
    
    // Normalize angle error
    position_error_[2] = M_TrajectoryPlanner::normalizeAngle(position_error_[2]);
}

void M_TrajectoryController::calculateVelocityError() {
    if (!current_trajectory_) {
        velocity_error_ = Eigen::Vector3d::Zero();
        return;
    }
    
    double trajectory_time = current_time_ - trajectory_start_time_;
    Eigen::Vector3d desired_vel = current_trajectory_->getVelocity(trajectory_time);
    
    velocity_error_ = desired_vel - robot_velocity_;
}

Eigen::Vector3d M_TrajectoryController::calculateVelocityFeedforward(double t) const {
    if (!current_trajectory_) {
        return Eigen::Vector3d::Zero();
    }
    
    return current_trajectory_->getVelocity(t - trajectory_start_time_);
}

Eigen::Vector3d M_TrajectoryController::calculatePositionCorrection() const {
    return config_.proportional_gain * position_error_;
}

Eigen::Vector3d M_TrajectoryController::limitVelocityCorrection(const Eigen::Vector3d& correction) const {
    Eigen::Vector3d limited = correction;
    
    // Limit linear velocity corrections
    limited[0] = std::clamp(limited[0], -config_.max_velocity_correction, config_.max_velocity_correction);
    limited[1] = std::clamp(limited[1], -config_.max_velocity_correction, config_.max_velocity_correction);
    
    // Limit angular velocity correction (use different limit for rotation)
    double max_angular_correction = config_.max_velocity_correction * 2.0; // Allow higher angular corrections
    limited[2] = std::clamp(limited[2], -max_angular_correction, max_angular_correction);
    
    return limited;
}

void M_TrajectoryController::startTrajectory(std::shared_ptr<M_Trajectory> trajectory) {
    current_trajectory_ = trajectory;
    trajectory_start_time_ = current_time_;
    current_state_ = M_TrajectoryState::EXECUTING;
    
    if (logging_enabled_) {
        std::cout << "[M_TrajectoryController] Started trajectory execution (duration: " 
                  << trajectory->getTotalTime() << "s)" << std::endl;
    }
}

void M_TrajectoryController::finishTrajectory() {
    current_state_ = M_TrajectoryState::FINISHED;
    
    if (logging_enabled_) {
        std::cout << "[M_TrajectoryController] Trajectory execution finished" << std::endl;
    }
    
    // Check if there are more trajectories in queue
    switchToNextTrajectory();
}

void M_TrajectoryController::switchToNextTrajectory() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (!trajectory_queue_.empty()) {
        const auto& next_entry = trajectory_queue_.front();
        startTrajectory(next_entry.trajectory);
        trajectory_queue_.pop();
    } else {
        current_trajectory_.reset();
        current_state_ = M_TrajectoryState::IDLE;
    }
}

Eigen::Vector3d M_TrajectoryController::getDesiredVelocity() const {
    return desired_velocity_;
}

Eigen::Vector3d M_TrajectoryController::getDesiredPosition() const {
    return desired_position_;
}

Eigen::Vector3d M_TrajectoryController::getDesiredAcceleration() const {
    return desired_acceleration_;
}

M_TrajectoryInfo M_TrajectoryController::getTrajectoryInfo() const {
    M_TrajectoryInfo info;
    info.state = current_state_;
    
    if (current_trajectory_) {
        info.total_time = current_trajectory_->getTotalTime();
        info.current_time = current_time_ - trajectory_start_time_;
        info.progress = info.current_time / info.total_time;
    }
    
    info.position_error = position_error_;
    info.velocity_error = velocity_error_;
    
    switch (current_state_) {
        case M_TrajectoryState::IDLE:
            info.status_message = "No active trajectory";
            break;
        case M_TrajectoryState::EXECUTING:
            info.status_message = "Executing trajectory";
            break;
        case M_TrajectoryState::FINISHED:
            info.status_message = "Trajectory finished";
            break;
        case M_TrajectoryState::FAILED:
            info.status_message = "Trajectory execution failed";
            break;
    }
    
    return info;
}

void M_TrajectoryController::printStatus() const {
    auto info = getTrajectoryInfo();
    std::cout << "[M_TrajectoryController] Status: " << info.status_message << std::endl;
    
    if (current_state_ == M_TrajectoryState::EXECUTING) {
        std::cout << "  Progress: " << (info.progress * 100.0) << "% (" 
                  << info.current_time << "s / " << info.total_time << "s)" << std::endl;
        std::cout << "  Position Error: [" << info.position_error.transpose() << "]" << std::endl;
        std::cout << "  Velocity Error: [" << info.velocity_error.transpose() << "]" << std::endl;
        std::cout << "  Desired Velocity: [" << desired_velocity_.transpose() << "]" << std::endl;
    }
}

void M_TrajectoryController::logTrajectoryInfo() const {
    if (current_state_ == M_TrajectoryState::EXECUTING) {
        auto info = getTrajectoryInfo();
        std::cout << "[M_TrajectoryController] t=" << info.current_time 
                  << "s pos_err=" << info.position_error.norm()
                  << " vel_err=" << info.velocity_error.norm()
                  << " des_vel=[" << desired_velocity_.transpose() << "]" << std::endl;
    }
}

void M_TrajectoryController::setEmergencyStop(bool stop) {
    emergency_stop_ = stop;
    
    if (stop) {
        desired_velocity_ = Eigen::Vector3d::Zero();
        desired_acceleration_ = Eigen::Vector3d::Zero();
        
        if (logging_enabled_) {
            std::cout << "[M_TrajectoryController] Emergency stop activated" << std::endl;
        }
    } else {
        if (logging_enabled_) {
            std::cout << "[M_TrajectoryController] Emergency stop deactivated" << std::endl;
        }
    }
}

void M_TrajectoryController::resetController() {
    stopCurrentTrajectory();
    clearTrajectoryQueue();
    emergency_stop_ = false;
    current_state_ = M_TrajectoryState::IDLE;
    
    desired_velocity_ = Eigen::Vector3d::Zero();
    desired_position_ = robot_position_;
    desired_acceleration_ = Eigen::Vector3d::Zero();
    position_error_ = Eigen::Vector3d::Zero();
    velocity_error_ = Eigen::Vector3d::Zero();
    
    if (logging_enabled_) {
        std::cout << "[M_TrajectoryController] Controller reset" << std::endl;
    }
}

//=============================================================================
// M_TrajectoryManager Implementation
//=============================================================================

M_TrajectoryManager::M_TrajectoryManager()
    : current_pose_(Eigen::Vector3d::Zero())
    , current_velocity_(Eigen::Vector3d::Zero())
    , current_time_(0.0)
    , current_waypoint_index_(0)
    , path_active_(false) {
    
    // Set default move constraints
    move_constraints_.vel_max = 1.0;        // Conservative default
    move_constraints_.acc_max = 2.0;
    move_constraints_.brk_max = 2.0;
    move_constraints_.vel_max_w = 8.0;
    move_constraints_.acc_max_w = 15.0;
}

bool M_TrajectoryManager::CreateTrajectoriesFromPath(
    const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    
    if (path_fWorld.size() < 2) {
        std::cerr << "[M_TrajectoryManager] Path too short (need at least 2 points)" << std::endl;
        return false;
    }
    
    current_path_ = path_fWorld;
    current_waypoint_index_ = 0;
    path_active_ = true;
    current_time_ = t_start_s;
    
    generateTrajectoryFromPath(path_fWorld, t_start_s);
    
    std::cout << "[M_TrajectoryManager] Created trajectory from path with " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    return true;
}

void M_TrajectoryManager::generateTrajectoryFromPath(
    const std::vector<Eigen::Vector3d>& path, double start_time) {
    
    controller_.clearTrajectoryQueue();
    
    if (path.size() < 2) {
        std::cerr << "[M_TrajectoryManager] Path too short for trajectory generation" << std::endl;
        return;
    }
    
    // Create robot state
    M_RobotState robot_state = createRobotState();
    
    // Generate single trajectory directly to final destination for optimal path following
    Eigen::Vector3d start_pose = current_pose_;
    Eigen::Vector3d final_destination = path.back();
    
    // Calculate primary direction for optimal path following
    Eigen::Vector3d displacement = final_destination - start_pose;
    Eigen::Vector2d primary_direction = displacement.head<2>().normalized();
    
    // Set primary direction in constraints for async trajectory generation
    M_MoveConstraints optimized_constraints = move_constraints_;
    optimized_constraints.primary_direction = primary_direction;
    
    // Reduce velocity limits to prevent safety stops
    optimized_constraints.vel_max = std::min(move_constraints_.vel_max, 0.8);  // Conservative limit
    optimized_constraints.acc_max = std::min(move_constraints_.acc_max, 2.0);
    
    auto trajectory = M_TrajectoryPlanner::generatePositionTrajectory(
        optimized_constraints, start_pose, robot_state.velocity, final_destination);
    
    if (trajectory) {
        controller_.addTrajectoryToQueue(trajectory, start_time, "direct_path");
        std::cout << "[M_TrajectoryManager] Generated direct trajectory (time: " 
                  << trajectory->getTotalTime() << "s, max_vel: " 
                  << optimized_constraints.vel_max << "m/s)" << std::endl;
    } else {
        std::cerr << "[M_TrajectoryManager] Failed to generate direct trajectory" << std::endl;
    }
}

void M_TrajectoryManager::UpdateRobotState(
    const Eigen::Vector3d& pose_est, const Eigen::Vector3d& velocity_est) {
    current_pose_ = pose_est;
    current_velocity_ = velocity_est;
    
    controller_.updateRobotState(pose_est, velocity_est);
}

std::pair<bool, Eigen::Vector3d> M_TrajectoryManager::Update(const Eigen::Vector3d& pose_est) {
    updateCurrentTime();
    UpdateRobotState(pose_est, current_velocity_);
    
    controller_.updateTime(current_time_);
    
    bool motion_finished = controller_.isTrajectoryFinished() && !path_active_;
    Eigen::Vector3d desired_velocity = controller_.getDesiredVelocity();
    
    return std::make_pair(motion_finished, desired_velocity);
}

Eigen::Vector3d M_TrajectoryManager::GetVelocityAtT(double current_time_s) {
    current_time_ = current_time_s;
    controller_.updateTime(current_time_s);
    
    return controller_.getDesiredVelocity();
}

Eigen::Vector3d M_TrajectoryManager::GetPositionAtT(double current_time_s) {
    current_time_ = current_time_s;
    controller_.updateTime(current_time_s);
    
    return controller_.getDesiredPosition();
}

void M_TrajectoryManager::SetMoveConstraints(const M_MoveConstraints& constraints) {
    move_constraints_ = constraints;
}

void M_TrajectoryManager::SetControllerConfig(const M_ControllerConfig& config) {
    controller_.setConfig(config);
}

void M_TrajectoryManager::Print() const {
    std::cout << "[M_TrajectoryManager] ";
    controller_.printStatus();
    
    auto info = controller_.getTrajectoryInfo();
    std::cout << "  Move Constraints: vel_max=" << move_constraints_.vel_max
              << " acc_max=" << move_constraints_.acc_max
              << " vel_max_w=" << move_constraints_.vel_max_w << std::endl;
}

void M_TrajectoryManager::updateCurrentTime() {
    current_time_ = util::GetCurrentTime();
}

M_RobotState M_TrajectoryManager::createRobotState() const {
    M_RobotState state;
    state.position = current_pose_;
    state.velocity = current_velocity_;
    state.constraints = move_constraints_;
    return state;
}

} // namespace ctrl