#include "PurePursuitTrajectoryManager.h"
#include "Utils.h"
#include <iostream>
#include <iomanip>

namespace ctrl {

PurePursuitTrajectoryManager::PurePursuitTrajectoryManager()
    : path_active_(false)
    , current_time_(0.0)
    , last_commanded_velocity_(Eigen::Vector3d::Zero()) {
    
    // Configure Pure Pursuit for SSL robots (smaller, faster)
    PurePursuitController::Config config;
    config.look_ahead_gain = 0.3;              // Reduced for smaller SSL robots
    config.base_look_ahead_distance = 0.08;    // 8cm base look-ahead for SSL field
    config.speed_proportional_gain = 3.0;      // Higher gain for responsive control
    config.position_tolerance = 0.03;          // 3cm position tolerance
    config.angular_tolerance = 0.15;           // ~8.6 degrees angular tolerance
    config.max_linear_velocity = 0.6;          // Conservative speed limit
    config.max_angular_velocity = 2.0;         // Conservative angular speed limit
    config.dt = 0.02;                          // 50Hz control rate
    
    pure_pursuit_controller_.setConfig(config);
    
    std::cout << "[PurePursuitTrajectoryManager] Initialized with Pure Pursuit controller" << std::endl;
}

bool PurePursuitTrajectoryManager::CreateTrajectoriesFromPath(
    const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    
    if (path_fWorld.size() < 2) {
        std::cerr << "[PurePursuitTrajectoryManager] Path too short (need at least 2 points)" << std::endl;
        return false;
    }
    
    current_path_ = path_fWorld;
    current_time_ = t_start_s;
    path_active_ = true;
    
    // Set the path in the Pure Pursuit controller
    pure_pursuit_controller_.setPath(path_fWorld);
    
    std::cout << "[PurePursuitTrajectoryManager] Created Pure Pursuit path with " 
              << path_fWorld.size() << " waypoints" << std::endl;
    
    // Print waypoints for debugging
    for (size_t i = 0; i < path_fWorld.size(); ++i) {
        std::cout << "  Waypoint " << i << ": (" << std::fixed << std::setprecision(3)
                  << path_fWorld[i].x() << ", " << path_fWorld[i].y() << ", " 
                  << path_fWorld[i].z() << " rad)" << std::endl;
    }
    
    return true;
}

void PurePursuitTrajectoryManager::UpdateRobotState(
    const Eigen::Vector3d& pose_est, const Eigen::Vector3d& velocity_est) {
    updateRobotState(pose_est, velocity_est);
}

std::pair<bool, Eigen::Vector3d> PurePursuitTrajectoryManager::Update(const Eigen::Vector3d& pose_est) {
    current_time_ = util::GetCurrentTime();
    
    // Update robot state (assuming zero velocity if not provided)
    updateRobotState(pose_est, Eigen::Vector3d::Zero());
    
    if (!path_active_ || current_path_.empty()) {
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Get Pure Pursuit control output
    auto control_output = pure_pursuit_controller_.update(robot_state_);
    
    // Check if path is finished
    if (control_output.path_finished) {
        path_active_ = false;
        last_commanded_velocity_ = Eigen::Vector3d::Zero();
        std::cout << "[PurePursuitTrajectoryManager] Path following completed!" << std::endl;
        return std::make_pair(true, Eigen::Vector3d::Zero());
    }
    
    // Store commanded velocity
    last_commanded_velocity_ = control_output.velocity();
    
    // Debug output
    if (static_cast<int>(current_time_ * 10) % 10 == 0) { // Print every 1 second
        std::cout << "[PurePursuitTrajectoryManager] Target: " << control_output.current_target_index
                  << ", Vel: (" << std::fixed << std::setprecision(2) 
                  << control_output.vx_cmd << ", " << control_output.vy_cmd << ", " 
                  << control_output.vyaw_cmd << ")" << std::endl;
    }
    
    return std::make_pair(false, last_commanded_velocity_);
}

Eigen::Vector3d PurePursuitTrajectoryManager::GetVelocityAtT(double current_time_s) {
    current_time_ = current_time_s;
    
    if (!path_active_) {
        return Eigen::Vector3d::Zero();
    }
    
    // For Pure Pursuit, we return the last computed velocity
    // since it's a reactive controller rather than trajectory-based
    return last_commanded_velocity_;
}

Eigen::Vector3d PurePursuitTrajectoryManager::GetPositionAtT(double current_time_s) {
    // Pure Pursuit doesn't pre-compute positions, so return current robot position
    return robot_state_.position();
}

void PurePursuitTrajectoryManager::SetPurePursuitConfig(const PurePursuitController::Config& config) {
    pure_pursuit_controller_.setConfig(config);
    std::cout << "[PurePursuitTrajectoryManager] Updated Pure Pursuit configuration" << std::endl;
}

const PurePursuitController::Config& PurePursuitTrajectoryManager::GetPurePursuitConfig() const {
    return pure_pursuit_controller_.getConfig();
}

void PurePursuitTrajectoryManager::Print() const {
    std::cout << "[PurePursuitTrajectoryManager] ";
    
    if (!path_active_) {
        std::cout << "No active path" << std::endl;
        return;
    }
    
    std::cout << "Active path with " << current_path_.size() << " waypoints" << std::endl;
    std::cout << "  Current target: " << pure_pursuit_controller_.getCurrentTargetIndex() << std::endl;
    std::cout << "  Robot position: (" << std::fixed << std::setprecision(3)
              << robot_state_.x << ", " << robot_state_.y << ", " << robot_state_.yaw << ")" << std::endl;
    std::cout << "  Last commanded velocity: (" << std::fixed << std::setprecision(2)
              << last_commanded_velocity_.x() << ", " << last_commanded_velocity_.y() << ", " 
              << last_commanded_velocity_.z() << ")" << std::endl;
    
    const auto& config = pure_pursuit_controller_.getConfig();
    std::cout << "  Pure Pursuit Config: look_ahead_gain=" << config.look_ahead_gain
              << ", base_distance=" << config.base_look_ahead_distance
              << ", max_vel=" << config.max_linear_velocity << std::endl;
}

int PurePursuitTrajectoryManager::GetCurrentWaypointIndex() const {
    return pure_pursuit_controller_.getCurrentTargetIndex();
}

double PurePursuitTrajectoryManager::GetDistanceToCurrentTarget() const {
    if (!path_active_ || current_path_.empty()) {
        return 0.0;
    }
    
    int target_index = pure_pursuit_controller_.getCurrentTargetIndex();
    if (target_index < 0 || target_index >= static_cast<int>(current_path_.size())) {
        return 0.0;
    }
    
    const auto& target = current_path_[target_index];
    double dx = target.x() - robot_state_.x;
    double dy = target.y() - robot_state_.y;
    return std::sqrt(dx * dx + dy * dy);
}

void PurePursuitTrajectoryManager::updateRobotState(
    const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity) {
    
    robot_state_.setPosition(pose);
    robot_state_.setVelocity(velocity);
}

} // namespace ctrl