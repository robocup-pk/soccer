#include "BangBangTrajectoryManager.h"
#include "Utils.h"
#include <iostream>
#include <iomanip>

namespace rob {

BangBangTrajectoryManager::BangBangTrajectoryManager() 
    : current_pose_(Eigen::Vector3d::Zero()),
      current_velocity_(Eigen::Vector3d::Zero()),
      trajectory_start_time_(0.0),
      is_initialized_(false) {
    
    bang_bang_trajectory_ = std::make_unique<ctrl::BangBangTrajectory>();
    bang_bang_trajectory_->SetLimits(v_max_, a_max_, omega_max_, alpha_max_);
    
    std::cout << "[BangBangTrajectoryManager] Initialized with BangBang trajectory generator" << std::endl;
}

bool BangBangTrajectoryManager::CreateTrajectoriesFromPath(
    const std::vector<Eigen::Vector3d>& path_fWorld, 
    double t_start_s) {
    
    if (path_fWorld.size() < 2) {
        std::cerr << "[BangBangTrajectoryManager] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[BangBangTrajectoryManager] Creating trajectories for " << path_fWorld.size() << " waypoints" << std::endl;
    
    // Print path for debugging (same format as original TrajectoryManager)
    std::cout << "[BangBangTrajectoryManager] Path: ";
    for (size_t i = 0; i < path_fWorld.size() - 1; ++i) {
        std::cout << "(" << std::fixed << std::setprecision(3) 
                  << path_fWorld[i][0] << ", " << path_fWorld[i][1] << ", " << path_fWorld[i][2] << ") -> ";
    }
    std::cout << "(" << path_fWorld.back()[0] << ", " << path_fWorld.back()[1] << ", " << path_fWorld.back()[2] << ")" << std::endl;
    
    try {
        // Use BangBangTrajectory to set path
        double start_time = (t_start_s > 0) ? t_start_s : GetCurrentTime();
        bool success = bang_bang_trajectory_->SetPath(path_fWorld, start_time);
        
        if (success) {
            trajectory_start_time_ = start_time;
            active_traj_t_finish_s = start_time + bang_bang_trajectory_->GetRemainingTime();
            
            std::cout << "[BangBangTrajectoryManager] Trajectory created successfully" << std::endl;
            std::cout << "[BangBangTrajectoryManager] Start time: " << trajectory_start_time_ << "s" << std::endl;
            std::cout << "[BangBangTrajectoryManager] Finish time: " << active_traj_t_finish_s << "s" << std::endl;
            std::cout << "[BangBangTrajectoryManager] Duration: " << (active_traj_t_finish_s - trajectory_start_time_) << "s" << std::endl;
        } else {
            std::cerr << "[BangBangTrajectoryManager] Failed to create trajectory" << std::endl;
        }
        
        return success;
        
    } catch (const std::exception& e) {
        std::cerr << "[BangBangTrajectoryManager] Exception: " << e.what() << std::endl;
        return false;
    }
}

std::tuple<bool, Eigen::Vector3d> BangBangTrajectoryManager::Update(const Eigen::Vector3d& pose_fWorld) {
    // Update current state
    Eigen::Vector3d velocity_estimate = Eigen::Vector3d::Zero(); // TODO: Get from state estimator
    UpdateState(pose_fWorld, velocity_estimate);
    
    double current_time = GetCurrentTime();
    
    // Get velocity command from BangBangTrajectory
    Eigen::Vector3d velocity_command = bang_bang_trajectory_->Update(pose_fWorld, current_time);
    
    bool finished = bang_bang_trajectory_->IsFinished();
    
    // Debug output (similar to original TrajectoryManager)
    static int update_count = 0;
    update_count++;
    if (update_count % 50 == 0) { // Print every 50 updates (~1 second at 50Hz)
        std::cout << "[BangBangTrajectoryManager] Update #" << update_count 
                  << " - Finished: " << (finished ? "YES" : "NO")
                  << ", Velocity: [" << std::fixed << std::setprecision(3)
                  << velocity_command[0] << ", " << velocity_command[1] << ", " << velocity_command[2] << "]" << std::endl;
    }
    
    return std::make_tuple(finished, velocity_command);
}

bool BangBangTrajectoryManager::IsFinished() const {
    return bang_bang_trajectory_->IsFinished();
}

double BangBangTrajectoryManager::GetRemainingTime() const {
    return bang_bang_trajectory_->GetRemainingTime();
}

bool BangBangTrajectoryManager::AddGoal(const Eigen::Vector3d& goal) {
    std::cout << "[BangBangTrajectoryManager] Adding goal: (" 
              << std::fixed << std::setprecision(3)
              << goal[0] << ", " << goal[1] << ", " << goal[2] << ")" << std::endl;
    
    return bang_bang_trajectory_->AddGoal(goal);
}

void BangBangTrajectoryManager::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
    
    bang_bang_trajectory_->SetLimits(v_max, a_max, omega_max, alpha_max);
    
    std::cout << "[BangBangTrajectoryManager] Updated limits:" << std::endl;
    std::cout << "  v_max: " << v_max_ << " m/s" << std::endl;
    std::cout << "  a_max: " << a_max_ << " m/s²" << std::endl;
    std::cout << "  ω_max: " << omega_max_ << " rad/s" << std::endl;
    std::cout << "  α_max: " << alpha_max_ << " rad/s²" << std::endl;
}

void BangBangTrajectoryManager::Initialize(const Eigen::Vector3d& initial_pose, const Eigen::Vector3d& initial_velocity) {
    current_pose_ = initial_pose;
    current_velocity_ = initial_velocity;
    is_initialized_ = true;
    
    std::cout << "[BangBangTrajectoryManager] Initialized with:" << std::endl;
    std::cout << "  Initial pose: [" << std::fixed << std::setprecision(3)
              << initial_pose[0] << ", " << initial_pose[1] << ", " << initial_pose[2] << "]" << std::endl;
    std::cout << "  Initial velocity: [" << initial_velocity[0] << ", " << initial_velocity[1] << ", " << initial_velocity[2] << "]" << std::endl;
}

void BangBangTrajectoryManager::PrintTrajectoryInfo() const {
    std::cout << "=== BangBang Trajectory Manager Status ===" << std::endl;
    std::cout << "Initialized: " << (is_initialized_ ? "YES" : "NO") << std::endl;
    std::cout << "Finished: " << (IsFinished() ? "YES" : "NO") << std::endl;
    std::cout << "Remaining time: " << GetRemainingTime() << " seconds" << std::endl;
    std::cout << "Current pose: [" << std::fixed << std::setprecision(3)
              << current_pose_[0] << ", " << current_pose_[1] << ", " << current_pose_[2] << "]" << std::endl;
    std::cout << "Current velocity: [" << current_velocity_[0] << ", " << current_velocity_[1] << ", " << current_velocity_[2] << "]" << std::endl;
    
    // Print acceleration envelope info
    std::cout << "\n=== Acceleration Envelope ===" << std::endl;
    bang_bang_trajectory_->PrintAccelerationEnvelope();
}

double BangBangTrajectoryManager::GetCurrentTime() const {
    return util::GetCurrentTime();
}

void BangBangTrajectoryManager::UpdateState(const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity) {
    current_pose_ = pose;
    current_velocity_ = velocity;
}

} // namespace rob