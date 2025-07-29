#ifndef BANG_BANG_TRAJECTORY_MANAGER_H
#define BANG_BANG_TRAJECTORY_MANAGER_H

#include "BangBangTrajectory.h"
#include "TrajectoryManager.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace rob {

/**
 * @brief Wrapper class to integrate BangBangTrajectory with existing TrajectoryManager interface
 * 
 * This class maintains compatibility with the current RobotManager::SetPath() and 
 * RobotManager::AddGoal() while using the new BangBangTrajectory implementation internally.
 */
class BangBangTrajectoryManager {
public:
    BangBangTrajectoryManager();
    ~BangBangTrajectoryManager() = default;
    
    /**
     * @brief Create trajectories from path (replaces TrajectoryManager::CreateTrajectoriesFromPath)
     * @param path_fWorld Waypoints in world frame
     * @param t_start_s Start time (default: current time)
     * @return true if successful
     */
    bool CreateTrajectoriesFromPath(
        const std::vector<Eigen::Vector3d>& path_fWorld, 
        double t_start_s = 0.0
    );
    
    /**
     * @brief Update trajectory and get velocity command (replaces TrajectoryManager::Update)
     * @param pose_fWorld Current robot pose [x, y, θ]
     * @return (finished, velocity) where velocity is [vx, vy, ω]
     */
    std::tuple<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_fWorld);
    
    /**
     * @brief Check if trajectory is finished
     */
    bool IsFinished() const;
    
    /**
     * @brief Get remaining execution time
     */
    double GetRemainingTime() const;
    
    /**
     * @brief Add goal to trajectory queue (for RobotManager::AddGoal integration)
     * @param goal Goal position [x, y, θ] 
     * @return true if successful
     */
    bool AddGoal(const Eigen::Vector3d& goal);
    
    /**
     * @brief Set velocity and acceleration limits
     */
    void SetLimits(double v_max, double a_max, double omega_max, double alpha_max);
    
    /**
     * @brief Initialize with current robot state
     */
    void Initialize(const Eigen::Vector3d& initial_pose, const Eigen::Vector3d& initial_velocity);
    
    /**
     * @brief Get trajectory information for debugging
     */
    void PrintTrajectoryInfo() const;
    
    // Compatibility with existing TrajectoryManager interface
    double active_traj_t_finish_s = 0.0;  // For compatibility with RobotManager logging
    
private:
    std::unique_ptr<ctrl::BangBangTrajectory> bang_bang_trajectory_;
    
    // State tracking
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_velocity_;
    double trajectory_start_time_;
    bool is_initialized_;
    
    // Limits
    double v_max_ = 2.0;
    double a_max_ = 3.0;
    double omega_max_ = 10.0;
    double alpha_max_ = 20.0;
    
    // Helper functions
    double GetCurrentTime() const;
    void UpdateState(const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity);
};

} // namespace rob

#endif // BANG_BANG_TRAJECTORY_MANAGER_H