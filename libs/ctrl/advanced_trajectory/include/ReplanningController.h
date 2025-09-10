#pragma once

#include "PathFinder.h"
#include "PathFinderInput.h"
#include "AdvancedMotionPlanner.h"
#include "MoveConstraints.h"
#include "IObstacle.h"
#include "TrajectoryTracker.h"
#include "PathfindingConfig.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <chrono>

namespace ctrl {

/**
 * @brief Implementation of replanning system
 * 
 * This implements the aggressive replanning approach:
 * 1. Recalculates path EVERY control loop iteration
 * 2. Dynamic obstacle detection and avoidance
 * 3. Collision prediction with automatic braking
 * 4. Adaptive velocity limiting when paths are problematic
 */
class ReplanningController {
public:
    /**
     * @brief Configuration for replanning behavior
     */
    struct ReplanningConfig {
        bool enable_replanning = true;
        double replan_frequency_hz = 50.0;  // How often to replan (50Hz as needed)
        double collision_brake_time_s = 0.05;  // Time horizon for collision detection
        double brake_velocity_tolerance = 0.6;  // Velocity buffer for braking calculations
        double max_position_error_m = 0.3;  // Position error threshold for emergency replanning
        double min_replanning_interval_s = 0.02;  // Minimum time between replans (50Hz)
        
        // Velocity limiting (as needed's maxRobotSpeedLimiter)
        double velocity_limiter_min = 0.5;  // Minimum allowed velocity when problems detected
        double velocity_increase_rate = 5.0;  // Rate to increase velocity when clear
        double velocity_decrease_rate = -5.0;  // Rate to decrease velocity when problems
    };

    ReplanningController();
    explicit ReplanningController(const ReplanningConfig& config);

    /**
     * @brief Set the target destination for trajectory planning
     * @param destination Target position (x, y, theta)
     */
    void setDestination(const Eigen::Vector3d& destination);
    
    /**
     * @brief Set movement constraints (velocity, acceleration limits)
     */
    void setMoveConstraints(const MoveConstraints& constraints);
    
    /**
     * @brief Add obstacles for path planning
     * @param obstacles List of obstacles to avoid
     */
    void setObstacles(const std::vector<std::shared_ptr<IObstacle>>& obstacles);
    
    /**
     * @brief Main update loop - Implementation of AMoveToSkill logic
     * 
     * This method implements the complete standard replanning cycle:
     * 1. Generate obstacles for current world state
     * 2. Create PathFinderInput with current robot state
     * 3. Calculate path using PathFinder
     * 4. Check for collisions and decide whether to brake or execute
     * 5. Update velocity limiter based on path quality
     * 
     * @param current_pose Current robot pose (x, y, theta)
     * @param current_velocity Current robot velocity (vx, vy, omega)
     * @return Body-frame velocity command or Zero if braking
     */
    Eigen::Vector3d update(const Eigen::Vector3d& current_pose, 
                          const Eigen::Vector3d& current_velocity);
    
    /**
     * @brief Check if robot has reached destination
     */
    bool isDestinationReached() const;
    
    /**
     * @brief Get current trajectory for visualization/debugging
     */
    std::shared_ptr<AdvancedMotionPlanner> getCurrentTrajectory() const;
    
    /**
     * @brief Get replanning statistics
     */
    struct ReplanningStats {
        int total_replans = 0;
        int emergency_brakes = 0;
        int collision_avoidances = 0;
        double current_velocity_limit = 1.0;
        double last_replan_time_s = 0.0;
        bool is_braking = false;
    };
    
    ReplanningStats getStats() const { return stats_; }
    
    /**
     * @brief Enable or disable replanning (for debugging)
     */
    void setReplanningEnabled(bool enabled) { config_.enable_replanning = enabled; }

private:
    // Core components (as needed's AMoveToSkill)
    PathFinder path_finder_;
    std::shared_ptr<TrajectoryTracker> trajectory_tracker_;
    std::shared_ptr<AdvancedMotionPlanner> current_planner_;
    
    // Configuration and state
    ReplanningConfig config_;
    ReplanningStats stats_;
    Eigen::Vector3d destination_;
    MoveConstraints move_constraints_;
    std::vector<std::shared_ptr<IObstacle>> obstacles_;
    
    // Timing and control
    std::chrono::steady_clock::time_point last_replan_time_;
    double velocity_limiter_value_;
    bool come_to_stop_;
    
    // Helper methods (exact copies from Advanced)
    
    /**
     * @brief Create PathFinderInput from current robot state (standard AMoveToSkill line 212)
     */
    PathFinderInput createPathFinderInput(const Eigen::Vector3d& current_pose,
                                        const Eigen::Vector3d& current_velocity);
    
    /**
     * @brief Check if robot needs to brake due to collision (standard AMoveToSkill line 181)
     */
    bool needToBrake(const PathFinderResult& path_result) const;
    
    /**
     * @brief Calculate brake time based on current velocity (standard AMoveToSkill line 187)
     */
    double calculateBrakeTime(const Eigen::Vector3d& current_velocity) const;
    
    /**
     * @brief Perform emergency braking (standard AMoveToSkill line 142)
     */
    Eigen::Vector3d performBrake(const Eigen::Vector3d& current_velocity);
    
    /**
     * @brief Execute path with trajectory tracker (standard AMoveToSkill line 268)
     */
    Eigen::Vector3d executePath(const PathFinderResult& path_result,
                               const Eigen::Vector3d& current_pose);
    
    /**
     * @brief Update velocity limiter (adaptive speed control as needed)
     */
    void updateVelocityLimiter(bool path_is_good, double dt);
    
    /**
     * @brief Limit robot speed based on current conditions (standard AMoveToSkill line 254)
     */
    MoveConstraints limitRobotSpeed(const MoveConstraints& base_constraints) const;
};

} // namespace ctrl