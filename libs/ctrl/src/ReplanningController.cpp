#include "ReplanningController.h"
#include "Utils.h"
#include <iostream>
#include <algorithm>
#include <cmath>

namespace ctrl {

ReplanningController::ReplanningController() 
    : ReplanningController(ReplanningConfig()) {}

ReplanningController::ReplanningController(const ReplanningConfig& config)
    : config_(config),
      trajectory_tracker_(std::make_shared<TrajectoryTracker>()),
      velocity_limiter_value_(1.0),
      come_to_stop_(false),
      last_replan_time_(std::chrono::steady_clock::now()) {
    
    std::cout << "[ReplanningController] Initialized with replanning" << std::endl;
    std::cout << "  Replan frequency: " << config_.replan_frequency_hz << " Hz" << std::endl;
    std::cout << "  Collision brake time: " << config_.collision_brake_time_s << " s" << std::endl;
    std::cout << "  Max position error: " << config_.max_position_error_m << " m" << std::endl;
}

void ReplanningController::setDestination(const Eigen::Vector3d& destination) {
    destination_ = destination;
    come_to_stop_ = false;
    
    std::cout << "[ReplanningController] New destination: (" 
              << destination.transpose() << ")" << std::endl;
}

void ReplanningController::setMoveConstraints(const MoveConstraints& constraints) {
    move_constraints_ = constraints;
}

void ReplanningController::setObstacles(const std::vector<std::shared_ptr<IObstacle>>& obstacles) {
    obstacles_ = obstacles;
}

Eigen::Vector3d ReplanningController::update(const Eigen::Vector3d& current_pose, 
                                           const Eigen::Vector3d& current_velocity) {
    
    // Check if replanning is disabled
    if (!config_.enable_replanning) {
        if (current_planner_ && trajectory_tracker_) {
            return trajectory_tracker_->update(current_pose);
        }
        return Eigen::Vector3d::Zero();
    }
    
    // Check replanning frequency (replan every frame!)
    auto current_time = std::chrono::steady_clock::now();
    double dt_since_last_replan = std::chrono::duration<double>(current_time - last_replan_time_).count();
    
    bool should_replan = (dt_since_last_replan >= (1.0 / config_.replan_frequency_hz));
    
    // Emergency replanning if large position error (additional safety)
    double position_error = (current_pose.head<2>() - destination_.head<2>()).norm();
    bool emergency_replan = (position_error > config_.max_position_error_m) && 
                           (dt_since_last_replan >= config_.min_replanning_interval_s);
    
    if (should_replan || emergency_replan) {
        
        if (emergency_replan) {
            std::cout << "[ReplanningController] EMERGENCY REPLAN! Position error: " 
                      << position_error << "m" << std::endl;
            stats_.emergency_brakes++;
        }
        
        // === ADVANCED REPLANNING CYCLE (AMoveToSkill.java lines 138-164) ===
        
        // Step 1: Create PathFinderInput with current robot state
        PathFinderInput path_input = createPathFinderInput(current_pose, current_velocity);
        
        // Step 2: Calculate path using PathFinder
        auto path_result_opt = path_finder_.calcPath(path_input);
        
        last_replan_time_ = current_time;
        stats_.total_replans++;
        
        if (come_to_stop_) {
            // User requested stop
            std::cout << "[ReplanningController] Coming to stop..." << std::endl;
            return performBrake(current_velocity);
        }
        else if (!path_result_opt.has_value() || needToBrake(path_result_opt.value())) {
            // No valid path OR collision detected - BRAKE!
            std::cout << "[ReplanningController] ";
            if (!path_result_opt.has_value()) {
                std::cout << "No valid path found - braking!" << std::endl;
            } else {
                std::cout << "Collision detected at t=" << path_result_opt->getFirstCollisionTime() 
                          << "s - braking!" << std::endl;
                stats_.collision_avoidances++;
            }
            
            // Decrease velocity limit (adaptive approach)
            updateVelocityLimiter(false, dt_since_last_replan);
            stats_.is_braking = true;
            
            return performBrake(current_velocity);
        }
        else {
            // Valid collision-free path found - EXECUTE!
            std::cout << "[ReplanningController] Valid path found, executing..." << std::endl;
            
            // Increase velocity limit (adaptive approach)  
            updateVelocityLimiter(true, dt_since_last_replan);
            stats_.is_braking = false;
            
            return executePath(path_result_opt.value(), current_pose);
        }
    }
    
    // Continue with existing trajectory
    if (current_planner_ && trajectory_tracker_) {
        return trajectory_tracker_->update(current_pose);
    }
    
    return Eigen::Vector3d::Zero();
}

PathFinderInput ReplanningController::createPathFinderInput(const Eigen::Vector3d& current_pose,
                                                          const Eigen::Vector3d& current_velocity) {
    
    // Apply velocity limiting
    MoveConstraints limited_constraints = limitRobotSpeed(move_constraints_);
    
    // Create PathFinderInput
    PathFinderInput input = PathFinderInput::fromBot(current_pose, current_velocity)
        .dest(destination_.head<2>())  // PathFinder only handles 2D destinations
        .obstacles(obstacles_)
        .moveConstraints(limited_constraints)
        .timestamp(0)  // Could use actual timestamp
        .build();
    
    std::cout << "[ReplanningController] Created PathFinderInput: "
              << obstacles_.size() << " obstacles, "
              << "vel_limit=" << limited_constraints.getVelMax() << "m/s" << std::endl;
    
    return input;
}

bool ReplanningController::needToBrake(const PathFinderResult& path_result) const {
    // Check if braking is needed
    double brake_time = config_.collision_brake_time_s;
    return path_result.getFirstCollisionTime() <= brake_time;
}

double ReplanningController::calculateBrakeTime(const Eigen::Vector3d& current_velocity) const {
    // Calculate brake time based on velocity
    double vel_magnitude = std::max(0.0, current_velocity.head<2>().norm() - config_.brake_velocity_tolerance);
    double brake_time = vel_magnitude / move_constraints_.getAccMax();
    return brake_time + config_.collision_brake_time_s;  // Add reaction time
}

Eigen::Vector3d ReplanningController::performBrake(const Eigen::Vector3d& current_velocity) {
    // Emergency braking behavior
    std::cout << "[ReplanningController] EMERGENCY BRAKE! Current vel: " 
              << current_velocity.head<2>().norm() << "m/s" << std::endl;
    
    // Clear current trajectory
    current_planner_ = nullptr;
    trajectory_tracker_ = std::make_shared<TrajectoryTracker>();
    
    // Return zero velocity (full stop)
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d ReplanningController::executePath(const PathFinderResult& path_result,
                                                const Eigen::Vector3d& current_pose) {
    
    // Create new AdvancedMotionPlanner from PathFinderResult
    // Note: PathFinder only gives 2D path, so we need to add orientation like we fixed before
    
    std::cout << "[ReplanningController] Executing new path (duration: " 
              << path_result.getTrajectory().getTotalTime() << "s)" << std::endl;
    
    // Create AdvancedMotionPlanner with complete trajectory (position + orientation)
    current_planner_ = std::make_shared<AdvancedMotionPlanner>();
    
    // Use the fixed approach that includes orientation
    current_planner_->planTrajectory(
        current_pose,                    // Current position with orientation
        Eigen::Vector3d::Zero(),        // Current velocity (simplified)
        destination_,                   // Full destination with orientation
        obstacles_,                     // Obstacles
        limitRobotSpeed(move_constraints_)  // Limited constraints
    );
    
    if (current_planner_->isValid()) {
        // Set new trajectory in tracker
        trajectory_tracker_->setTrajectory(current_planner_);
        
        // Get first velocity command
        return trajectory_tracker_->update(current_pose);
    } else {
        std::cout << "[ReplanningController] Failed to create valid AdvancedMotionPlanner!" << std::endl;
        return performBrake(Eigen::Vector3d::Zero());
    }
}

void ReplanningController::updateVelocityLimiter(bool path_is_good, double dt) {
    // Velocity limiter logic (maxRobotSpeedLimiter)
    
    if (path_is_good) {
        // Increase velocity limit
        velocity_limiter_value_ += config_.velocity_increase_rate * dt;
        velocity_limiter_value_ = std::min(velocity_limiter_value_, move_constraints_.getVelMax());
    } else {
        // Decrease velocity limit  
        velocity_limiter_value_ += config_.velocity_decrease_rate * dt;
        velocity_limiter_value_ = std::max(velocity_limiter_value_, config_.velocity_limiter_min);
    }
    
    stats_.current_velocity_limit = velocity_limiter_value_;
    
    std::cout << "[ReplanningController] Velocity limiter: " << velocity_limiter_value_ 
              << "m/s (path_good=" << (path_is_good ? "true" : "false") << ")" << std::endl;
}

MoveConstraints ReplanningController::limitRobotSpeed(const MoveConstraints& base_constraints) const {
    // Apply acceleration limiting
    MoveConstraints limited = base_constraints;
    
    // Apply velocity limiter
    limited.setVelMax(std::min(base_constraints.getVelMax(), velocity_limiter_value_));
    
    return limited;
}

bool ReplanningController::isDestinationReached() const {
    if (!current_planner_) return false;
    
    // Check if trajectory is finished and we're close to destination
    bool trajectory_done = trajectory_tracker_ ? trajectory_tracker_->isFinished() : true;
    
    return trajectory_done;
}

std::shared_ptr<AdvancedMotionPlanner> ReplanningController::getCurrentTrajectory() const {
    return current_planner_;
}

} // namespace ctrl