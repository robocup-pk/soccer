#include <iostream>
#include <memory>
#include <chrono>
#include "BSplineTrajectoryManager.h"
#include "Utils.h"

namespace ctrl {

class BSplineTrajectoryManagerWithReplanning : public BSplineTrajectoryManager {
public:
    BSplineTrajectoryManagerWithReplanning() 
        : BSplineTrajectoryManager(),
          replan_threshold_(0.1),  // Replan if error > 10cm
          replan_interval_(0.5),   // Don't replan more than every 0.5s
          last_replan_time_(0.0),
          replan_count_(0) {
        std::cout << "[BSplineTrajectoryManagerWithReplanning] Initialized with replanning capability" << std::endl;
    }
    
    std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_est) override {
        p_fworld_ = pose_est;
        
        if (!trajectory_active_ || trajectory_finished_) {
            return std::make_pair(true, Eigen::Vector3d::Zero());
        }
        
        double current_time = util::GetCurrentTime();
        
        // Check if replanning is needed
        if (ShouldReplan(pose_est, current_time)) {
            PerformReplanning(pose_est, current_time);
        }
        
        // Continue with normal update
        return BSplineTrajectoryManager::Update(pose_est);
    }
    
private:
    bool ShouldReplan(const Eigen::Vector3d& current_pose, double current_time) {
        // Don't replan too frequently
        if (current_time - last_replan_time_ < replan_interval_) {
            return false;
        }
        
        // Get expected position from trajectory
        Eigen::Vector3d expected_pose = bspline_trajectory_->GetCurrentDesiredPosition();
        
        // Calculate position error (only X-Y, ignore orientation for now)
        double position_error = (current_pose.head<2>() - expected_pose.head<2>()).norm();
        
        // Log the error for analysis
        if (replan_count_ % 10 == 0) {  // Log every 10th check
            std::cout << "[Replanning] Position error: " << position_error 
                      << "m (threshold: " << replan_threshold_ << "m)" << std::endl;
        }
        
        return position_error > replan_threshold_;
    }
    
    void PerformReplanning(const Eigen::Vector3d& current_pose, double current_time) {
        std::cout << "[Replanning] Triggered at time " << current_time 
                  << " (Replan #" << replan_count_ + 1 << ")" << std::endl;
        
        // Get remaining waypoints from current trajectory
        std::vector<Eigen::Vector3d> remaining_waypoints = 
            bspline_trajectory_->GetRemainingWaypoints();
        
        if (remaining_waypoints.empty()) {
            std::cout << "[Replanning] No remaining waypoints, keeping current trajectory" << std::endl;
            return;
        }
        
        // Create new waypoint list starting from current position
        std::vector<Eigen::Vector3d> new_waypoints;
        new_waypoints.push_back(current_pose);  // Start from current position
        
        // Add remaining waypoints
        for (const auto& wp : remaining_waypoints) {
            new_waypoints.push_back(wp);
        }
        
        // Create new trajectory
        bool success = CreateTrajectoriesFromPath(new_waypoints, current_time);
        
        if (success) {
            last_replan_time_ = current_time;
            replan_count_++;
            std::cout << "[Replanning] Successfully created new trajectory from current position" << std::endl;
            std::cout << "[Replanning] New path: " << current_pose.transpose() 
                      << " -> " << new_waypoints.back().transpose() << std::endl;
        } else {
            std::cout << "[Replanning] Failed to create new trajectory!" << std::endl;
        }
    }
    
    double replan_threshold_;    // Position error threshold for replanning
    double replan_interval_;     // Minimum time between replans
    double last_replan_time_;    // Time of last replan
    int replan_count_;          // Number of replans performed
};

} // namespace ctrl