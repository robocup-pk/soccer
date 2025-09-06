#include "AdvancedMotionPlanner.h"
#include "Utils.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

void AdvancedMotionPlanner::plan(const std::vector<Eigen::Vector3d>& waypoints,
                                 double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc) {
    segments_.clear();
    total_time_ = 0.0;
    
    if (waypoints.size() < 2) {
        std::cout << "[AdvancedMotionPlanner] Error: Need at least 2 waypoints" << std::endl;
        return;
    }

    max_velocity_ = maxVel;
    max_acceleration_ = maxAcc;
    max_angular_velocity_ = maxOmega;
    max_angular_acceleration_ = maxOmegaAcc;

    std::cout << "[AdvancedMotionPlanner] Planning BangBang trajectory through " 
              << waypoints.size() << " waypoints" << std::endl;

    // Create BangBang trajectory segments between consecutive waypoints
    double cumulative_time = 0.0;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Eigen::Vector3d& start = waypoints[i];
        const Eigen::Vector3d& end = waypoints[i + 1];
        
        TrajectorySegment segment;
        segment.start_time = cumulative_time;
        
        // Extract 2D positions and orientations
        Eigen::Vector2d start_pos = start.head<2>();
        Eigen::Vector2d end_pos = end.head<2>();
        double start_theta = start.z();
        double end_theta = end.z();
        
        // Estimate initial velocity based on previous trajectory (if available)
        Eigen::Vector2d initial_velocity = Eigen::Vector2d::Zero();
        double initial_angular_velocity = 0.0;
        
        if (i > 0 && !segments_.empty()) {
            // Get velocity from previous segment at its end time
            const auto& prev_segment = segments_.back();
            double prev_end_time = prev_segment.duration;
            initial_velocity = prev_segment.position_traj.getVelocity(prev_end_time);
            initial_angular_velocity = prev_segment.orientation_traj.getVelocity(prev_end_time);
        }
        
        // Create synchronized 2D position trajectory
        segment.position_traj = factory_.sync(
            start_pos, end_pos, initial_velocity, maxVel, maxAcc
        );
        
        // Create orientation trajectory  
        segment.orientation_traj = factory_.orientation(
            start_theta, end_theta, initial_angular_velocity, maxOmega, maxOmegaAcc
        );
        
        // Duration is the maximum of position and orientation trajectory times
        segment.duration = std::max(
            segment.position_traj.getTotalTime(),
            segment.orientation_traj.getTotalTime()
        );
        
        cumulative_time += segment.duration;
        segments_.push_back(segment);
        
        std::cout << "[AdvancedMotionPlanner] Segment " << i << ": " 
                  << "pos_time=" << segment.position_traj.getTotalTime() << "s, "
                  << "orient_time=" << segment.orientation_traj.getTotalTime() << "s, "
                  << "duration=" << segment.duration << "s" << std::endl;
    }
    
    total_time_ = cumulative_time;
    std::cout << "[AdvancedMotionPlanner] Total trajectory time: " << total_time_ << "s" << std::endl;
}

int AdvancedMotionPlanner::findActiveSegment(double time) const {
    for (int i = 0; i < (int)segments_.size(); ++i) {
        if (segments_[i].isActive(time)) {
            return i;
        }
    }
    // Return last segment if time is beyond all segments
    return std::max(0, (int)segments_.size() - 1);
}

Eigen::Vector3d AdvancedMotionPlanner::getPosition(double time) const {
    if (!isValid()) return Eigen::Vector3d::Zero();
    
    if (time <= 0.0) {
        // Return first waypoint
        const auto& first_segment = segments_[0];
        Eigen::Vector2d pos = first_segment.position_traj.getPosition(0.0);
        double theta = first_segment.orientation_traj.getPosition(0.0);
        return Eigen::Vector3d(pos.x(), pos.y(), theta);
    }
    
    if (time >= total_time_) {
        // Return last waypoint
        const auto& last_segment = segments_.back();
        double local_time = last_segment.duration;
        Eigen::Vector2d pos = last_segment.position_traj.getPosition(local_time);
        double theta = last_segment.orientation_traj.getPosition(local_time);
        return Eigen::Vector3d(pos.x(), pos.y(), theta);
    }
    
    // Find active segment and get position
    int active_idx = findActiveSegment(time);
    const auto& segment = segments_[active_idx];
    double local_time = segment.getLocalTime(time);
    
    Eigen::Vector2d pos = segment.position_traj.getPosition(local_time);
    double theta = segment.orientation_traj.getPosition(local_time);
    
    return Eigen::Vector3d(pos.x(), pos.y(), theta);
}

Eigen::Vector3d AdvancedMotionPlanner::getVelocity(double time) const {
    if (!isValid()) return Eigen::Vector3d::Zero();
    
    if (time <= 0.0 || time >= total_time_) {
        return Eigen::Vector3d::Zero(); // Zero velocity at start and end
    }
    
    // Find active segment and get velocity
    int active_idx = findActiveSegment(time);
    const auto& segment = segments_[active_idx];
    double local_time = segment.getLocalTime(time);
    
    Eigen::Vector2d vel = segment.position_traj.getVelocity(local_time);
    double omega = segment.orientation_traj.getVelocity(local_time);
    
    return Eigen::Vector3d(vel.x(), vel.y(), omega);
}

double AdvancedMotionPlanner::getTotalTime() const {
    return total_time_;
}

} // namespace ctrl