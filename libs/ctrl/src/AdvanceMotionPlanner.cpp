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
            Eigen::Vector3d prev_velocity = prev_segment.trajectory.getVelocity(prev_end_time);
            initial_velocity = prev_velocity.head<2>();
            initial_angular_velocity = prev_velocity.z();
        }
        
        // Create separate XY and orientation trajectories
        BangBangTrajectory2D xy_traj = factory_.sync(
            start_pos, end_pos, initial_velocity, maxVel, maxAcc
        );
        
        BangBangTrajectory1DOrient orient_traj = factory_.orientation(
            start_theta, end_theta, initial_angular_velocity, maxOmega, maxOmegaAcc
        );
        
        // Combine into unified TrajectoryXyw (true TIGERs approach)
        segment.trajectory = TrajectoryXyw(xy_traj, orient_traj);
        segment.duration = segment.trajectory.getTotalTime();
        
        cumulative_time += segment.duration;
        segments_.push_back(segment);
        
        std::cout << "[AdvancedMotionPlanner] Segment " << i << ": " 
                  << "pos_time=" << xy_traj.getTotalTime() << "s, "
                  << "orient_time=" << orient_traj.getTotalTime() << "s, "
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
        return segments_[0].trajectory.getPosition(0.0);
    }
    
    if (time >= total_time_) {
        // Return last waypoint
        const auto& last_segment = segments_.back();
        return last_segment.trajectory.getPosition(last_segment.duration);
    }
    
    // Find active segment and get position
    int active_idx = findActiveSegment(time);
    const auto& segment = segments_[active_idx];
    double local_time = segment.getLocalTime(time);
    
    return segment.trajectory.getPosition(local_time);
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
    
    return segment.trajectory.getVelocity(local_time);
}

double AdvancedMotionPlanner::getTotalTime() const {
    return total_time_;
}

} // namespace ctrl