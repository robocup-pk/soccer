#pragma once

#include "BangBangTrajectoryFactory.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief Complete TIGERs Mannheim-style motion planner using BangBang trajectories.
 *
 * This class implements the true TIGERs approach:
 * 1. Takes waypoints from a pathfinder (e.g., RRTX).
 * 2. Creates optimal BangBang trajectories between consecutive waypoints using BangBangTrajectoryFactory.
 * 3. Handles both position (XY) and orientation (theta) trajectories separately.
 * 4. Provides a unified time-parameterized interface for TrajectoryTracker.
 * 
 * This is a direct implementation of the Sumatra trajectory planning approach.
 */
class AdvancedMotionPlanner {
public:
    AdvancedMotionPlanner() = default;

    /**
     * @brief Plans a complete trajectory from a list of waypoints using BangBang trajectories.
     * @param waypoints List of 3D waypoints (x, y, theta)
     * @param maxVel Maximum linear velocity [m/s]
     * @param maxAcc Maximum linear acceleration [m/s²]
     * @param maxOmega Maximum angular velocity [rad/s]
     * @param maxOmegaAcc Maximum angular acceleration [rad/s²]
     */
    void plan(const std::vector<Eigen::Vector3d>& waypoints,
              double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc);

    // --- Trajectory Query Functions ---
    Eigen::Vector3d getPosition(double time) const;
    Eigen::Vector3d getVelocity(double time) const;
    double getTotalTime() const;
    bool isValid() const { return !segments_.empty(); }

private:
    /**
     * @brief Trajectory segment combining position and orientation trajectories
     */
    struct TrajectorySegment {
        BangBangTrajectory2D position_traj;        // XY position trajectory
        BangBangTrajectory1DOrient orientation_traj; // Theta orientation trajectory
        double start_time{0.0};                    // Absolute start time of this segment
        double duration{0.0};                      // Duration of this segment
        
        bool isActive(double time) const {
            return time >= start_time && time <= (start_time + duration);
        }
        
        double getLocalTime(double time) const {
            return std::max(0.0, time - start_time);
        }
    };

    // --- Helper Functions ---
    int findActiveSegment(double time) const;
    
    // --- Trajectory Data ---
    std::vector<TrajectorySegment> segments_;
    BangBangTrajectoryFactory factory_;
    
    // --- Robot limits ---
    double max_velocity_;
    double max_acceleration_;
    double max_angular_velocity_;
    double max_angular_acceleration_;
    
    // --- Cached values ---
    double total_time_{0.0};
};

} // namespace ctrl