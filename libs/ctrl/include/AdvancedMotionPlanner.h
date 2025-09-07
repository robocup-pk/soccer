#pragma once

#include "BangBangTrajectoryFactory.h"
#include "TrajectoryXyw.h"
#include "TrajPath.h"
#include "PathFinder.h"
#include "PathFinderInput.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's trajectory planning system using PathFinder.
 *
 * This class implements the COMPLETE Advanced approach:
 * 1. Uses PathFinderInput with MoveConstraints for proper input handling
 * 2. Uses PathFinder for obstacle avoidance and path generation
 * 3. Uses TrajPath for smooth multi-waypoint motion
 * 4. Integrates complete Team Mannheim trajectory planning pipeline
 * 
 * Direct port of Advanced's complete trajectory planning system.
 */
class AdvancedMotionPlanner {
public:
    AdvancedMotionPlanner() = default;

    /**
     * @brief Plan trajectory using COMPLETE Advanced PathFinder system.
     * @param botPos Current robot position (x, y, theta)
     * @param botVel Current robot velocity (vx, vy, omega)
     * @param dest Destination position (x, y, theta)
     * @param obstacles List of obstacles for avoidance
     * @param moveConstraints Movement constraints (EXACT Advanced format)
     */
    void planTrajectory(const Eigen::Vector3d& botPos,
                       const Eigen::Vector3d& botVel,
                       const Eigen::Vector3d& dest,
                       const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                       const MoveConstraints& moveConstraints);
    
    /**
     * @brief Plan smooth trajectory through multiple waypoints (for backward compatibility).
     */
    void planSmoothTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                             double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc);

    // --- Trajectory Query Functions ---
    Eigen::Vector3d getPosition(double time) const;
    Eigen::Vector3d getVelocity(double time) const;
    double getTotalTime() const;
    bool isValid() const { return is_valid_; }
    
    // --- Advanced-style TrajPath Access ---
    TrajPath getTrajPath() const { return trajPath_; }

private:
    // --- Complete Advanced system components ---
    TrajPath trajPath_;
    PathFinder pathFinder_;
    bool is_valid_{false};
    
    // --- Helper methods ---
    double findOptimalConnectionTime(double segmentDuration, double maxVel) const;
};

} // namespace ctrl