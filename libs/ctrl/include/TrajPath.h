#pragma once

#include "BangBangTrajectoryFactory.h"
#include "TrajectoryXyw.h"
#include "MoveConstraints.h"
#include <vector>
#include <memory>
#include <iostream>

namespace ctrl {

/**
 * @brief EXACT copy of Sumatra's TrajPath - A path of connected trajectory segments for smooth multi-waypoint motion.
 * 
 * This implements the REAL Sumatra approach for smooth trajectories:
 * - Chains trajectory segments together BEFORE the robot stops
 * - Each segment starts with position/velocity from previous segment at connection time
 * - Connection time is before segment ends (maintaining non-zero velocity)
 * 
 * Direct port of: /Sumatra/modules/sumatra-pathfinder/src/main/java/edu/tigers/sumatra/pathfinder/finder/TrajPath.java
 */
class TrajPath {
private:
    TrajectoryXyw trajectory_;
    double tEnd_;
    std::shared_ptr<TrajPath> child_;
    
public:
    // Default constructor
    TrajPath() : tEnd_(0.0), child_(nullptr) {}
    
    // Private constructor for internal use
    TrajPath(const TrajectoryXyw& trajectory, double tEnd, std::shared_ptr<TrajPath> child)
        : trajectory_(trajectory), tEnd_(tEnd), child_(child) {
        // Assert tEnd >= -0.2 (like Sumatra)
        if (tEnd < -0.2) {
            std::cerr << "[TrajPath] Invalid tEnd: " << tEnd << std::endl;
        }
    }
    
    /**
     * Create a new path with the given inputs (EXACT copy of Sumatra's with() method)
     */
    static TrajPath with(const MoveConstraints& mc, const Eigen::Vector2d& curPos, 
                        const Eigen::Vector2d& curVel, const Eigen::Vector2d& dest);
    
    /**
     * Create a new path with full state (position, velocity, orientation)
     */
    static TrajPath with(const Eigen::Vector2d& curPos, const Eigen::Vector2d& curVel, 
                        double curTheta, double curOmega,
                        const Eigen::Vector2d& dest, double destTheta,
                        double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc);
    
    /**
     * Append a new segment to this path at the given time (EXACT copy of Sumatra's append() method)
     */
    TrajPath append(double connectionTime, const Eigen::Vector2d& dest, double destTheta,
                   double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc);
    
    /**
     * Get next destination at given time (used by Sumatra's executePath)
     */
    Eigen::Vector2d getNextDestination(double t) const;
    
    // Trajectory interface (EXACT copy of Sumatra's methods)
    Eigen::Vector3d getPosition(double t) const;
    Eigen::Vector3d getVelocity(double t) const; 
    Eigen::Vector3d getAcceleration(double t) const;
    Eigen::Vector3d getFinalDestination() const;
    double getTotalTime() const;
    
    // For Sumatra compatibility
    Eigen::Vector3d getPositionMM(double t) const { return getPosition(t) * 1000.0; } // Convert m to mm
    double getMaxSpeed() const;  // Get maximum speed along path
    
private:
    /**
     * Connect this path with another path at the given time (EXACT copy of Sumatra's connect() method)
     */
    TrajPath connect(const TrajPath& path, double tConnect);
    
    BangBangTrajectoryFactory factory_;
};

} // namespace ctrl