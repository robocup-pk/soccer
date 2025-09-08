#pragma once

#include "BangBangTrajectoryFactory.h"
#include "TrajectoryXyw.h"
#include "MoveConstraints.h"
#include <vector>
#include <memory>
#include <iostream>

namespace ctrl {

/**
 * @brief A path of connected trajectory segments for smooth multi-waypoint motion.
 * 
 * This implements smooth trajectories:
 * - Chains trajectory segments together BEFORE the robot stops
 * - Each segment starts with position/velocity from previous segment at connection time
 * - Connection time is before segment ends (maintaining non-zero velocity)
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
        // Assert tEnd >= -0.2
        if (tEnd < -0.2) {
            std::cerr << "[TrajPath] Invalid tEnd: " << tEnd << std::endl;
        }
    }
    
    /**
     * Create a new path with the given inputs
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
     * Append a new segment to this path at the given time
     */
    TrajPath append(double connectionTime, const Eigen::Vector2d& dest, double destTheta,
                   double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc);
    
    /**
     * Get next destination at given time
     */
    Eigen::Vector2d getNextDestination(double t) const;
    
    // Trajectory interface
    Eigen::Vector3d getPosition(double t) const;
    Eigen::Vector3d getVelocity(double t) const; 
    Eigen::Vector3d getAcceleration(double t) const;
    Eigen::Vector3d getFinalDestination() const;
    double getTotalTime() const;
    
    // Position in millimeters
    Eigen::Vector3d getPositionMM(double t) const { return getPosition(t) * 1000.0; } // Convert m to mm
    double getMaxSpeed() const;  // Get maximum speed along path
    
private:
    /**
     * Connect this path with another path at the given time
     */
    TrajPath connect(const TrajPath& path, double tConnect);
    
    BangBangTrajectoryFactory factory_;
};

} // namespace ctrl