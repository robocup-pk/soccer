#pragma once

#include "BangBangTrajectoryFactory.h" 
#include "DestinationForTimedPositionCalc.h"
#include "MoveConstraints.h"
#include "BangBangTrajectory2D.h"
#include "BangBangTrajectory1DOrient.h"
#include <Eigen/Dense>
#include <memory>

namespace ctrl {

/**
 * @brief Implementation of TrajectoryGenerator.java
 * Generate BangBang trajectories (THE MAIN ENTRY POINT for standard trajectory system)
 */
class TrajectoryGenerator {
private:
    static BangBangTrajectoryFactory trajectoryFactory_;
    static DestinationForTimedPositionCalc offsetCalc_;
    
    // Private helper struct
    struct StateAfterComeToAStop {
        Eigen::Vector2d pos;
        Eigen::Vector2d vel;
        double lookAhead;
        
        StateAfterComeToAStop(const Eigen::Vector2d& p, const Eigen::Vector2d& v, double la)
            : pos(p), vel(v), lookAhead(la) {}
    };
    
    static StateAfterComeToAStop stateAfterComeToAStop(const MoveConstraints& mc, 
                                                       const Eigen::Vector2d& s0, 
                                                       const Eigen::Vector2d& v0);

public:
    TrajectoryGenerator() = delete; // Static class as needed
    
    //************************************************************************
    // Position Trajectory (Based on standard methods)
    //************************************************************************
    
    /**
     * @param mc Movement constraints
     * @param curPos Current position [m] 
     * @param curVel Current velocity [m/s]
     * @param dest Destination [m]
     * @return Position trajectory
     */
    static BangBangTrajectory2D generatePositionTrajectory(const MoveConstraints& mc,
                                                           const Eigen::Vector2d& curPos,
                                                           const Eigen::Vector2d& curVel, 
                                                           const Eigen::Vector2d& dest);
    
    //************************************************************************
    // Rotation Trajectory (Based on standard methods)
    //************************************************************************
    
    static BangBangTrajectory1DOrient generateRotationTrajectory(double curOrientation,
                                                                double curAVel,
                                                                double targetAngle,
                                                                const MoveConstraints& mc);
    
    //************************************************************************
    // Overshoot Trajectory (Based on standard methods)
    //************************************************************************
    
    /**
     * Generate virtual position to reach point in specific time
     */
    static Eigen::Vector2d generateVirtualPositionToReachPointInTime(const MoveConstraints& mc,
                                                                     const Eigen::Vector2d& curPos,
                                                                     const Eigen::Vector2d& curVel,
                                                                     const Eigen::Vector2d& dest,
                                                                     double targetTime);
    
    static BangBangTrajectory2D generatePositionTrajectoryToReachPointInTime(const MoveConstraints& mc,
                                                                             const Eigen::Vector2d& curPos,
                                                                             const Eigen::Vector2d& curVel,
                                                                             const Eigen::Vector2d& dest,
                                                                             double targetTime);
    
    static bool isComeToAStopFaster(const MoveConstraints& mc,
                                   const Eigen::Vector2d& curPos,
                                   const Eigen::Vector2d& curVel,
                                   const Eigen::Vector2d& dest);
};

} // namespace ctrl