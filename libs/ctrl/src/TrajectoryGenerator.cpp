#include "TrajectoryGenerator.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

// Static member initialization
BangBangTrajectoryFactory TrajectoryGenerator::trajectoryFactory_;
DestinationForTimedPositionCalc TrajectoryGenerator::offsetCalc_;

BangBangTrajectory2D TrajectoryGenerator::generatePositionTrajectory(const MoveConstraints& mc,
                                                                     const Eigen::Vector2d& curPos,
                                                                     const Eigen::Vector2d& curVel, 
                                                                     const Eigen::Vector2d& dest) {
    
    std::cout << "[TrajectoryGenerator] Generating position trajectory (EXACT Sumatra approach)" << std::endl;
    std::cout << "  From: (" << curPos.x() << ", " << curPos.y() << ")" << std::endl;
    std::cout << "  To: (" << dest.x() << ", " << dest.y() << ")" << std::endl;
    std::cout << "  Vel: (" << curVel.x() << ", " << curVel.y() << ")" << std::endl;
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 64-81)
    if (mc.getPrimaryDirection().norm() < 1e-6) {
        // Use synchronous trajectory (no primary direction)
        std::cout << "  Using sync trajectory (no primary direction)" << std::endl;
        return trajectoryFactory_.sync(curPos, dest, curVel, mc.getVelMax(), mc.getAccMaxDerived());
    } else {
        // Use asynchronous trajectory (with primary direction) - need to convert to BangBangTrajectory2D
        std::cout << "  Using async trajectory with primary direction: (" 
                  << mc.getPrimaryDirection().x() << ", " << mc.getPrimaryDirection().y() << ")" << std::endl;
        BangBangTrajectory2DAsync asyncTraj = trajectoryFactory_.async(curPos, dest, curVel, mc.getVelMax(), 
                                                                       mc.getAccMaxDerived(), mc.getPrimaryDirection());
        
        // Return the child trajectory which is a BangBangTrajectory2D
        return asyncTraj.getChild();
    }
}

BangBangTrajectory1DOrient TrajectoryGenerator::generateRotationTrajectory(double curOrientation,
                                                                           double curAVel,
                                                                           double targetAngle,
                                                                           const MoveConstraints& mc) {
    
    std::cout << "[TrajectoryGenerator] Generating rotation trajectory" << std::endl;
    std::cout << "  From: " << curOrientation << " rad" << std::endl;
    std::cout << "  To: " << targetAngle << " rad" << std::endl;
    std::cout << "  Angular vel: " << curAVel << " rad/s" << std::endl;
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 115-118)
    return trajectoryFactory_.orientation(curOrientation, targetAngle, curAVel,
                                         mc.getVelMaxW(), mc.getAccMaxW());
}

Eigen::Vector2d TrajectoryGenerator::generateVirtualPositionToReachPointInTime(const MoveConstraints& mc,
                                                                              const Eigen::Vector2d& curPos,
                                                                              const Eigen::Vector2d& curVel,
                                                                              const Eigen::Vector2d& dest,
                                                                              double targetTime) {
    
    std::cout << "[TrajectoryGenerator] Generating virtual position for timed arrival" << std::endl;
    std::cout << "  Target time: " << targetTime << "s" << std::endl;
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 178-201)
    if (mc.getPrimaryDirection().norm() < 1e-6) {
        // Use synchronous calculation
        return offsetCalc_.destinationForBangBang2dSync(curPos, dest, curVel,
                                                       mc.getVelMax(), mc.getAccMax(), targetTime);
    } else {
        // Use asynchronous calculation  
        return offsetCalc_.destinationForBangBang2dAsync(curPos, dest, curVel,
                                                        mc.getVelMax(), mc.getAccMax(), targetTime,
                                                        mc.getPrimaryDirection());
    }
}

BangBangTrajectory2D TrajectoryGenerator::generatePositionTrajectoryToReachPointInTime(const MoveConstraints& mc,
                                                                                       const Eigen::Vector2d& curPos,
                                                                                       const Eigen::Vector2d& curVel,
                                                                                       const Eigen::Vector2d& dest,
                                                                                       double targetTime) {
    
    std::cout << "[TrajectoryGenerator] Generating timed trajectory" << std::endl;
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 226-232)
    // This can get optimized as an addition to the generateVirtualPositionToReachPointInTime could directly create
    // full trajectories and not only a position.
    Eigen::Vector2d virtualDest = generateVirtualPositionToReachPointInTime(mc, curPos, curVel, dest, targetTime);
    return generatePositionTrajectory(mc, curPos, curVel, virtualDest);
}

bool TrajectoryGenerator::isComeToAStopFaster(const MoveConstraints& mc,
                                             const Eigen::Vector2d& curPos,
                                             const Eigen::Vector2d& curVel,
                                             const Eigen::Vector2d& dest) {
    
    std::cout << "[TrajectoryGenerator] Checking if come-to-stop is faster" << std::endl;
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 121-141)
    auto trajWithoutComeToAStop = generatePositionTrajectory(mc, curPos, curVel, dest);
    
    auto stateAfterStop = stateAfterComeToAStop(mc, 
                                               trajWithoutComeToAStop.getPosition(0.0), 
                                               trajWithoutComeToAStop.getVelocity(0.0));
    
    auto trajAfterBrk = generatePositionTrajectory(mc, 
                                                  stateAfterStop.pos,
                                                  stateAfterStop.vel, 
                                                  dest);
    
    return trajAfterBrk.getTotalTime() + stateAfterStop.lookAhead + 0.01 
           < trajWithoutComeToAStop.getTotalTime();
}

TrajectoryGenerator::StateAfterComeToAStop TrajectoryGenerator::stateAfterComeToAStop(const MoveConstraints& mc, 
                                                                                      const Eigen::Vector2d& s0, 
                                                                                      const Eigen::Vector2d& v0) {
    
    // EXACT copy of Sumatra's logic (TrajectoryGenerator.java lines 262-274)
    double lookAhead = 0.05; // 5 AI iterations
    double acc = mc.getBrkMax() * 0.9;
    
    double tBreak = std::min(v0.norm() / acc, lookAhead);
    Eigen::Vector2d a0 = -v0.normalized() * acc;
    Eigen::Vector2d v1 = a0 * tBreak + v0;
    Eigen::Vector2d s1 = s0 + (v0 * tBreak) + (a0 * 0.5 * tBreak * tBreak);
    
    return StateAfterComeToAStop(s1, v1, lookAhead);
}

} // namespace ctrl