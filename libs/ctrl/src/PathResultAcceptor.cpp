#include "PathResultAcceptor.h"
#include <iostream>
#include <cmath>

namespace ctrl {

bool PathResultAcceptor::acceptPath(const PathFinderResult& result) {
    // ACTUAL standard logic for smart path acceptance
    
    if (result.isCollisionFree()) {
        std::cout << "[PathResultAcceptor] Accepting collision-free path" << std::endl;
        return true; // Always accept collision-free paths
    }
    
    // Use the smart acceptance criteria
    double firstCollisionTime = result.getFirstCollisionTime();
    double totalTime = result.getTrajectory().getTotalTime();
    
    // Accept if collision happens late in the trajectory
    if (firstCollisionTime > COLLISION_TIME_THRESHOLD) {
        std::cout << "[PathResultAcceptor] Accepting path: collision at t=" << firstCollisionTime 
                  << "s is > " << COLLISION_TIME_THRESHOLD << "s threshold" << std::endl;
        return true;
    }
    
    // Accept based on relative distance to collision
    Eigen::Vector3d startPos = result.getTrajectory().getPosition(0);
    Eigen::Vector3d endPos = result.getTrajectory().getFinalDestination();
    Eigen::Vector3d collisionPos = result.getTrajectory().getPosition(firstCollisionTime);
    
    double startToEndDist = (endPos.head<2>() - startPos.head<2>()).norm();
    double collisionToEndDist = (endPos.head<2>() - collisionPos.head<2>()).norm();
    
    if (startToEndDist > 0) {
        double relativeDist = collisionToEndDist / startToEndDist;
        
        if (relativeDist < DISTANCE_THRESHOLD) {
            std::cout << "[PathResultAcceptor] Accepting path: collision at " 
                      << (relativeDist * 100) << "% of total distance (< " 
                      << (DISTANCE_THRESHOLD * 100) << "% threshold)" << std::endl;
            return true;
        }
    }
    
    std::cout << "[PathResultAcceptor] Rejecting path: collision at t=" << firstCollisionTime 
              << "s too early and too close to start" << std::endl;
    return false;
}

} // namespace ctrl