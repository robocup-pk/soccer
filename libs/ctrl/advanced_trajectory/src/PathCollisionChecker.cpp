#include "PathCollisionChecker.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

PathCollisionChecker::PathCollisionChecker(const TrajPath& path, 
                                          const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                                          double initialTimeOffset) 
    : path_(path), timeOffset_(initialTimeOffset) {
    
    double maxSpeed = path.getMaxSpeed();
    
    // Create collision checker for each obstacle
    obstacleCollisionCheckers_.reserve(obstacles.size());
    for (const auto& obstacle : obstacles) {
        obstacleCollisionCheckers_.emplace_back(obstacle, maxSpeed);
    }
    
    std::cout << "[PathCollisionChecker] Created with " << obstacles.size() 
              << " obstacles, maxSpeed=" << maxSpeed << "m/s, initialTimeOffset=" 
              << initialTimeOffset << "s" << std::endl;
}

PathCollisionChecker PathCollisionChecker::ofPath(const TrajPath& path,
                                                  const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                                                  double initialTimeOffset) {
    return PathCollisionChecker(path, obstacles, initialTimeOffset);
}

PathFinderResult PathCollisionChecker::checkForCollisions(double maxTime) {
    std::cout << "[PathCollisionChecker] Checking collisions for path, maxTime=" << maxTime << "s" << std::endl;
    
    double totalTime = std::min(maxTime, path_.getTotalTime());
    
    // Collision detection loop
    while (timeOffset_ < totalTime) {
        // Get robot position and velocity at current time
        Eigen::Vector3d pos3d = path_.getPosition(timeOffset_);
        Eigen::Vector3d vel3d = path_.getVelocity(timeOffset_);
        
        Eigen::Vector2d robotPos = pos3d.head<2>();
        Eigen::Vector2d robotVel = vel3d.head<2>();
        
        // Step all obstacle checkers
        double minNextTimeOffset = std::numeric_limits<double>::infinity();
        for (auto& checker : obstacleCollisionCheckers_) {
            checker.stepFront(robotPos, robotVel, timeOffset_);
            minNextTimeOffset = std::min(minNextTimeOffset, checker.getNextFrontTimeOffset());
        }
        
        // Check if any collision found
        bool hasCollision = false;
        for (const auto& checker : obstacleCollisionCheckers_) {
            if (checker.hasCollision()) {
                hasCollision = true;
                break;
            }
        }
        
        if (hasCollision) {
            std::cout << "[PathCollisionChecker] Collision found, returning result with collisions" << std::endl;
            auto collisions = getCollisions();
            return PathFinderResult::withCollision(path_, collisions);
        }
        
        // Update time offset using the adaptive approach
        if (std::isfinite(minNextTimeOffset) && minNextTimeOffset > timeOffset_) {
            timeOffset_ = minNextTimeOffset;
        } else {
            // Fallback increment if no obstacles provide next time
            timeOffset_ += 0.02; // 20ms steps as fallback
        }
    }
    
    std::cout << "[PathCollisionChecker] No collisions found, path is collision-free" << std::endl;
    return PathFinderResult::success(path_);
}

std::vector<PathFinderCollision> PathCollisionChecker::getCollisions() const {
    std::vector<PathFinderCollision> collisions;
    
    for (const auto& checker : obstacleCollisionCheckers_) {
        if (checker.hasCollision()) {
            collisions.emplace_back(checker.getFirstCollision(), 
                                   checker.getObstacle()->getIdentifier());
        }
    }
    
    return collisions;
}

} // namespace ctrl