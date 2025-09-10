#include "ObstacleCollisionChecker.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

ObstacleCollisionChecker ObstacleCollisionChecker::copy() const {
    ObstacleCollisionChecker checker(obstacle_, maxSpeed_);
    checker.nextFrontTimeOffset_ = nextFrontTimeOffset_;
    checker.firstCollision_ = firstCollision_;
    return checker;
}

void ObstacleCollisionChecker::stepFront(const Eigen::Vector2d& robotPos, const Eigen::Vector2d& robotVel, double timeOffset) {
    if (skipCheck(robotPos, robotVel, timeOffset)) {
        return;
    }

    double distance = distanceToObstacle(robotPos);
    bool collides = distance <= 0;
    
    // Removed debug output for cleaner logs
    
    if (collides) {
        firstCollision_ = timeOffset;
        nextFrontTimeOffset_ = std::numeric_limits<double>::infinity();
        
        std::cout << "[ObstacleCollisionChecker] COLLISION detected with " << obstacle_->getIdentifier() 
                  << " at t=" << timeOffset << "s, distance=" << distance << "mm" << std::endl;
    } else {
        nextFrontTimeOffset_ = timeOffset + getTimeToNextCheck(distance);
        
        // Debug output for far obstacles
        if (distance > 500) { // Only log if obstacle is more than 500mm away
            std::cout << "[ObstacleCollisionChecker] " << obstacle_->getIdentifier() 
                      << " at distance=" << distance << "mm, next check at t=" 
                      << nextFrontTimeOffset_ << "s (adaptive step)" << std::endl;
        }
    }
}

bool ObstacleCollisionChecker::skipCheck(const Eigen::Vector2d& robotPos, const Eigen::Vector2d& robotVel, double timeOffset) const {
    // Skip check logic
    if (timeOffset < nextFrontTimeOffset_) {
        return true; // Not time to check yet
    }
    
    if (hasCollision()) {
        return true; // Already found collision
    }
    
    // Use canCollide method for smart skipping
    if (!obstacle_->canCollide(robotPos, timeOffset, robotVel)) {
        return true; // Obstacle says it can't collide (e.g., too far away)
    }
    
    return false;
}

double ObstacleCollisionChecker::distanceToObstacle(const Eigen::Vector2d& robotPos) const {
    // Use obstacle's distance calculation with robot radius margin
    const double robotRadius = 90.0; // 90mm SSL robot radius
    double raw_distance = obstacle_->distanceTo(robotPos);
    double collision_distance = raw_distance - robotRadius;
    
    // Removed debug output for performance
    
    return collision_distance;
}

double ObstacleCollisionChecker::getTimeToNextCheck(double distance) const {
    // Adaptive time calculation
    double dist = std::max(0.0, distance / 1000.0); // Convert mm to m
    double combinedSpeed = maxSpeed_ + obstacle_->getMaxSpeed();
    
    if (combinedSpeed <= 0) {
        return 0;
    }
    
    // This is the KEY optimization - check more frequently when obstacles are close
    double timeStep = dist / combinedSpeed;
    
    return timeStep;
}

} // namespace ctrl