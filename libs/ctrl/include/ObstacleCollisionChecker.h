#pragma once

#include "IObstacle.h"
#include <memory>
#include <limits>

namespace ctrl {

/**
 * @brief Implementation of ObstacleCollisionChecker
 * Check for collisions for one obstacle, beginning at the front and tracking the front collision duration.
 */
class ObstacleCollisionChecker {
private:
    std::shared_ptr<IObstacle> obstacle_;
    double maxSpeed_;
    
    double firstCollision_ = std::numeric_limits<double>::infinity();
    double nextFrontTimeOffset_ = -std::numeric_limits<double>::infinity();

public:
    ObstacleCollisionChecker(std::shared_ptr<IObstacle> obstacle, double maxSpeed)
        : obstacle_(obstacle), maxSpeed_(maxSpeed) {}
    
    // Copy constructor for cloning
    ObstacleCollisionChecker copy() const;
    
    // Core collision checking (Implementation of stepFront)
    void stepFront(const Eigen::Vector2d& robotPos, const Eigen::Vector2d& robotVel, double timeOffset);
    
    // Getters (Based on Advanced)
    std::shared_ptr<IObstacle> getObstacle() const { return obstacle_; }
    double getFirstCollision() const { return firstCollision_; }
    double getNextFrontTimeOffset() const { return nextFrontTimeOffset_; }
    bool hasCollision() const { return std::isfinite(firstCollision_); }
    
private:
    // Implementation of collision logic
    bool skipCheck(const Eigen::Vector2d& robotPos, const Eigen::Vector2d& robotVel, double timeOffset) const;
    double distanceToObstacle(const Eigen::Vector2d& robotPos) const;
    double getTimeToNextCheck(double distance) const;
};

} // namespace ctrl