#pragma once

#include <Eigen/Dense>
#include <string>

namespace ctrl {

/**
 * @brief EXACT copy of Sumatra's IObstacle interface
 * Base interface for all obstacles in path planning
 */
class IObstacle {
public:
    virtual ~IObstacle() = default;
    
    // Core interface methods (EXACT copy of Sumatra)
    virtual std::string getIdentifier() const = 0;
    virtual bool isMotionLess() const = 0;
    virtual bool isPointInside(const Eigen::Vector2d& point, double margin = 0.0) const = 0;
    virtual double distanceTo(const Eigen::Vector2d& point) const = 0;
    virtual Eigen::Vector2d nearestPointOutside(const Eigen::Vector2d& point, double margin = 0.0) const = 0;
    
    // EXACT copy of Sumatra's collision checking interface
    virtual bool canCollide(const Eigen::Vector2d& robotPos, double timeOffset, const Eigen::Vector2d& robotVel) const = 0;
    virtual double getMaxSpeed() const = 0;
    
    // For collision checking (deprecated - use canCollide instead)
    virtual bool collidesWith(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, double robotRadius, double timeHorizon) const = 0;
    
    // For visualization (simplified)
    virtual void getShapes() const {} // Placeholder for drawable shapes
};

} // namespace ctrl