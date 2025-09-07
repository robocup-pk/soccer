#pragma once

#include "IObstacle.h"
#include <Eigen/Dense>
#include <string>

namespace ctrl {

/**
 * @brief Simple circular obstacle for testing Sumatra PathFinder system
 */
class CircularObstacle : public IObstacle {
private:
    Eigen::Vector2d center_;
    double radius_;
    std::string id_;
    
public:
    CircularObstacle(const Eigen::Vector2d& center, double radius, const std::string& id = "CircularObstacle")
        : center_(center), radius_(radius), id_(id) {}
    
    std::string getIdentifier() const override { return id_; }
    bool isMotionLess() const override { return true; }
    
    // Getter for center position (needed for simple collision checking)
    Eigen::Vector2d getCenter() const { return center_; }
    double getRadius() const { return radius_; }
    
    bool isPointInside(const Eigen::Vector2d& point, double margin = 0.0) const override {
        return (point - center_).norm() <= (radius_ + margin);
    }
    
    double distanceTo(const Eigen::Vector2d& point) const override {
        double distance_to_center = (point - center_).norm();
        double surface_distance = std::max(0.0, distance_to_center - radius_);
        
        // Convert to mm for consistency
        return surface_distance * 1000.0;
    }
    
    Eigen::Vector2d nearestPointOutside(const Eigen::Vector2d& point, double margin = 0.0) const override {
        Eigen::Vector2d direction = point - center_;
        if (direction.norm() < 1e-6) {
            return center_ + Eigen::Vector2d(radius_ + margin, 0.0);
        }
        direction.normalize();
        return center_ + direction * (radius_ + margin);
    }
    
    // EXACT copy of Sumatra's collision checking interface
    bool canCollide(const Eigen::Vector2d& robotPos, double timeOffset, const Eigen::Vector2d& robotVel) const override {
        // Smart optimization: skip collision check if obstacle is very far
        double distance = distanceTo(robotPos);
        
        // Skip if obstacle is very far away (more than 2 meters)
        if (distance > 2000.0) { // 2000mm = 2m
            return false;
        }
        
        // Also consider robot velocity - if moving away from obstacle, might not collide
        if (robotVel.norm() > 0.1) { // Robot is moving
            Eigen::Vector2d toObstacle = center_ - robotPos;
            if (toObstacle.dot(robotVel.normalized()) < 0) { // Moving away from obstacle
                // If far enough and moving away, skip collision check
                if (distance > 500.0) { // 500mm margin
                    return false;
                }
            }
        }
        
        return true; // Can potentially collide
    }
    
    double getMaxSpeed() const override { 
        return 0.0; // Stationary obstacle
    }
    
    // Deprecated method for backward compatibility
    bool collidesWith(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, 
                     double robotRadius, double timeHorizon) const override {
        return (pos - center_).norm() <= (radius_ + robotRadius);
    }
};

} // namespace ctrl