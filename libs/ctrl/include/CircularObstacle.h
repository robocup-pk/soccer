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
    
    bool isPointInside(const Eigen::Vector2d& point, double margin = 0.0) const override {
        return (point - center_).norm() <= (radius_ + margin);
    }
    
    double distanceTo(const Eigen::Vector2d& point) const override {
        return std::max(0.0, (point - center_).norm() - radius_);
    }
    
    Eigen::Vector2d nearestPointOutside(const Eigen::Vector2d& point, double margin = 0.0) const override {
        Eigen::Vector2d direction = point - center_;
        if (direction.norm() < 1e-6) {
            return center_ + Eigen::Vector2d(radius_ + margin, 0.0);
        }
        direction.normalize();
        return center_ + direction * (radius_ + margin);
    }
    
    bool collidesWith(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, 
                     double robotRadius, double timeHorizon) const override {
        return (pos - center_).norm() <= (radius_ + robotRadius);
    }
};

} // namespace ctrl