#pragma once

#include "TrajPath.h"
#include <vector>
#include <algorithm>
#include <limits>

namespace ctrl {

// Forward declaration for collision info (simplified)
struct PathFinderCollision {
    double firstCollisionTime;
    std::string obstacleId;
    
    PathFinderCollision(double time, const std::string& id) 
        : firstCollisionTime(time), obstacleId(id) {}
    
    double getFirstCollisionTime() const { return firstCollisionTime; }
};

/**
 * @brief EXACT copy of Advanced's PathFinderResult.java
 * Result wrapper for path finding operations
 */
class PathFinderResult {
private:
    TrajPath trajectory_;
    std::vector<PathFinderCollision> collisions_;
    
public:
    PathFinderResult(const TrajPath& trajectory, const std::vector<PathFinderCollision>& collisions)
        : trajectory_(trajectory), collisions_(collisions) {}
    
    // Static factory methods (EXACT copy of Advanced)
    static PathFinderResult success(const TrajPath& trajectory) {
        return PathFinderResult(trajectory, {});
    }
    
    static PathFinderResult withCollision(const TrajPath& trajectory, 
                                         const std::vector<PathFinderCollision>& collisions) {
        return PathFinderResult(trajectory, collisions);
    }
    
    // Getters (EXACT copy of Advanced interface)
    const TrajPath& getTrajectory() const { return trajectory_; }
    const std::vector<PathFinderCollision>& getCollisions() const { return collisions_; }
    
    PathFinderResult merge(const PathFinderResult& result) const {
        std::vector<PathFinderCollision> allCollisions = collisions_;
        const auto& otherCollisions = result.getCollisions();
        allCollisions.insert(allCollisions.end(), otherCollisions.begin(), otherCollisions.end());
        return PathFinderResult(trajectory_, allCollisions);
    }
    
    bool isCollisionFree() const {
        return collisions_.empty();
    }
    
    double getFirstCollisionTime() const {
        if (collisions_.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        
        auto minElement = std::min_element(collisions_.begin(), collisions_.end(),
            [](const PathFinderCollision& a, const PathFinderCollision& b) {
                return a.getFirstCollisionTime() < b.getFirstCollisionTime();
            });
        
        return minElement->getFirstCollisionTime();
    }
    
    const PathFinderCollision* getFirstCollision() const {
        if (collisions_.empty()) {
            return nullptr;
        }
        
        auto minElement = std::min_element(collisions_.begin(), collisions_.end(),
            [](const PathFinderCollision& a, const PathFinderCollision& b) {
                return a.getFirstCollisionTime() < b.getFirstCollisionTime();
            });
        
        return &(*minElement);
    }
};

} // namespace ctrl