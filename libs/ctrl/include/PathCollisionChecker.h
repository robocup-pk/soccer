#pragma once

#include "ObstacleCollisionChecker.h"
#include "TrajPath.h"
#include "PathFinderResult.h"
#include <vector>
#include <memory>

namespace ctrl {

/**
 * @brief EXACT copy of Sumatra's PathCollisionChecker
 * Check for collisions on a specific path for multiple obstacles.
 */
class PathCollisionChecker {
private:
    TrajPath path_;
    std::vector<ObstacleCollisionChecker> obstacleCollisionCheckers_;
    double timeOffset_;

public:
    PathCollisionChecker(const TrajPath& path, 
                        const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                        double initialTimeOffset);
    
    // EXACT copy of Sumatra's factory method
    static PathCollisionChecker ofPath(const TrajPath& path,
                                      const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                                      double initialTimeOffset);
    
    // Core collision checking methods (EXACT copy of Sumatra)
    PathFinderResult checkForCollisions(double maxTime);
    
    // Getters
    const TrajPath& getPath() const { return path_; }
    
private:
    // EXACT copy of Sumatra's collision detection logic
    void updateTimeOffset(double maxTime);
    std::vector<PathFinderCollision> getCollisions() const;
};

} // namespace ctrl