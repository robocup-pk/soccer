#pragma once

#include "TrajPath.h"
#include "PathFinderInput.h"
#include "PathFinderResult.h"
#include "IObstacle.h"
#include <vector>
#include <memory>
#include <optional>

namespace ctrl {

/**
 * @brief EXACT copy of Sumatra's PathFinder.java
 * Main pathfinding class that generates smooth trajectories through obstacles
 */
class PathFinder {
private:
    std::vector<std::shared_ptr<IObstacle>> obstacles_;
    MoveConstraints moveConstraints_;
    
public:
    PathFinder() = default;
    
    /**
     * Calculate path from input (EXACT copy of Sumatra's calcPath method)
     * @return Optional PathFinderResult (like Sumatra's Optional<PathFinderResult>)
     */
    std::optional<PathFinderResult> calcPath(const PathFinderInput& input);
    
    /**
     * Check if direct path is possible without obstacles
     */
    bool isDirectPathPossible(const PathFinderInput& input);
    
    /**
     * Generate waypoints around obstacles (simplified for now)
     */
    std::vector<Eigen::Vector2d> generateWaypoints(const PathFinderInput& input);
    
    /**
     * Create smooth path through multiple waypoints (KEY method!)
     */
    TrajPath createSmoothPath(const std::vector<Eigen::Vector2d>& waypoints,
                             const Eigen::Vector2d& startVel,
                             const MoveConstraints& mc);
};

} // namespace ctrl