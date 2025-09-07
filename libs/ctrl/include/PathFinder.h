#pragma once

#include "TrajPath.h"
#include "PathFinderInput.h"
#include "PathFinderResult.h"
#include "IObstacle.h"
#include "PathfindingConfig.h"
#include <vector>
#include <memory>
#include <optional>

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's PathFinder.java
 * Main pathfinding class that generates smooth trajectories through obstacles
 */
class PathFinder {
private:
    std::vector<std::shared_ptr<IObstacle>> obstacles_;
    MoveConstraints moveConstraints_;
    PathfindingConfig config_;
    
public:
    PathFinder() : config_(PathfindingConfig::createBalanced()) {}
    explicit PathFinder(const PathfindingConfig& config) : config_(config) {}
    
    // Configuration management
    void setConfig(const PathfindingConfig& config) { config_ = config; }
    const PathfindingConfig& getConfig() const { return config_; }
    
    /**
     * Calculate path from input (EXACT copy of Advanced's calcPath method)
     * @return Optional PathFinderResult (like Advanced's Optional<PathFinderResult>)
     */
    std::optional<PathFinderResult> calcPath(const PathFinderInput& input);
    
    /**
     * Check if direct path is possible without obstacles
     */
    bool isDirectPathPossible(const PathFinderInput& input);
    
    /**
     * Simple collision checking methods
     */
    bool isDirectPathClear(const PathFinderInput& input);
    bool isPathClear(const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
                    const std::vector<std::shared_ptr<IObstacle>>& obstacles);
    double distanceFromLineToObstacle(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                                     std::shared_ptr<IObstacle> obstacle);
    
    /**
     * Generate waypoints around obstacles (simplified for now)
     */
    std::vector<Eigen::Vector2d> generateWaypoints(const PathFinderInput& input);
    
    /**
     * Create path to destination (EXACT copy of Advanced)
     */
    TrajPath createPath(const PathFinderInput& input, const Eigen::Vector2d& dest);
    
    /**
     * Create smooth path through multiple waypoints (KEY method!)
     */
    TrajPath createSmoothPath(const std::vector<Eigen::Vector2d>& waypoints,
                             const Eigen::Vector2d& startVel,
                             const MoveConstraints& mc);
};

} // namespace ctrl