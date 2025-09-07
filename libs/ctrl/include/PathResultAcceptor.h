#pragma once

#include "PathFinderResult.h"

namespace ctrl {

/**
 * @brief EXACT copy of Advanced's path acceptance logic
 * Determines whether a path with collisions is acceptable
 */
class PathResultAcceptor {
public:
    /**
     * Accept paths based on Advanced's MotionLessObstacleResultAcceptor logic
     * @param result PathFinder result to evaluate
     * @return true if path is acceptable despite collisions
     */
    static bool acceptPath(const PathFinderResult& result);

private:
    // Configuration parameters (Balanced for good obstacle avoidance demonstration)
    static constexpr double COLLISION_TIME_THRESHOLD = 2.0; // Accept if collision is > 2s away
    static constexpr double DISTANCE_THRESHOLD = 0.3;      // Accept if collision is > 30% of total path
};

} // namespace ctrl