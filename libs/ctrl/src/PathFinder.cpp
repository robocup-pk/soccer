#include "PathFinder.h"
#include "PathCollisionChecker.h"
#include "PathResultAcceptor.h"
#include "TrajectoryGenerator.h"
#include "CircularObstacle.h"
#include "PathfindingConfig.h"
#include <iostream>

namespace ctrl {

// EXACT copy of Sumatra's PathFinder configuration
static constexpr double TIME_CORRECTION_RANGE = 0.02;
static constexpr double INITIAL_TIME_OFFSET = 0.5; // Start checking collision after 0.5s to avoid immediate false collisions

std::optional<PathFinderResult> PathFinder::calcPath(const PathFinderInput& input) {
    // Configurable obstacle avoidance system
    
    if (config_.debug.enable_pathfinding_logs) {
        std::cout << "[PathFinder] Configurable obstacle avoidance approach" << std::endl;
        std::cout << "[PathFinder] From (" << input.getPos().x() << "," << input.getPos().y() 
                  << ") to (" << input.getDest().x() << "," << input.getDest().y() << ")" << std::endl;
        std::cout << "[PathFinder] " << input.getObstacles().size() << " obstacles to consider" << std::endl;
        std::cout << "[PathFinder] Safety margin: " << config_.obstacle_avoidance.base_safety_margin_m << "m" << std::endl;
    }
    
    // Step 1: Check if direct path is clear (simple distance check)
    if (input.getObstacles().empty() || isDirectPathClear(input)) {
        TrajPath directPath = createPath(input, input.getDest());
        std::cout << "[PathFinder] Direct path is clear - using it!" << std::endl;
        return PathFinderResult::success(directPath);
    }
    
    std::cout << "[PathFinder] Direct path blocked, generating waypoint detour..." << std::endl;
    
    // Step 2: Generate simple waypoint that goes around obstacles
    std::vector<Eigen::Vector2d> waypoints = generateWaypoints(input);
    
    for (size_t i = 1; i < waypoints.size() - 1; ++i) { // Skip start and end
        std::cout << "[PathFinder] Trying waypoint " << i << ": (" 
                  << waypoints[i].x() << "," << waypoints[i].y() << ")" << std::endl;
        
        // Check if path to waypoint is clear AND path from waypoint to destination is clear
        bool pathToWaypointClear = isPathClear(input.getPos(), waypoints[i], input.getObstacles());
        bool pathFromWaypointClear = isPathClear(waypoints[i], input.getDest(), input.getObstacles());
        
        if (pathToWaypointClear && pathFromWaypointClear) {
            std::cout << "[PathFinder] Found complete clear path via waypoint!" << std::endl;
            
            // Create complete path through waypoint to final destination
            std::vector<Eigen::Vector2d> completePath = {input.getPos(), waypoints[i], input.getDest()};
            TrajPath completeTrajPath = createSmoothPath(completePath, input.getVel(), input.getMoveConstraints());
            
            return PathFinderResult::success(completeTrajPath);
        }
    }
    
    // Step 3: If no waypoints work, create path with largest detour TO FINAL DESTINATION
    if (waypoints.size() > 2) {
        std::cout << "[PathFinder] Creating complete detour path via first waypoint" << std::endl;
        
        // Create complete path through first waypoint to final destination
        std::vector<Eigen::Vector2d> detourPath = {input.getPos(), waypoints[1], input.getDest()};
        TrajPath completeDetourPath = createSmoothPath(detourPath, input.getVel(), input.getMoveConstraints());
        
        return PathFinderResult::success(completeDetourPath);
    }
    
    // Final fallback: direct path
    std::cout << "[PathFinder] Using direct path as last resort" << std::endl;
    TrajPath directPath = createPath(input, input.getDest());
    return PathFinderResult::success(directPath);
}

bool PathFinder::isDirectPathPossible(const PathFinderInput& input) {
    // Simple direct path check
    return isDirectPathClear(input);
}

bool PathFinder::isDirectPathClear(const PathFinderInput& input) {
    return isPathClear(input.getPos(), input.getDest(), input.getObstacles());
}

bool PathFinder::isPathClear(const Eigen::Vector2d& start, const Eigen::Vector2d& end, 
                            const std::vector<std::shared_ptr<IObstacle>>& obstacles) {
    // Configurable line-circle collision check
    for (const auto& obstacle : obstacles) {
        // Skip far obstacles if optimization is enabled
        if (config_.collision_detection.enable_distance_optimizations) {
            double maxDist = config_.collision_detection.max_check_distance_m * 1000.0; // Convert to mm
            if (obstacle->distanceTo(start) > maxDist && obstacle->distanceTo(end) > maxDist) {
                continue;
            }
        }
        
        double distanceToObstacle = distanceFromLineToObstacle(start, end, obstacle);
        double safetyMargin = config_.collision_detection.safety_margin_mm;
        
        // Use adaptive margin if enabled
        if (config_.collision_detection.enable_adaptive_margins) {
            auto circularObstacle = std::dynamic_pointer_cast<CircularObstacle>(obstacle);
            if (circularObstacle) {
                safetyMargin = (circularObstacle->getRadius() * 1000.0) * config_.collision_detection.adaptive_margin_multiplier;
            }
        }
        
        if (distanceToObstacle < safetyMargin) {
            if (config_.debug.enable_collision_logs) {
                std::cout << "[PathFinder] Collision detected: distance=" << distanceToObstacle 
                          << "mm, margin=" << safetyMargin << "mm" << std::endl;
            }
            return false;
        }
    }
    return true;
}

double PathFinder::distanceFromLineToObstacle(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                                             std::shared_ptr<IObstacle> obstacle) {
    // Simple distance from line segment to circular obstacle
    auto circularObstacle = std::dynamic_pointer_cast<CircularObstacle>(obstacle);
    if (!circularObstacle) {
        // If not a circular obstacle, use a generic distance check
        return obstacle->distanceTo((start + end) * 0.5); // Distance to midpoint
    }
    
    Eigen::Vector2d obstacleCenter = circularObstacle->getCenter();
    
    // Vector from start to end
    Eigen::Vector2d line = end - start;
    double lineLength = line.norm();
    
    if (lineLength < 1e-6) {
        // Start and end are the same point
        return (obstacleCenter - start).norm() * 1000.0; // Convert to mm
    }
    
    // Project obstacle center onto line
    double t = (obstacleCenter - start).dot(line) / (lineLength * lineLength);
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]
    
    // Find closest point on line segment
    Eigen::Vector2d closestPoint = start + t * line;
    
    // Distance from obstacle center to closest point on line
    double distance = (obstacleCenter - closestPoint).norm() * 1000.0; // Convert to mm
    
    return distance;
}

std::vector<Eigen::Vector2d> PathFinder::generateWaypoints(const PathFinderInput& input) {
    // Configurable waypoint generation system
    
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.push_back(input.getPos());
    
    const auto& obstacles = input.getObstacles();
    Eigen::Vector2d current = input.getPos();
    Eigen::Vector2d target = input.getDest();
    Eigen::Vector2d direction = (target - current).normalized();
    Eigen::Vector2d perpendicular(-direction.y(), direction.x());
    
    if (config_.debug.enable_waypoint_logs) {
        std::cout << "[PathFinder] Configurable waypoint generation for " << obstacles.size() << " obstacles" << std::endl;
    }
    
    if (!obstacles.empty()) {
        // Simple but effective: Generate waypoints in a grid pattern around obstacles
        double path_length = (target - current).norm();
        
        // Calculate appropriate offset based on obstacle sizes and configuration
        double maxObstacleRadius = 0.0;
        if (config_.obstacle_avoidance.enable_adaptive_offsets) {
            for (const auto& obstacle : obstacles) {
                auto circularObstacle = std::dynamic_pointer_cast<CircularObstacle>(obstacle);
                if (circularObstacle) {
                    maxObstacleRadius = std::max(maxObstacleRadius, circularObstacle->getRadius());
                }
            }
        } else {
            maxObstacleRadius = 0.1; // Default assumption for fixed offsets
        }
        
        // Create configurable offsets
        std::vector<double> offsets;
        if (config_.obstacle_avoidance.enable_adaptive_offsets) {
            double minOffset = maxObstacleRadius + config_.obstacle_avoidance.base_safety_margin_m;
            double maxOffset = maxObstacleRadius + config_.obstacle_avoidance.extended_safety_margin_m;
            offsets = {minOffset, -minOffset, maxOffset, -maxOffset};
        } else {
            // Use fixed multipliers from config
            for (double multiplier : config_.obstacle_avoidance.offset_multipliers) {
                offsets.push_back(multiplier * maxObstacleRadius);
            }
        }
        
        std::vector<double> fractions = config_.obstacle_avoidance.path_fractions;
        
        if (config_.debug.enable_waypoint_logs) {
            std::cout << "[PathFinder] Using offsets: ";
            for (double offset : offsets) std::cout << offset << "m ";
            std::cout << "(obstacle radius: " << maxObstacleRadius << "m)" << std::endl;
        }
        
        for (double fraction : fractions) {
            Eigen::Vector2d point_on_path = current + fraction * (target - current);
            
            for (double offset : offsets) {
                Eigen::Vector2d candidate = point_on_path + perpendicular * offset;
                
                // Simple clearance check - must be far enough from ALL obstacles
                bool waypoint_clear = true;
                for (const auto& obstacle : obstacles) {
                    double dist = obstacle->distanceTo(candidate);
                    double requiredClearance = (maxObstacleRadius + 0.1) * 1000.0; // Convert to mm + 100mm safety
                    if (dist < requiredClearance) {
                        waypoint_clear = false;
                        break;
                    }
                }
                
                if (waypoint_clear) {
                    waypoints.push_back(candidate);
                    std::cout << "[PathFinder] Added waypoint: (" << candidate.x() 
                              << "," << candidate.y() << ") with " << offset << "m offset" << std::endl;
                    
                    // Don't add too many waypoints - keep it simple
                    if (waypoints.size() >= 6) break;
                }
            }
            if (waypoints.size() >= 6) break;
        }
        
        // Fallback: If no clear waypoints, try wider circular pattern
        if (waypoints.size() == 1) { // Only start point
            std::cout << "[PathFinder] No clear waypoints found, trying wider pattern" << std::endl;
            
            // Try circular pattern around center point with appropriate radius
            Eigen::Vector2d center = current + 0.5 * (target - current);
            double radius = maxObstacleRadius + 0.3; // Just enough to clear obstacles
            
            for (double angle : {M_PI/4, -M_PI/4, 3*M_PI/4, -3*M_PI/4}) {
                Eigen::Vector2d candidate = center + radius * Eigen::Vector2d(cos(angle), sin(angle));
                
                bool waypoint_clear = true;
                for (const auto& obstacle : obstacles) {
                    double requiredClearance = (maxObstacleRadius + 0.1) * 1000.0; // Convert to mm + 100mm safety
                    if (obstacle->distanceTo(candidate) < requiredClearance) {
                        waypoint_clear = false;
                        break;
                    }
                }
                
                if (waypoint_clear) {
                    waypoints.push_back(candidate);
                    std::cout << "[PathFinder] Added circular waypoint: (" << candidate.x() 
                              << "," << candidate.y() << ")" << std::endl;
                    break; // Just one good waypoint is enough
                }
            }
        }
    }
    
    waypoints.push_back(target);
    
    std::cout << "[PathFinder] Generated " << waypoints.size() << " total waypoints" << std::endl;
    
    return waypoints;
}

TrajPath PathFinder::createPath(const PathFinderInput& input, const Eigen::Vector2d& dest) {
    // EXACT copy of Sumatra's createPath method
    return TrajPath::with(
        input.getMoveConstraints(),
        input.getPos(),
        input.getVel(),
        dest
    );
}

TrajPath PathFinder::createSmoothPath(const std::vector<Eigen::Vector2d>& waypoints,
                                     const Eigen::Vector2d& startVel,
                                     const MoveConstraints& mc) {
    // EXACT copy of Sumatra's smooth path creation using TrajPath chaining
    
    if (waypoints.size() < 2) {
        std::cerr << "[PathFinder] ERROR: Need at least 2 waypoints for path creation" << std::endl;
        return TrajPath();
    }
    
    std::cout << "[PathFinder] Creating smooth path through " << waypoints.size() << " waypoints" << std::endl;
    
    // Create initial path segment using TrajectoryGenerator (EXACT Sumatra approach)
    auto posTrajectory = TrajectoryGenerator::generatePositionTrajectory(
        mc, waypoints[0], startVel, waypoints[1]);
    auto rotTrajectory = TrajectoryGenerator::generateRotationTrajectory(
        0.0, 0.0, 0.0, mc);
    TrajectoryXyw trajectory(posTrajectory, rotTrajectory);
    TrajPath path(trajectory, trajectory.getTotalTime(), nullptr);
    
    // Chain additional segments using Sumatra's connection approach
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        std::cout << "[PathFinder] Adding waypoint " << i+1 << ": (" 
                  << waypoints[i+1].x() << "," << waypoints[i+1].y() << ")" << std::endl;
        
        // KEY: Connect at 60% of segment time (like Sumatra does)
        double segmentTime = path.getTotalTime();
        double connectionTime = segmentTime * 0.6;
        
        path = path.append(connectionTime, waypoints[i+1], 0.0,
                          mc.getVelMax(), mc.getAccMaxDerived(),
                          mc.getVelMaxW(), mc.getAccMaxW());
    }
    
    std::cout << "[PathFinder] Created smooth path with total time: " << path.getTotalTime() << "s" << std::endl;
    
    return path;
}

} // namespace ctrl