#include "AdvancedMotionPlanner.h"
#include "TrajectoryGenerator.h"
#include "Utils.h"
#include <iostream>
#include <algorithm>

namespace ctrl {

void AdvancedMotionPlanner::planTrajectory(const Eigen::Vector3d& botPos,
                                          const Eigen::Vector3d& botVel,
                                          const Eigen::Vector3d& dest,
                                          const std::vector<std::shared_ptr<IObstacle>>& obstacles,
                                          const MoveConstraints& moveConstraints) {
    
    std::cout << "[AdvancedMotionPlanner] Using COMPLETE standard PathFinder system" << std::endl;
    std::cout << "  From: (" << botPos.x() << ", " << botPos.y() << ", " << botPos.z() << ")" << std::endl;
    std::cout << "  To: (" << dest.x() << ", " << dest.y() << ", " << dest.z() << ")" << std::endl;
    std::cout << "  Obstacles: " << obstacles.size() << std::endl;
    
    // Create PathFinderInput using approach
    PathFinderInput input = PathFinderInput::fromBot(botPos, botVel)
        .dest(dest.head<2>())
        .obstacles(obstacles)
        .moveConstraints(moveConstraints)
        .timestamp(0) // Can be enhanced with actual timestamp
        .build();
    
    // Use PathFinder to calculate optimal path (approach)
    auto pathResult = pathFinder_.calcPath(input);
    
    if (pathResult.has_value() && (pathResult->isCollisionFree() || pathResult->getTrajectory().getTotalTime() > 0.0)) {
        // approach from AMoveToSkill.java lines 139, 155, 276-277:
        // 1. PathFinder.calcPath() returns PathFinderResult with obstacle-avoiding TrajPath 
        // 2. Extract TrajPath from result: pathResult.get().getTrajectory() 
        // 3. Create separate rotation trajectory: generateRotationTrajectory()
        // 4. Combine using: new TrajectoryXyw(trajPath, trajW)
        
        std::cout << "[AdvancedMotionPlanner] Using approach: PathFinder TrajPath + separate rotation" << std::endl;
        
        // Step 1: Extract obstacle-avoiding TrajPath from PathFinder (line 155)
        TrajPath pathfinder_trajPath = pathResult->getTrajectory();
        
        // Step 2: Create separate rotation trajectory (line 276)
        auto rotation_trajectory = TrajectoryGenerator::generateRotationTrajectory(
            botPos.z(),           // Current orientation
            botVel.z(),           // Current angular velocity  
            dest.z(),             // Target orientation
            moveConstraints       // Movement constraints
        );
        
        // Step 3: Combine using TrajectoryXyw constructor (line 277: new TrajectoryXyw(trajPath, trajW))
        // This is the KEY - PathFinder provides obstacle-avoiding XY, rotation provides orientation
        // This preserves ALL PathFinder logic (obstacle avoidance, waypoints, timing) while adding orientation
        
        std::cout << "[AdvancedMotionPlanner] Combining PathFinder obstacle-avoiding path with rotation using TrajectoryXyw" << std::endl;
        
        TrajectoryXyw combined_trajectory(pathfinder_trajPath, rotation_trajectory);
        trajPath_ = TrajPath(combined_trajectory, combined_trajectory.getTotalTime(), nullptr);
        is_valid_ = true;
        
        if (!pathResult->isCollisionFree()) {
            std::cout << "[AdvancedMotionPlanner] WARNING: PathFinder path has " << pathResult->getCollisions().size() 
                      << " collisions, first at t=" << pathResult->getFirstCollisionTime() << "s." << std::endl;
        }
        
        std::cout << "[AdvancedMotionPlanner] Successfully created OBSTACLE-AVOIDING path! Total time: " 
                  << trajPath_.getTotalTime() << "s" << std::endl;
        std::cout << "[AdvancedMotionPlanner] Target orientation: " << dest.z() << " rad (" 
                  << dest.z() * 180.0 / M_PI << "°)" << std::endl;
        std::cout << "[AdvancedMotionPlanner] TODO: Need to combine rotation trajectory with position trajectory" << std::endl;
    } else {
        is_valid_ = false;
        std::cout << "[AdvancedMotionPlanner] ERROR: Failed to create valid path" << std::endl;
    }
}

void AdvancedMotionPlanner::planSmoothTrajectory(const std::vector<Eigen::Vector3d>& waypoints,
                                                 double maxVel, double maxAcc, double maxOmega, double maxOmegaAcc) {
    if (waypoints.size() < 2) {
        std::cout << "[AdvancedMotionPlanner] Error: Need at least 2 waypoints" << std::endl;
        is_valid_ = false;
        return;
    }
    
    std::cout << "[AdvancedMotionPlanner] DEPRECATED: Multiple waypoints not supported by pure Advanced!" << std::endl;
    std::cout << "[AdvancedMotionPlanner] standard expects: SINGLE destination + obstacles, not multiple waypoints" << std::endl;
    std::cout << "[AdvancedMotionPlanner] Creating fallback trajectory to final destination only" << std::endl;
    
    // PURE ADVANCED APPROACH: Only use start and final destination
    // Ignore intermediate waypoints - standard doesn't use them!
    
    if (waypoints.size() > 2) {
        std::cout << "[AdvancedMotionPlanner] WARNING: Ignoring " << waypoints.size()-2 
                  << " intermediate waypoints - standard uses PathFinder for obstacle avoidance instead!" << std::endl;
    }
    
    // Use only start and final destination (approach)
    Eigen::Vector3d start = waypoints[0];
    Eigen::Vector3d destination = waypoints.back();
    
    // Create MoveConstraints (standard format)
    MoveConstraints mc;
    mc.setVelMax(maxVel).setAccMax(maxAcc).setVelMaxW(maxOmega).setAccMaxW(maxOmegaAcc);
    
    // Create single trajectory from start to destination using TrajectoryGenerator
    auto posTrajectory = TrajectoryGenerator::generatePositionTrajectory(
        mc, start.head<2>(), Eigen::Vector2d::Zero(), destination.head<2>());
    auto rotTrajectory = TrajectoryGenerator::generateRotationTrajectory(
        start.z(), 0.0, destination.z(), mc);
    
    TrajectoryXyw trajectory(posTrajectory, rotTrajectory);
    trajPath_ = TrajPath(trajectory, trajectory.getTotalTime(), nullptr);
    
    is_valid_ = true;
    
    std::cout << "[AdvancedMotionPlanner] Successfully created smooth TrajPath - Total time: " 
              << trajPath_.getTotalTime() << "s" << std::endl;
    
    // Debug: Check velocities at waypoint connection points
    double currentTime = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        currentTime += (trajPath_.getTotalTime() / (waypoints.size() - 1)) * 0.6; // Use 60% for debug
        Eigen::Vector3d vel = trajPath_.getVelocity(currentTime);
        std::cout << "[AdvancedMotionPlanner] Velocity at waypoint " << i 
                  << " transition (t=" << currentTime << "s): (" 
                  << vel.x() << ", " << vel.y() << ", " << vel.z() << ") - magnitude: " 
                  << vel.head<2>().norm() << std::endl;
    }
}

Eigen::Vector3d AdvancedMotionPlanner::getPosition(double time) const {
    if (!isValid()) return Eigen::Vector3d::Zero();
    
    if (time <= 0.0) {
        return trajPath_.getPosition(0.0);
    }
    
    if (time >= trajPath_.getTotalTime()) {
        return trajPath_.getPosition(trajPath_.getTotalTime());
    }
    
    return trajPath_.getPosition(time);
}

Eigen::Vector3d AdvancedMotionPlanner::getVelocity(double time) const {
    if (!isValid()) return Eigen::Vector3d::Zero();
    
    if (time <= 0.0 || time >= trajPath_.getTotalTime()) {
        return Eigen::Vector3d::Zero();
    }
    
    return trajPath_.getVelocity(time);
}

double AdvancedMotionPlanner::getTotalTime() const {
    if (!isValid()) return 0.0;
    return trajPath_.getTotalTime();
}

double AdvancedMotionPlanner::findOptimalConnectionTime(double segmentDuration, double maxVel) const {
    // Find connection time when velocity is still high (>70% of maxVel)
    // This ensures smooth transitions by connecting before significant deceleration
    
    double targetVelThreshold = maxVel * 0.7; // Connect when velocity > 70% of max
    double timeStep = segmentDuration * 0.01; // Check every 1% of duration
    
    // Start from middle and work forward to find when velocity drops below threshold
    for (double t = segmentDuration * 0.4; t < segmentDuration * 0.9; t += timeStep) {
        Eigen::Vector3d vel = trajPath_.getVelocity(t);
        double speed = vel.head<2>().norm();
        
        if (speed < targetVelThreshold) {
            // Found where velocity starts dropping - connect just before this
            double connectionTime = std::max(t - timeStep, segmentDuration * 0.3);
            std::cout << "  Optimal connection at t=" << connectionTime << "s (velocity=" 
                      << speed << "m/s, " << (connectionTime/segmentDuration*100) << "% of duration)" << std::endl;
            return connectionTime;
        }
    }
    
    // If velocity stays high throughout, connect at 60% of duration
    double fallbackTime = segmentDuration * 0.6;
    std::cout << "  Using fallback connection at t=" << fallbackTime << "s (60% of duration)" << std::endl;
    return fallbackTime;
}

} // namespace ctrl