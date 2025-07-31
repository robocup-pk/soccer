#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include "M_TrajectoryPlanner.h"
#include "M_TrajectorySegment.h"

/**
 * Simple demonstration of the difference between:
 * 1. Original approach: Only using first and last waypoints
 * 2. New approach: Using ALL waypoints from RRTX
 */

int main() {
    std::cout << "\n=== Waypoint Trajectory Comparison Demo ===\n" << std::endl;
    
    // Simulated RRTX waypoints (robot navigating around obstacles)
    std::vector<Eigen::Vector3d> rrtx_waypoints = {
        Eigen::Vector3d(0.0, 0.0, 0.0),      // Start
        Eigen::Vector3d(0.5, 0.2, 0.4),      // Waypoint 1: slight right turn
        Eigen::Vector3d(1.0, 0.5, 0.8),      // Waypoint 2: continue right
        Eigen::Vector3d(1.2, 1.0, 1.57),     // Waypoint 3: turn north
        Eigen::Vector3d(1.0, 1.5, 2.36),     // Waypoint 4: turn northwest  
        Eigen::Vector3d(0.5, 1.8, 3.14),     // Waypoint 5: face west
        Eigen::Vector3d(0.0, 2.0, 3.14)      // Goal: final destination
    };
    
    std::cout << "RRTX generated " << rrtx_waypoints.size() << " waypoints:" << std::endl;
    for (size_t i = 0; i < rrtx_waypoints.size(); ++i) {
        std::cout << "  Waypoint " << i << ": (" 
                  << rrtx_waypoints[i].x() << ", " 
                  << rrtx_waypoints[i].y() << ", " 
                  << rrtx_waypoints[i].z() << " rad)" << std::endl;
    }
    
    // Setup constraints
    ctrl::M_MoveConstraints constraints;
    constraints.vel_max = 2.0;
    constraints.acc_max = 3.0;
    constraints.vel_max_w = 5.0;
    constraints.acc_max_w = 10.0;
    
    // Initial robot state
    ctrl::M_RobotState robot_state;
    robot_state.position = rrtx_waypoints[0];
    robot_state.velocity = Eigen::Vector3d::Zero();
    robot_state.constraints = constraints;
    
    std::cout << "\n--- ORIGINAL APPROACH (First & Last Only) ---" << std::endl;
    {
        // Original approach: only use first and last waypoints
        std::vector<Eigen::Vector3d> simple_path = {
            rrtx_waypoints.front(),  // Start
            rrtx_waypoints.back()    // End
        };
        
        auto direct_traj = ctrl::M_TrajectoryPlanner::generatePositionTrajectory(
            robot_state, simple_path.back()
        );
        
        std::cout << "Direct trajectory (ignoring intermediate waypoints):" << std::endl;
        std::cout << "  Total time: " << direct_traj->getTotalTime() << "s" << std::endl;
        std::cout << "  Path: " << simple_path[0].transpose() << " -> " << simple_path[1].transpose() << std::endl;
        
        // Sample some points along the trajectory
        std::cout << "  Sample positions:" << std::endl;
        for (double t = 0; t <= direct_traj->getTotalTime(); t += direct_traj->getTotalTime() / 5) {
            Eigen::Vector3d pos = direct_traj->getPosition(t);
            std::cout << "    t=" << t << "s: (" << pos.x() << ", " << pos.y() << ")" << std::endl;
        }
    }
    
    std::cout << "\n--- NEW APPROACH (All Waypoints) ---" << std::endl;
    {
        // New approach: use ALL waypoints
        auto multi_traj = ctrl::M_MultiWaypointPlanner::generateMultiWaypointTrajectory(
            robot_state, rrtx_waypoints, constraints
        );
        
        std::cout << "Multi-waypoint trajectory (using all RRTX waypoints):" << std::endl;
        std::cout << "  Total time: " << multi_traj->getTotalTime() << "s" << std::endl;
        multi_traj->print();
        
        // Check which waypoints the trajectory passes through
        std::cout << "\n  Waypoint verification:" << std::endl;
        for (size_t i = 0; i < rrtx_waypoints.size(); ++i) {
            bool passes = multi_traj->passesThrough(rrtx_waypoints[i], 0.05);
            std::cout << "    Waypoint " << i << ": " << (passes ? "PASSES" : "MISSES") << std::endl;
        }
        
        // Sample positions to show the path follows waypoints
        std::cout << "\n  Sample positions along trajectory:" << std::endl;
        int samples = 20;
        for (int i = 0; i <= samples; ++i) {
            double t = (multi_traj->getTotalTime() * i) / samples;
            Eigen::Vector3d pos = multi_traj->getPosition(t);
            Eigen::Vector3d next_dest = multi_traj->getNextDestination(t);
            std::cout << "    t=" << std::fixed << std::setprecision(2) << t 
                      << "s: pos=(" << pos.x() << ", " << pos.y() << ")"
                      << " next_target=(" << next_dest.x() << ", " << next_dest.y() << ")" << std::endl;
        }
    }
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "The original approach creates a direct path from start to end, potentially:" << std::endl;
    std::cout << "- Colliding with obstacles that RRTX was trying to avoid" << std::endl;
    std::cout << "- Missing important waypoints for navigation" << std::endl;
    std::cout << "\nThe new approach follows ALL waypoints from RRTX, ensuring:" << std::endl;
    std::cout << "- The path avoids obstacles as RRTX intended" << std::endl;
    std::cout << "- Smooth transitions between waypoints" << std::endl;
    std::cout << "- Runtime replanning capability if needed" << std::endl;
    
    return 0;
}