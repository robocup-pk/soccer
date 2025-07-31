#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include "RobotManager.h"
#include "Utils.h"

// Demo program to test Hermite Spline trajectory planning with RRT* waypoints
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Hermite Spline Trajectory Planning Demo" << std::endl;
    std::cout << "For RRT* Waypoint Following (SSL 2010-2015 approach)" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Create robot manager
    rob::RobotManager robot_manager;
    
    // Wait for initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Initialize robot at origin
    Eigen::Vector3d initial_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(initial_pose);
    robot_manager.InitializeHome(initial_pose);
    
    std::cout << "\nRobot initialized at: " << initial_pose.transpose() << std::endl;
    
    // Simulate RRT* waypoints (typical output from RRT* algorithm)
    // These waypoints might not be smoothly connected, which is why we need
    // the Hermite Spline trajectory planner
    std::vector<std::vector<Eigen::Vector3d>> test_paths;
    
    // Test 1: Simple straight line with waypoints
    test_paths.push_back({
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.5, 0.0, 0.0),
        Eigen::Vector3d(2.0, 0.0, 0.0)
    });
    
    // Test 2: L-shaped path (typical RRT* output around obstacle)
    test_paths.push_back({
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.5, M_PI/2),
        Eigen::Vector3d(1.0, 1.0, M_PI/2)
    });
    
    // Test 3: Complex path with multiple turns (typical RRT* in cluttered environment)
    test_paths.push_back({
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.3, 0.1, 0.2),
        Eigen::Vector3d(0.6, 0.3, 0.5),
        Eigen::Vector3d(0.8, 0.6, 0.8),
        Eigen::Vector3d(1.0, 1.0, M_PI/4),
        Eigen::Vector3d(1.5, 1.2, 0.0),
        Eigen::Vector3d(2.0, 1.0, -M_PI/4)
    });
    
    // Test 4: Sharp turns (challenging for trajectory planners)
    test_paths.push_back({
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, M_PI/2),
        Eigen::Vector3d(0.0, 1.0, M_PI),
        Eigen::Vector3d(0.0, 0.0, -M_PI/2)
    });
    
    // Run tests
    for (size_t test_idx = 0; test_idx < test_paths.size(); ++test_idx) {
        std::cout << "\n========================================" << std::endl;
        std::cout << "Test " << (test_idx + 1) << ": ";
        
        switch(test_idx) {
            case 0: std::cout << "Simple straight line" << std::endl; break;
            case 1: std::cout << "L-shaped path" << std::endl; break;
            case 2: std::cout << "Complex path with multiple turns" << std::endl; break;
            case 3: std::cout << "Sharp turns (square path)" << std::endl; break;
        }
        
        std::cout << "Waypoints: " << test_paths[test_idx].size() << std::endl;
        
        // Set Hermite Spline trajectory
        robot_manager.SetHermiteSplinePath(test_paths[test_idx]);
        
        // Monitor trajectory execution
        double start_time = util::GetCurrentTime();
        double max_duration = 30.0; // Maximum 30 seconds per test
        bool trajectory_complete = false;
        
        while (!trajectory_complete && (util::GetCurrentTime() - start_time) < max_duration) {
            // Get current robot state
            Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d current_velocity = robot_manager.GetVelocityInWorldFrame();
            
            // Check if reached final waypoint
            Eigen::Vector3d final_waypoint = test_paths[test_idx].back();
            double position_error = (current_pose.head<2>() - final_waypoint.head<2>()).norm();
            
            if (position_error < 0.05) { // 5cm tolerance
                trajectory_complete = true;
                std::cout << "Trajectory completed successfully!" << std::endl;
            }
            
            // Print status every second
            static double last_print_time = 0;
            if (util::GetCurrentTime() - last_print_time > 1.0) {
                std::cout << "  Pose: " << current_pose.transpose() 
                          << " | Vel: " << current_velocity.transpose()
                          << " | Error: " << position_error << "m" << std::endl;
                last_print_time = util::GetCurrentTime();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        if (!trajectory_complete) {
            std::cout << "Trajectory timed out!" << std::endl;
        }
        
        // Wait between tests
        std::cout << "Waiting for next test..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Reset robot to origin
        robot_manager.GoHome();
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Demo completed!" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}