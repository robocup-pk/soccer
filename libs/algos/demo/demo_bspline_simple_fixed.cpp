#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

int main(int argc, char* argv[]) {
    std::cout << "[Simple B-Spline Demo] Testing basic B-spline trajectory execution" << std::endl;
    
    // Initialize visualization with a single robot
    std::vector<state::SoccerObject> soccer_objects;
    
    // Create one robot for visualization
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),  // Robot size
                           Eigen::Vector3d::Zero(),       // Initial velocity
                           Eigen::Vector3d::Zero(),       // Initial acceleration
                           10));                          // Mass
    
    // Add ball for reference
    soccer_objects.push_back(
        state::SoccerObject("ball", Eigen::Vector3d(1, 0, 0),
                           Eigen::Vector2d(0.043*2, 0.043*2),  // Ball diameter
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           0.046f));
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Test waypoints - simple forward motion
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    
    // Create robot manager
    auto robot = std::make_unique<rob::RobotManager>();
    
    // Set to B-spline mode and give it the path
    robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    double start_time = util::GetCurrentTime();
    robot->SetBSplinePath(waypoints, start_time);
    
    std::cout << "\nStarting B-spline trajectory execution..." << std::endl;
    std::cout << "Robot should move from (0,0) to (1,0)" << std::endl;
    
    // Initialize the robot pose
    Eigen::Vector3d initial_pose = Eigen::Vector3d::Zero();
    robot->InitializePose(initial_pose);
    
    double last_print_time = start_time;
    int frame = 0;
    const int max_frames = 600;  // 10 seconds at 60Hz
    
    // Main loop
    while (frame < max_frames && gl_sim) {
        double current_time = util::GetCurrentTime();
        double dt = 0.016;  // 60Hz
        
        // Update robot control
        robot->ControlLogic();
        robot->SenseLogic();
        
        // Get current pose and velocity
        Eigen::Vector3d current_pose = robot->GetPoseInWorldFrame();
        Eigen::Vector3d current_velocity = robot->GetVelocityInWorldFrame();
        
        // Update visualization
        soccer_objects[0].position = current_pose;
        
        // Print status every second
        if (current_time - last_print_time > 1.0) {
            std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                      << current_time - start_time << "s]" << std::endl;
            std::cout << "Position: " << std::fixed << std::setprecision(3) 
                      << current_pose.transpose() << std::endl;
            std::cout << "Velocity: " << current_velocity.transpose() << " m/s" << std::endl;
            std::cout << "Velocity magnitude: " << current_velocity.norm() << " m/s" << std::endl;
            
            // Check progress
            double progress = current_pose[0] * 100.0;  // percentage to goal
            std::cout << "Progress to goal: " << progress << "%" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Check if reached goal
        if ((current_pose.head<2>() - Eigen::Vector2d(1.0, 0.0)).norm() < 0.05 && 
            current_velocity.norm() < 0.01) {
            std::cout << "\n=== GOAL REACHED ===" << std::endl;
            std::cout << "Final position: " << current_pose.transpose() << std::endl;
            std::cout << "Time taken: " << current_time - start_time << " seconds" << std::endl;
            break;
        }
        
        // Run visualization
        if (!gl_sim->RunSimulationStep(soccer_objects, dt)) {
            break;
        }
        
        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    if (frame >= max_frames) {
        std::cout << "\n=== TIMEOUT ===" << std::endl;
        Eigen::Vector3d final_pose = robot->GetPoseInWorldFrame();
        std::cout << "Final position after timeout: " << final_pose.transpose() << std::endl;
        std::cout << "Distance traveled: " << final_pose[0] << "m" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    return 0;
}