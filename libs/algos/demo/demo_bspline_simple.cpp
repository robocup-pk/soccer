#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <memory>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

int main(int argc, char* argv[]) {
    std::cout << "[Simple B-Spline Demo] Testing B-spline with friction simulation" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Create robot managers using smart pointers for better memory management
    auto ideal_robot = std::make_unique<rob::RobotManager>();
    auto friction_robot = std::make_unique<rob::RobotManager>();
    
    // Test case selection
    int test_case = (argc > 1) ? std::atoi(argv[1]) : 1;
    std::vector<Eigen::Vector3d> waypoints;
    
    std::cout << "\nTest cases:" << std::endl;
    std::cout << "1. Straight 1m forward (default)" << std::endl;
    std::cout << "2. 90-degree turn" << std::endl;
    std::cout << "3. 180-degree rotation" << std::endl;
    
    switch (test_case) {
        case 1:
            std::cout << "\nRunning Test 1: 1m straight forward" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
            
        case 2:
            std::cout << "\nRunning Test 2: 90-degree turn" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            break;
            
        case 3:
            std::cout << "\nRunning Test 3: 180-degree rotation" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            break;
    }
    
    // Configure both robots with B-spline
    double start_time = util::GetCurrentTime();
    
    ideal_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    ideal_robot->SetBSplinePath(waypoints, start_time);
    
    friction_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    friction_robot->SetBSplinePath(waypoints, start_time);
    
    std::cout << "\nRobot assignments:" << std::endl;
    std::cout << "Red robot: Ideal (no friction)" << std::endl;
    std::cout << "Blue robot: With 75% friction (simulating carpet)" << std::endl;
    
    // Main simulation loop
    double last_print_time = start_time;
    int stationary_count = 0;
    const double friction_factor = 0.75; // Simulates carpet friction
    
    while (stationary_count < 60) { // Stop after 1 second of no movement
        double current_time = util::GetCurrentTime();
        double dt = 0.016; // 60Hz update rate
        
        // Update ideal robot
        ideal_robot->ControlLogic();
        ideal_robot->SenseLogic();
        
        // Update friction robot with simulated physics
        // Get commanded velocity before update
        Eigen::Vector3d commanded_vel = friction_robot->GetVelocityInWorldFrame();
        
        // Apply friction to commanded velocity
        Eigen::Vector3d actual_vel = commanded_vel * friction_factor;
        
        // Update robot's internal state
        friction_robot->ControlLogic();
        friction_robot->SenseLogic();
        
        // Simulate the effect of friction by modifying the pose update
        // This is a simplified simulation - in reality, friction affects acceleration
        Eigen::Vector3d current_pose = friction_robot->GetPoseInWorldFrame();
        Eigen::Vector3d pose_delta = actual_vel * dt;
        
        // For demonstration, we'll show the difference visually
        // The friction robot will appear to lag behind
        
        // Update visualization
        soccer_objects[0].position = ideal_robot->GetPoseInWorldFrame();
        soccer_objects[1].position = friction_robot->GetPoseInWorldFrame();
        
        // Simulate friction effect on position (visual only)
        static Eigen::Vector3d friction_accumulated_error = Eigen::Vector3d::Zero();
        friction_accumulated_error += (commanded_vel - actual_vel) * dt * 0.5;
        soccer_objects[1].position -= friction_accumulated_error;
        
        // Print comparison every second
        if (current_time - last_print_time > 1.0) {
            Eigen::Vector3d ideal_pose = ideal_robot->GetPoseInWorldFrame();
            Eigen::Vector3d friction_pose = soccer_objects[1].position; // Use visual position
            
            std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                      << current_time - start_time << "s]" << std::endl;
            std::cout << "Ideal:     " << std::fixed << std::setprecision(3) 
                      << ideal_pose.transpose() << std::endl;
            std::cout << "Friction:  " << friction_pose.transpose() << std::endl;
            
            double position_error = (ideal_pose.head<2>() - friction_pose.head<2>()).norm();
            std::cout << "Position error: " << position_error << "m" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Check if trajectory is complete
        Eigen::Vector3d ideal_vel = ideal_robot->GetVelocityInWorldFrame();
        if (ideal_vel.norm() < 0.01) {
            stationary_count++;
        } else {
            stationary_count = 0;
        }
        
        // Run visualization
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            break;
        }
        
        // Timeout after 20 seconds
        if (current_time - start_time > 20.0) {
            std::cout << "\n[Timeout] Stopping after 20 seconds" << std::endl;
            break;
        }
    }
    
    // Final results
    std::cout << "\n=== FINAL RESULTS ===" << std::endl;
    
    Eigen::Vector3d ideal_final = ideal_robot->GetPoseInWorldFrame();
    Eigen::Vector3d friction_final = soccer_objects[1].position;
    
    std::cout << "\nFinal positions:" << std::endl;
    std::cout << "Ideal:     " << ideal_final.transpose() << std::endl;
    std::cout << "Friction:  " << friction_final.transpose() << std::endl;
    
    if (test_case == 1) {
        std::cout << "\nExpected distance: 1.0m" << std::endl;
        std::cout << "Ideal achieved:    " << ideal_final[0] << "m (" 
                  << (ideal_final[0]/1.0)*100 << "%)" << std::endl;
        std::cout << "Friction achieved: " << friction_final[0] << "m (" 
                  << (friction_final[0]/1.0)*100 << "%)" << std::endl;
        std::cout << "\nFriction robot travels approximately " 
                  << (friction_final[0]/ideal_final[0])*100 << "% of ideal distance" << std::endl;
    } else if (test_case == 3) {
        std::cout << "\nExpected rotation: 180 degrees" << std::endl;
        std::cout << "Ideal achieved:    " << ideal_final[2] * 180/M_PI << " degrees" << std::endl;
        std::cout << "Friction achieved: " << friction_final[2] * 180/M_PI << " degrees" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    
    // Proper cleanup - robots will be destroyed automatically
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    return 0;
}