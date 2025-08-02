#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

// Simple friction simulator that modifies robot position directly
void simulateFriction(state::SoccerObject& robot, double friction_factor, double dt) {
    // Simple model: reduce velocity by friction factor
    // In real world, friction affects acceleration, but for demo purposes
    // we'll just scale the movement
    static Eigen::Vector3d last_position = robot.position;
    Eigen::Vector3d velocity = (robot.position - last_position) / dt;
    
    // Apply friction
    velocity *= friction_factor;
    
    // Update position based on friction-affected velocity
    robot.position = last_position + velocity * dt;
    last_position = robot.position;
}

int main(int argc, char* argv[]) {
    std::cout << "[B-Spline Friction Demo] Demonstrating B-spline with simulated friction" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Test case selection
    int test_case = (argc > 1) ? std::atoi(argv[1]) : 1;
    std::vector<Eigen::Vector3d> waypoints;
    
    std::cout << "\nTest cases:" << std::endl;
    std::cout << "1. Straight 1m forward (default)" << std::endl;
    std::cout << "2. Square path" << std::endl;
    std::cout << "3. 180-degree rotation" << std::endl;
    
    switch (test_case) {
        case 1:
            std::cout << "\nRunning Test 1: 1m straight forward" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
            
        case 2:
            std::cout << "\nRunning Test 2: Square path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
            
        case 3:
            std::cout << "\nRunning Test 3: 180-degree rotation" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            break;
    }
    
    // Create a single robot manager in a scope
    {
        rob::RobotManager robot;
        
        // Configure with B-spline
        double start_time = util::GetCurrentTime();
        robot.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        robot.SetBSplinePath(waypoints, start_time);
        
        std::cout << "\nRunning B-spline trajectory..." << std::endl;
        std::cout << "Red robot: Ideal B-spline trajectory" << std::endl;
        std::cout << "Blue robot: Same trajectory with 75% friction (simulating carpet)" << std::endl;
        
        // Track positions for friction simulation
        Eigen::Vector3d friction_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_ideal_position = Eigen::Vector3d::Zero();
        
        // Main simulation loop
        double last_print_time = start_time;
        int stationary_count = 0;
        const double friction_factor = 0.75;
        const double timeout = 10.0; // 10 second timeout
        
        while (stationary_count < 60 && (util::GetCurrentTime() - start_time) < timeout) {
            double current_time = util::GetCurrentTime();
            double dt = 0.016; // 60Hz
            
            // Update robot
            robot.ControlLogic();
            robot.SenseLogic();
            
            // Get current position
            Eigen::Vector3d ideal_position = robot.GetPoseInWorldFrame();
            
            // Calculate velocity and apply friction for second robot
            Eigen::Vector3d ideal_velocity = (ideal_position - last_ideal_position) / dt;
            Eigen::Vector3d friction_velocity = ideal_velocity * friction_factor;
            friction_position += friction_velocity * dt;
            
            // Update visualization
            soccer_objects[0].position = ideal_position;
            soccer_objects[1].position = friction_position;
            
            // Print status every second
            if (current_time - last_print_time > 1.0) {
                std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                          << current_time - start_time << "s]" << std::endl;
                std::cout << "Ideal:    " << std::fixed << std::setprecision(3) 
                          << ideal_position.transpose() << std::endl;
                std::cout << "Friction: " << friction_position.transpose() << std::endl;
                
                double error = (ideal_position.head<2>() - friction_position.head<2>()).norm();
                std::cout << "Position error: " << error << "m" << std::endl;
                
                last_print_time = current_time;
            }
            
            // Check if robot has stopped
            if (robot.GetVelocityInWorldFrame().norm() < 0.01) {
                stationary_count++;
            } else {
                stationary_count = 0;
            }
            
            // Update for next iteration
            last_ideal_position = ideal_position;
            
            // Run visualization
            if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
                break;
            }
            
            // Small delay to maintain 60Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        
        // Final results
        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        std::cout << "Ideal final position:    " << robot.GetPoseInWorldFrame().transpose() << std::endl;
        std::cout << "Friction final position: " << friction_position.transpose() << std::endl;
        
        if (test_case == 1) {
            double ideal_distance = robot.GetPoseInWorldFrame()[0];
            double friction_distance = friction_position[0];
            std::cout << "\nExpected: 1.0m forward" << std::endl;
            std::cout << "Ideal achieved:    " << ideal_distance << "m (" 
                      << (ideal_distance/1.0)*100 << "%)" << std::endl;
            std::cout << "Friction achieved: " << friction_distance << "m (" 
                      << (friction_distance/1.0)*100 << "%)" << std::endl;
            std::cout << "\nFriction robot travels " << (friction_distance/ideal_distance)*100 
                      << "% of ideal distance" << std::endl;
        }
        
        std::cout << "\n[Demo Complete]" << std::endl;
    }
    
    // Robot is destroyed here, before gl_simulation
    
    // Give time for cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Exiting cleanly." << std::endl;
    return 0;
}