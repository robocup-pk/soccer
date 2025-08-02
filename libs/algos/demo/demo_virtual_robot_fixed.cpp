#include <iostream>
#include <random>
#include <chrono>
#include <thread>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <memory>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

// Virtual Robot Environment - simulates real robot sensor conditions
class VirtualRobotEnvironment {
public:
    VirtualRobotEnvironment() 
        : noise_generator_(std::chrono::steady_clock::now().time_since_epoch().count()),
          camera_noise_(0.0, 0.005),  // 5mm standard deviation
          gyro_noise_(0.0, 0.1),       // 0.1 rad/s noise
          encoder_noise_(0.0, 2.0),     // 2 RPM noise
          wheel_slip_factor_(0.95),     // 5% slip
          gyro_bias_(0.05),            // Small gyro bias
          camera_delay_ms_(50),         // 50ms camera delay
          last_camera_update_(0) {
    }

    // Simulate carpet friction effects on velocity
    Eigen::Vector3d ApplyFrictionEffects(const Eigen::Vector3d& commanded_velocity) {
        Eigen::Vector3d actual_velocity = commanded_velocity;
        
        // Carpet friction reduces actual velocity (this is what causes 0.75m instead of 1m)
        actual_velocity[0] *= 0.75;  // Linear X
        actual_velocity[1] *= 0.75;  // Linear Y  
        actual_velocity[2] *= 0.65;  // Angular (more friction on rotation)
        
        return actual_velocity;
    }

    // Add sensor noise
    Eigen::Vector3d AddCameraNoise(const Eigen::Vector3d& true_pose) {
        Eigen::Vector3d noisy_pose = true_pose;
        noisy_pose[0] += camera_noise_(noise_generator_);
        noisy_pose[1] += camera_noise_(noise_generator_);
        noisy_pose[2] += camera_noise_(noise_generator_) * 0.1; // Less noise on angle
        return noisy_pose;
    }

private:
    std::mt19937 noise_generator_;
    std::normal_distribution<double> camera_noise_;
    std::normal_distribution<double> gyro_noise_;
    std::normal_distribution<double> encoder_noise_;
    
    double wheel_slip_factor_;
    double gyro_bias_;
    double camera_delay_ms_;
    double last_camera_update_;
};

// Wrapper class that safely manages robot lifetime
class ManagedRobot {
public:
    ManagedRobot() : robot_(std::make_unique<rob::RobotManager>()) {
        // Wait a bit for threads to initialize
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    ~ManagedRobot() {
        // Ensure robot is properly destroyed
        robot_.reset();
        // Wait for threads to finish
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    rob::RobotManager* operator->() { return robot_.get(); }
    rob::RobotManager& operator*() { return *robot_; }
    
private:
    std::unique_ptr<rob::RobotManager> robot_;
};

int main(int argc, char* argv[]) {
    std::cout << "[Virtual Robot Demo] Testing B-spline with real robot conditions" << std::endl;
    
    // Initialize visualization with custom soccer objects
    std::vector<state::SoccerObject> soccer_objects;
    
    // Create two robots for visualization
    // Robot 0: Ideal conditions (red)
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),  // Robot size
                           Eigen::Vector3d::Zero(),       // Initial velocity
                           Eigen::Vector3d::Zero(),       // Initial acceleration
                           10));                          // Mass
    
    // Robot 1: Real robot with friction (blue)
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(0, 0.3, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           10));
    
    // Add ball for completeness
    soccer_objects.push_back(
        state::SoccerObject("ball", Eigen::Vector3d(2, 0, 0),
                           Eigen::Vector2d(0.043*2, 0.043*2),  // Ball diameter
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           0.046f));
    
    // Create GL simulation
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Test case selection
    int test_case = (argc > 1) ? std::atoi(argv[1]) : 1;
    std::vector<Eigen::Vector3d> waypoints;
    
    std::cout << "\nTest cases:" << std::endl;
    std::cout << "1. Straight 1m forward (tests 0.75m issue)" << std::endl;
    std::cout << "2. Square path (tests corner cutting)" << std::endl;
    std::cout << "3. 180-degree rotation (tests rotation achievement)" << std::endl;
    
    switch (test_case) {
        case 1:
            std::cout << "\nRunning Test 1: 1m straight forward" << std::endl;
            std::cout << "Expected: Robot should travel 1m" << std::endl;
            std::cout << "Real robot issue: Only travels ~0.75m due to friction" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
            
        case 2:
            std::cout << "\nRunning Test 2: Square path" << std::endl;
            std::cout << "Expected: Sharp 90-degree corners" << std::endl;
            std::cout << "Real robot issue: Corners become rounded arcs" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
            
        case 3:
            std::cout << "\nRunning Test 3: 180-degree rotation" << std::endl;
            std::cout << "Expected: Full 180-degree rotation" << std::endl;
            std::cout << "Real robot issue: May not achieve full rotation due to friction" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            break;
    }
    
    // Create virtual environment
    VirtualRobotEnvironment virtual_env;
    
    // Use scope to control robot lifetime
    {
        // Create robots
        ManagedRobot ideal_robot;
        ManagedRobot real_robot;
        
        // Configure both robots
        double start_time = util::GetCurrentTime();
        
        ideal_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        ideal_robot->SetBSplinePath(waypoints, start_time);
        
        real_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        real_robot->SetBSplinePath(waypoints, start_time);
        
        std::cout << "\nRobot visualization:" << std::endl;
        std::cout << "Red robot: Ideal conditions (simulator)" << std::endl;
        std::cout << "Blue robot: Real robot conditions (friction + noise)" << std::endl;
        
        // Simulation variables
        double last_print_time = start_time;
        int stationary_count = 0;
        double timeout = 30.0;  // Increased timeout for trajectory completion
        
        // Track real robot position with physics simulation
        Eigen::Vector3d real_robot_true_pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_real_pose = Eigen::Vector3d::Zero();
        
        // Main loop
        while (stationary_count < 120 && (util::GetCurrentTime() - start_time) < timeout) {  // Wait 2 seconds after stopping
            double current_time = util::GetCurrentTime();
            double dt = 0.016; // 60Hz
            
            // Update ideal robot
            ideal_robot->ControlLogic();
            ideal_robot->SenseLogic();
            
            // Update real robot controller
            real_robot->ControlLogic();
            real_robot->SenseLogic();
            
            // Get commanded velocity from real robot
            Eigen::Vector3d commanded_vel = real_robot->GetVelocityInWorldFrame();
            
            // Apply friction to simulate real physics
            Eigen::Vector3d actual_vel = virtual_env.ApplyFrictionEffects(commanded_vel);
            
            // Update real robot's true position based on friction-affected velocity
            // Use the real robot's current estimated pose as base
            Eigen::Vector3d current_real_pose = real_robot->GetPoseInWorldFrame();
            Eigen::Vector3d pose_change = current_real_pose - last_real_pose;
            
            // Apply friction to the movement
            pose_change[0] *= 0.75;  // 75% of commanded distance
            pose_change[1] *= 0.75;
            pose_change[2] *= 0.65;  // Even more friction on rotation
            
            real_robot_true_pose += pose_change;
            last_real_pose = current_real_pose;
            
            // Normalize angle
            while (real_robot_true_pose[2] > M_PI) real_robot_true_pose[2] -= 2 * M_PI;
            while (real_robot_true_pose[2] < -M_PI) real_robot_true_pose[2] += 2 * M_PI;
            
            // Simulate noisy camera feedback to real robot
            if (stationary_count % 3 == 0) { // Camera updates at ~20Hz
                Eigen::Vector3d noisy_pose = virtual_env.AddCameraNoise(real_robot_true_pose);
                real_robot->NewCameraData(noisy_pose);
            }
            
            // Update visualization
            soccer_objects[0].position = ideal_robot->GetPoseInWorldFrame();
            soccer_objects[1].position = real_robot_true_pose;
            
            // Get current positions for analysis
            Eigen::Vector3d ideal_pose = ideal_robot->GetPoseInWorldFrame();
            Eigen::Vector3d goal_pose = waypoints.back();
            double distance_to_goal = (ideal_pose.head<2>() - goal_pose.head<2>()).norm();
            
            // Print comparison every second
            if (current_time - last_print_time > 1.0) {
                
                std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                          << current_time - start_time << "s]" << std::endl;
                std::cout << "Ideal (simulator):  " << std::fixed << std::setprecision(3) 
                          << ideal_pose.transpose() << std::endl;
                std::cout << "Real (with friction): " << real_robot_true_pose.transpose() << std::endl;
                
                double position_error = (ideal_pose.head<2>() - real_robot_true_pose.head<2>()).norm();
                std::cout << "Position error: " << position_error << "m" << std::endl;
                
                // Show progress towards goal
                Eigen::Vector3d goal = waypoints.back();
                double progress = 0.0;
                if (test_case == 1) {
                    progress = (ideal_pose[0] / goal[0]) * 100.0;
                } else if (test_case == 3) {
                    progress = (ideal_pose[2] / goal[2]) * 100.0;
                }
                std::cout << "Progress: " << progress << "%" << std::endl;
                std::cout << "Velocity: " << ideal_robot->GetVelocityInWorldFrame().norm() << " m/s" << std::endl;
                std::cout << "Distance to goal: " << distance_to_goal << "m" << std::endl;
                
                last_print_time = current_time;
            }
            
            // Check if movement stopped - but also check if we're near the goal
            double angle_diff = std::abs(ideal_pose[2] - goal_pose[2]);
            
            // Consider trajectory complete only if very close to goal AND velocity is low
            bool near_goal = (distance_to_goal < 0.05 && angle_diff < 0.1);
            bool very_slow = (ideal_robot->GetVelocityInWorldFrame().norm() < 0.001);
            
            // Only consider stopped if BOTH conditions are met, or if we've been going for a while
            if (near_goal && very_slow) {
                stationary_count++;
                if (stationary_count == 1) {
                    std::cout << "\nTrajectory completed - reached goal!" << std::endl;
                }
            } else if ((current_time - start_time) > 20.0 && very_slow) {
                // After 20 seconds, if velocity is very low, consider it done
                stationary_count++;
                if (stationary_count == 1) {
                    std::cout << "\nTrajectory stopped - velocity too low to continue" << std::endl;
                }
            } else {
                stationary_count = 0;
            }
            
            // Run visualization
            if (!gl_sim->RunSimulationStep(soccer_objects, dt)) {
                break;
            }
        }
        
        // Final results
        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        
        Eigen::Vector3d ideal_final = ideal_robot->GetPoseInWorldFrame();
        Eigen::Vector3d real_final = real_robot_true_pose;
        
        std::cout << "\nFinal positions:" << std::endl;
        std::cout << "Ideal (simulator):    " << ideal_final.transpose() << std::endl;
        std::cout << "Real (with friction): " << real_final.transpose() << std::endl;
        
        if (test_case == 1) {
            std::cout << "\nDistance traveled:" << std::endl;
            std::cout << "Expected:     1.0m" << std::endl;
            std::cout << "Ideal robot:  " << ideal_final[0] << "m (" 
                      << (ideal_final[0]/1.0)*100 << "%)" << std::endl;
            std::cout << "Real robot:   " << real_final[0] << "m (" 
                      << (real_final[0]/1.0)*100 << "%)" << std::endl;
            std::cout << "\nThis demonstrates why the real robot only travels ~" 
                      << (real_final[0]/ideal_final[0])*100 << "% of commanded distance" << std::endl;
        } else if (test_case == 3) {
            std::cout << "\nRotation achieved:" << std::endl;
            std::cout << "Expected:     180 degrees" << std::endl;
            std::cout << "Ideal robot:  " << ideal_final[2] * 180/M_PI << " degrees" << std::endl;
            std::cout << "Real robot:   " << real_final[2] * 180/M_PI << " degrees" << std::endl;
        }
        
        std::cout << "\n[Demo Complete]" << std::endl;
    }
    // Robots are destroyed here
    
    // Clean up visualization
    gl_sim.reset();
    
    std::cout << "Exiting cleanly." << std::endl;
    return 0;
}