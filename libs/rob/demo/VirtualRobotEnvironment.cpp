#include <iostream>
#include <fstream>
#include <sstream>
#include <random>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "StateEstimator.h"
#include "HardwareManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

// Simulates real robot sensor noise and delays
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

    // Simulate noisy camera data with delay
    std::optional<Eigen::Vector3d> GetCameraData(const Eigen::Vector3d& true_pose, double current_time) {
        // Camera updates at 20Hz
        if (current_time - last_camera_update_ < 0.05) {
            return std::nullopt;
        }
        
        last_camera_update_ = current_time;
        
        // Add delay buffer
        camera_buffer_.push_back({current_time, true_pose});
        
        // Return delayed measurement
        double delayed_time = current_time - camera_delay_ms_ / 1000.0;
        for (auto it = camera_buffer_.begin(); it != camera_buffer_.end(); ) {
            if (it->timestamp < delayed_time - 0.1) {
                it = camera_buffer_.erase(it);
            } else if (it->timestamp <= delayed_time) {
                Eigen::Vector3d noisy_pose = it->pose;
                noisy_pose[0] += camera_noise_(noise_generator_);
                noisy_pose[1] += camera_noise_(noise_generator_);
                noisy_pose[2] += camera_noise_(noise_generator_) * 0.1; // Less noise on angle
                return noisy_pose;
            } else {
                ++it;
            }
        }
        
        return std::nullopt;
    }
    
    // Simulate noisy gyro data
    double GetGyroData(double true_angular_velocity) {
        // Add bias and noise
        return true_angular_velocity + gyro_bias_ + gyro_noise_(noise_generator_);
    }
    
    // Simulate noisy encoder data with slip
    Eigen::Vector4d GetEncoderData(const Eigen::Vector4d& commanded_rpm) {
        Eigen::Vector4d actual_rpm;
        for (int i = 0; i < 4; ++i) {
            // Apply slip factor (more slip at higher speeds)
            double slip = wheel_slip_factor_;
            if (std::abs(commanded_rpm[i]) > 100) {
                slip = wheel_slip_factor_ - 0.05; // More slip at high speed
            }
            
            actual_rpm[i] = commanded_rpm[i] * slip + encoder_noise_(noise_generator_);
        }
        return actual_rpm;
    }
    
    // Simulate carpet friction effects
    Eigen::Vector3d ApplyFrictionEffects(const Eigen::Vector3d& commanded_velocity) {
        Eigen::Vector3d actual_velocity = commanded_velocity;
        
        // Carpet friction reduces actual velocity
        actual_velocity[0] *= 0.75;  // Linear X
        actual_velocity[1] *= 0.75;  // Linear Y  
        actual_velocity[2] *= 0.65;  // Angular (more friction on rotation)
        
        return actual_velocity;
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
    
    struct DelayedPose {
        double timestamp;
        Eigen::Vector3d pose;
    };
    std::vector<DelayedPose> camera_buffer_;
};

int main(int argc, char* argv[]) {
    std::cout << "[VirtualRobotEnvironment] Starting virtual robot test environment" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Create two robots - one ideal, one with realistic sensors
    rob::RobotManager ideal_robot;
    rob::RobotManager realistic_robot;
    
    // Virtual environment for sensor simulation
    VirtualRobotEnvironment virtual_env;
    
    // Test trajectory
    std::vector<Eigen::Vector3d> waypoints;
    int test_case = (argc > 1) ? std::atoi(argv[1]) : 1;
    
    switch (test_case) {
        case 1: // Straight line
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
        case 2: // Square
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        case 3: // 180-degree rotation
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            break;
    }
    
    // Configure robots
    ideal_robot.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    ideal_robot.SetBSplinePath(waypoints, util::GetCurrentTime());
    
    realistic_robot.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    realistic_robot.SetBSplinePath(waypoints, util::GetCurrentTime());
    
    // Disable smoothing for realistic robot
    // realistic_robot.GetBSplineManager()->SetSmoothingEnabled(false);
    
    // Main loop
    double start_time = util::GetCurrentTime();
    double last_print_time = start_time;
    
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        // Update ideal robot (perfect sensors)
        ideal_robot.ControlLogic();
        ideal_robot.SenseLogic();
        
        // Simulate realistic robot with noisy sensors
        {
            // Get true state from physics
            Eigen::Vector3d true_pose = realistic_robot.GetPoseInWorldFrame();
            Eigen::Vector3d commanded_velocity = realistic_robot.GetVelocityInWorldFrame();
            
            // Simulate sensor data
            auto camera_data = virtual_env.GetCameraData(true_pose, current_time);
            double gyro_data = virtual_env.GetGyroData(commanded_velocity[2]);
            
            // Get commanded wheel speeds from hardware manager
            // This would come from realistic_robot.hardware_manager in real implementation
            Eigen::Vector4d commanded_rpm = Eigen::Vector4d::Zero(); // Placeholder
            Eigen::Vector4d encoder_data = virtual_env.GetEncoderData(commanded_rpm);
            
            // Feed sensor data to state estimator
            if (camera_data.has_value()) {
                realistic_robot.NewCameraData(camera_data.value());
            }
            realistic_robot.NewGyroData(gyro_data);
            realistic_robot.NewMotorsData(encoder_data);
            
            // Apply friction effects to actual movement
            Eigen::Vector3d actual_velocity = virtual_env.ApplyFrictionEffects(commanded_velocity);
            // In real robot, this would be handled by physics
            // Here we simulate it by modifying the robot's actual movement
        }
        
        realistic_robot.ControlLogic();
        realistic_robot.SenseLogic();
        
        // Update visualization
        soccer_objects[0].position = ideal_robot.GetPoseInWorldFrame();
        soccer_objects[1].position = realistic_robot.GetPoseInWorldFrame();
        
        // Print comparison every second
        if (current_time - last_print_time > 1.0) {
            Eigen::Vector3d ideal_pose = ideal_robot.GetPoseInWorldFrame();
            Eigen::Vector3d realistic_pose = realistic_robot.GetPoseInWorldFrame();
            Eigen::Vector3d error = ideal_pose - realistic_pose;
            
            std::cout << "\n[Time: " << current_time - start_time << "s]" << std::endl;
            std::cout << "Ideal Robot:     " << ideal_pose.transpose() << std::endl;
            std::cout << "Realistic Robot: " << realistic_pose.transpose() << std::endl;
            std::cout << "Error:           " << error.transpose() << std::endl;
            std::cout << "Error magnitude: " << error.head<2>().norm() << "m" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Run simulation
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            break;
        }
        
        // Stop after 30 seconds
        if (current_time - start_time > 30.0) {
            std::cout << "\n[VirtualRobotEnvironment] Test completed" << std::endl;
            break;
        }
    }
    
    return 0;
}