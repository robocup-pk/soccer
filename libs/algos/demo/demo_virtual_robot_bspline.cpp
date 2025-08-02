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
#include "StateEstimator.h"
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

// Robot wrapper for simulating real-world conditions
class RealisticRobotSimulator {
public:
    RealisticRobotSimulator(VirtualRobotEnvironment* env) 
        : virtual_env_(env), true_pose_(Eigen::Vector3d::Zero()) {}
    
    void SetPath(const std::vector<Eigen::Vector3d>& path, double start_time) {
        robot_.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        robot_.SetBSplinePath(path, start_time);
    }
    
    void Update(double dt) {
        // Get commanded velocity before updating
        Eigen::Vector3d commanded_vel = robot_.GetVelocityInWorldFrame();
        
        // Update robot control
        robot_.ControlLogic();
        robot_.SenseLogic();
        
        // Apply friction to simulate real physics
        // Convert world velocity to body frame for friction
        double theta = true_pose_[2];
        Eigen::Vector3d body_vel;
        body_vel[0] = commanded_vel[0] * cos(-theta) - commanded_vel[1] * sin(-theta);
        body_vel[1] = commanded_vel[0] * sin(-theta) + commanded_vel[1] * cos(-theta);
        body_vel[2] = commanded_vel[2];
        
        // Apply friction in body frame
        Eigen::Vector3d actual_body_vel = virtual_env_->ApplyFrictionEffects(body_vel);
        
        // Convert back to world frame
        Eigen::Vector3d actual_world_vel;
        actual_world_vel[0] = actual_body_vel[0] * cos(theta) - actual_body_vel[1] * sin(theta);
        actual_world_vel[1] = actual_body_vel[0] * sin(theta) + actual_body_vel[1] * cos(theta);
        actual_world_vel[2] = actual_body_vel[2];
        
        // Update true pose based on friction-affected velocity
        true_pose_ += actual_world_vel * dt;
        
        // Normalize angle
        while (true_pose_[2] > M_PI) true_pose_[2] -= 2 * M_PI;
        while (true_pose_[2] < -M_PI) true_pose_[2] += 2 * M_PI;
    }
    
    Eigen::Vector3d GetTruePose() const { return true_pose_; }
    Eigen::Vector3d GetEstimatedPose() const { return robot_.GetPoseInWorldFrame(); }
    rob::RobotManager& GetRobot() { return robot_; }
    
private:
    rob::RobotManager robot_;
    VirtualRobotEnvironment* virtual_env_;
    Eigen::Vector3d true_pose_;
};

// Global flag for clean shutdown
std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int) {
    g_shutdown_requested = true;
}

int main(int argc, char* argv[]) {
    std::cout << "[Virtual Robot B-Spline Demo] Testing B-spline trajectories with simulated real robot conditions" << std::endl;
    
    // Install signal handler for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Create virtual environment
    VirtualRobotEnvironment virtual_env;
    
    // Create robots using unique_ptr for proper cleanup
    auto ideal_robot = std::make_unique<rob::RobotManager>();
    auto realistic_robot = std::make_unique<RealisticRobotSimulator>(&virtual_env);
    auto ros_simulated_robot = std::make_unique<rob::RobotManager>();
    
    // Test case selection
    int test_case = (argc > 1) ? std::atoi(argv[1]) : 1;
    std::vector<Eigen::Vector3d> waypoints;
    
    std::cout << "\nTest cases:" << std::endl;
    std::cout << "1. Straight 1m forward (default)" << std::endl;
    std::cout << "2. Square path with sharp corners" << std::endl;
    std::cout << "3. 180-degree rotation" << std::endl;
    std::cout << "4. Complex path with multiple turns" << std::endl;
    
    switch (test_case) {
        case 1: // Straight line - test the 0.75m issue
            std::cout << "\nRunning Test 1: 1m straight forward" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
            
        case 2: // Square - test corner cutting
            std::cout << "\nRunning Test 2: Square path (1m x 1m)" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
            
        case 3: // 180-degree rotation
            std::cout << "\nRunning Test 3: 180-degree rotation" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            break;
            
        case 4: // Complex path
            std::cout << "\nRunning Test 4: Complex path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.5, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, -0.5, -M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, -0.5, M_PI));
            break;
    }
    
    // Configure all robots with B-spline
    double start_time = util::GetCurrentTime();
    
    ideal_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    ideal_robot->SetBSplinePath(waypoints, start_time);
    
    realistic_robot->SetPath(waypoints, start_time);
    
    ros_simulated_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    ros_simulated_robot->SetBSplinePath(waypoints, start_time);
    
    // Assign robots to visualization objects
    // Robot 0 (Red): Ideal robot
    // Robot 1 (Blue): Realistic robot with friction
    // Robot 2 (Yellow): ROS-simulated robot
    
    std::cout << "\nRobot assignments:" << std::endl;
    std::cout << "Red robot: Ideal (perfect sensors, no friction)" << std::endl;
    std::cout << "Blue robot: Realistic (noisy sensors, friction)" << std::endl;
    std::cout << "Yellow robot: ROS-simulated (camera + gyro + motor data)" << std::endl;
    
    // Main simulation loop
    double last_print_time = start_time;
    bool trajectory_complete = false;
    
    while (!trajectory_complete && !g_shutdown_requested) {
        double current_time = util::GetCurrentTime();
        double dt = 0.016; // 60Hz update rate
        
        // Update ideal robot (perfect conditions)
        ideal_robot->ControlLogic();
        ideal_robot->SenseLogic();
        
        // Update realistic robot with physics simulation
        realistic_robot->Update(dt);
        
        // Get realistic robot's true pose for sensor simulation
        Eigen::Vector3d true_pose = realistic_robot->GetTruePose();
        
        // Simulate ROS node behavior with sensor data
        {
            // Camera data (delayed and noisy)
            auto camera_data = virtual_env.GetCameraData(true_pose, current_time);
            if (camera_data.has_value()) {
                ros_simulated_robot->NewCameraData(camera_data.value());
            }
            
            // Gyro data (biased and noisy)
            double true_angular_vel = realistic_robot->GetRobot().GetVelocityInWorldFrame()[2];
            double gyro_data = virtual_env.GetGyroData(true_angular_vel);
            ros_simulated_robot->NewCameraData(ros_simulated_robot->GetPoseInWorldFrame()); // Feed back estimated pose
            
            // Note: In a real implementation, we would feed gyro and motor data to state estimator
            // For now, we simulate the effect by adding noise to the pose estimate
        }
        
        ros_simulated_robot->ControlLogic();
        ros_simulated_robot->SenseLogic();
        
        // Update visualization
        soccer_objects[0].position = ideal_robot->GetPoseInWorldFrame();
        soccer_objects[1].position = realistic_robot->GetTruePose();
        soccer_objects[2].position = ros_simulated_robot->GetPoseInWorldFrame();
        
        // Print comparison every second
        if (current_time - last_print_time > 1.0) {
            Eigen::Vector3d ideal_pose = ideal_robot->GetPoseInWorldFrame();
            Eigen::Vector3d realistic_pose = realistic_robot->GetTruePose();
            Eigen::Vector3d ros_pose = ros_simulated_robot->GetPoseInWorldFrame();
            
            std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                      << current_time - start_time << "s]" << std::endl;
            std::cout << "Ideal:      " << std::fixed << std::setprecision(3) 
                      << ideal_pose.transpose() << std::endl;
            std::cout << "Realistic:  " << realistic_pose.transpose() << std::endl;
            std::cout << "ROS-sim:    " << ros_pose.transpose() << std::endl;
            
            // Calculate errors
            double ideal_to_realistic_error = (ideal_pose.head<2>() - realistic_pose.head<2>()).norm();
            double ideal_to_ros_error = (ideal_pose.head<2>() - ros_pose.head<2>()).norm();
            
            std::cout << "Ideal->Realistic error: " << ideal_to_realistic_error << "m" << std::endl;
            std::cout << "Ideal->ROS error:       " << ideal_to_ros_error << "m" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Check if trajectory is complete (check if robots have stopped moving)
        static int stationary_count = 0;
        Eigen::Vector3d ideal_vel = ideal_robot->GetVelocityInWorldFrame();
        if (ideal_vel.norm() < 0.01) {
            stationary_count++;
            if (stationary_count > 60) { // 1 second of no movement
                trajectory_complete = true;
            }
        } else {
            stationary_count = 0;
        }
        
        // Run visualization
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            break;
        }
        
        // Timeout after 5 seconds
        if (current_time - start_time > 5.0) {
            std::cout << "\n[Timeout] Stopping after 5 seconds" << std::endl;
            break;
        }
    }
    
    // Final results
    std::cout << "\n=== FINAL RESULTS ===" << std::endl;
    
    Eigen::Vector3d ideal_final = ideal_robot->GetPoseInWorldFrame();
    Eigen::Vector3d realistic_final = realistic_robot->GetTruePose();
    Eigen::Vector3d ros_final = ros_simulated_robot->GetPoseInWorldFrame();
    
    std::cout << "\nFinal positions:" << std::endl;
    std::cout << "Ideal:      " << ideal_final.transpose() << std::endl;
    std::cout << "Realistic:  " << realistic_final.transpose() << std::endl;
    std::cout << "ROS-sim:    " << ros_final.transpose() << std::endl;
    
    if (test_case == 1) {
        // For straight line test, show distance achieved
        std::cout << "\nExpected distance: 1.0m" << std::endl;
        std::cout << "Ideal achieved:     " << ideal_final[0] << "m (" 
                  << (ideal_final[0]/1.0)*100 << "%)" << std::endl;
        std::cout << "Realistic achieved: " << realistic_final[0] << "m (" 
                  << (realistic_final[0]/1.0)*100 << "%)" << std::endl;
        std::cout << "ROS-sim achieved:   " << ros_final[0] << "m (" 
                  << (ros_final[0]/1.0)*100 << "%)" << std::endl;
    } else if (test_case == 3) {
        // For rotation test, show angle achieved
        std::cout << "\nExpected rotation: 180 degrees" << std::endl;
        std::cout << "Ideal achieved:     " << ideal_final[2] * 180/M_PI << " degrees" << std::endl;
        std::cout << "Realistic achieved: " << realistic_final[2] * 180/M_PI << " degrees" << std::endl;
        std::cout << "ROS-sim achieved:   " << ros_final[2] * 180/M_PI << " degrees" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    
    // Clean shutdown - destroy robots before exiting try block
    std::cout << "\nShutting down robots..." << std::endl;
    ideal_robot.reset();
    realistic_robot.reset();
    ros_simulated_robot.reset();
    
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }
    
    // Give time for threads to finish cleanly
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "Clean exit." << std::endl;
    return 0;
}