#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <chrono>
#include <thread>
#include <iomanip>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

// Parse log file and replay sensor data
class LogDataParser {
public:
    struct SensorReading {
        double timestamp;
        std::optional<Eigen::Vector4d> wheel_rpms;
        std::optional<double> gyro_data;
        std::optional<Eigen::Vector3d> camera_pose;
        std::optional<Eigen::Vector3d> commanded_velocity;
    };

    bool LoadLogFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
            return false;
        }

        std::string line;
        double current_time = 0.0;
        
        while (std::getline(file, line)) {
            SensorReading reading;
            reading.timestamp = current_time;
            
            // Parse different log entries
            if (line.find("RPMS:") != std::string::npos) {
                // Extract wheel RPMs
                std::regex rpm_regex("RPMS: ([0-9.-]+) ([0-9.-]+) ([0-9.-]+) ([0-9.-]+)");
                std::smatch match;
                if (std::regex_search(line, match, rpm_regex)) {
                    reading.wheel_rpms = Eigen::Vector4d(
                        std::stod(match[1]), std::stod(match[2]),
                        std::stod(match[3]), std::stod(match[4])
                    );
                }
            }
            else if (line.find("Gyro Data:") != std::string::npos) {
                // Extract gyro data
                std::regex gyro_regex("Gyro Data: ([0-9.-]+)");
                std::smatch match;
                if (std::regex_search(line, match, gyro_regex)) {
                    reading.gyro_data = std::stod(match[1]);
                }
            }
            else if (line.find("[RobotNode::PublishOdometry] Pose:") != std::string::npos) {
                // Extract pose from odometry
                std::regex pose_regex("Pose: ([0-9.-]+) ([0-9.-]+) ([0-9.-]+)");
                std::smatch match;
                if (std::regex_search(line, match, pose_regex)) {
                    reading.camera_pose = Eigen::Vector3d(
                        std::stod(match[1]), std::stod(match[2]), std::stod(match[3])
                    );
                }
            }
            else if (line.find("Body Velocity:") != std::string::npos) {
                // Extract commanded body velocity
                std::regex vel_regex("Body Velocity:\\s+([0-9.-]+)\\s+([0-9.-]+)\\s+([0-9.-]+)");
                std::smatch match;
                if (std::regex_search(line, match, vel_regex)) {
                    reading.commanded_velocity = Eigen::Vector3d(
                        std::stod(match[1]), std::stod(match[2]), std::stod(match[3])
                    );
                }
            }
            
            // Store reading if it has any data
            if (reading.wheel_rpms || reading.gyro_data || reading.camera_pose || reading.commanded_velocity) {
                readings_.push_back(reading);
                current_time += 0.016; // Approximate 60Hz
            }
        }
        
        std::cout << "Loaded " << readings_.size() << " sensor readings from log" << std::endl;
        return true;
    }

    const std::vector<SensorReading>& GetReadings() const { return readings_; }

private:
    std::vector<SensorReading> readings_;
};

// Virtual environment to simulate friction effects
class VirtualRobotEnvironment {
public:
    // Apply friction to actual movement (this is what causes 0.75m instead of 1m)
    Eigen::Vector3d ApplyFrictionEffects(const Eigen::Vector3d& commanded_velocity) {
        Eigen::Vector3d actual_velocity = commanded_velocity;
        
        // Carpet friction reduces actual velocity
        actual_velocity[0] *= 0.75;  // Linear X - this causes 0.75m travel instead of 1m
        actual_velocity[1] *= 0.75;  // Linear Y  
        actual_velocity[2] *= 0.65;  // Angular (more friction on rotation)
        
        return actual_velocity;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "[Replay Sensor Data Demo] Testing B-spline with actual robot sensor data" << std::endl;
    
    // Load log file
    std::string log_file = (argc > 1) ? argv[1] : "/home/kodek/my_project/soccer/robot0_odometry.log";
    LogDataParser parser;
    if (!parser.LoadLogFile(log_file)) {
        std::cerr << "Failed to load log file: " << log_file << std::endl;
        return 1;
    }
    
    // Initialize visualization with custom soccer objects
    std::vector<state::SoccerObject> soccer_objects;
    
    // Create three robots for visualization
    // Robot 0: Ideal B-spline (red)
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),  // Robot size
                           Eigen::Vector3d::Zero(),       // Initial velocity
                           Eigen::Vector3d::Zero(),       // Initial acceleration
                           10));                          // Mass
    
    // Robot 1: B-spline with sensor feedback (blue)
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(0, 0.3, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           10));
    
    // Robot 2: True position from sensors (yellow/green)
    soccer_objects.push_back(
        state::SoccerObject("robot2", Eigen::Vector3d(0, -0.3, 0), 
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
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Create virtual environment
    VirtualRobotEnvironment virtual_env;
    
    // Create test waypoints (from your log data, it looks like 1m forward movement)
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    
    // Create robots with scope control
    {
        // Create robot managers
        auto ideal_robot = std::make_unique<rob::RobotManager>();
        auto sensor_replay_robot = std::make_unique<rob::RobotManager>();
        
        // Configure robots with B-spline
        double start_time = util::GetCurrentTime();
        
        ideal_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        ideal_robot->SetBSplinePath(waypoints, start_time);
        
        sensor_replay_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        sensor_replay_robot->SetBSplinePath(waypoints, start_time);
        
        std::cout << "\nRobot visualization:" << std::endl;
        std::cout << "Red robot: Ideal B-spline (no sensor feedback)" << std::endl;
        std::cout << "Blue robot: B-spline with actual sensor data from log file" << std::endl;
        std::cout << "Yellow robot: True position based on sensor data with friction" << std::endl;
        
        // Get sensor readings
        const auto& readings = parser.GetReadings();
        size_t reading_index = 0;
        
        // Track positions
        Eigen::Vector3d true_position_from_sensors = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_camera_pose = Eigen::Vector3d::Zero();
        bool first_camera_reading = true;
        
        std::cout << "\nReplaying " << readings.size() << " sensor readings..." << std::endl;
        
        // Main loop
        double last_print_time = start_time;
        int frame_count = 0;
        
        while (reading_index < readings.size() && gl_sim) {
            double current_time = util::GetCurrentTime();
            double dt = 0.016; // 60Hz
            
            // Update ideal robot (no sensor feedback)
            ideal_robot->ControlLogic();
            ideal_robot->SenseLogic();
            
            // Get current sensor reading
            if (reading_index < readings.size()) {
                const auto& reading = readings[reading_index];
                
                // Feed sensor data to replay robot
                if (reading.camera_pose) {
                    sensor_replay_robot->NewCameraData(reading.camera_pose.value());
                    
                    // Track true position from camera
                    if (first_camera_reading) {
                        last_camera_pose = reading.camera_pose.value();
                        first_camera_reading = false;
                    }
                    true_position_from_sensors = reading.camera_pose.value();
                }
                
                // Note: Gyro and motor data would be fed to state estimator if it was accessible
                // For now, we use camera data which already includes the effects
                
                reading_index++;
            }
            
            // Update sensor replay robot
            sensor_replay_robot->ControlLogic();
            sensor_replay_robot->SenseLogic();
            
            // Update visualization
            soccer_objects[0].position = ideal_robot->GetPoseInWorldFrame();
            soccer_objects[1].position = sensor_replay_robot->GetPoseInWorldFrame();
            soccer_objects[2].position = true_position_from_sensors;
            
            // Print comparison every second
            if (current_time - last_print_time > 1.0) {
                Eigen::Vector3d ideal_pose = ideal_robot->GetPoseInWorldFrame();
                Eigen::Vector3d replay_pose = sensor_replay_robot->GetPoseInWorldFrame();
                
                std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                          << current_time - start_time << "s]" << std::endl;
                std::cout << "Ideal B-spline:      " << std::fixed << std::setprecision(3) 
                          << ideal_pose.transpose() << std::endl;
                std::cout << "With sensor data:    " << replay_pose.transpose() << std::endl;
                std::cout << "True sensor position: " << true_position_from_sensors.transpose() << std::endl;
                
                // Calculate errors
                double ideal_to_sensor_error = (ideal_pose.head<2>() - true_position_from_sensors.head<2>()).norm();
                std::cout << "Ideal vs Sensor error: " << ideal_to_sensor_error << "m" << std::endl;
                
                // Show commanded velocity if available
                if (reading_index > 0 && reading_index < readings.size()) {
                    const auto& reading = readings[reading_index-1];
                    if (reading.commanded_velocity) {
                        std::cout << "Commanded velocity: " << reading.commanded_velocity.value().transpose() << " m/s" << std::endl;
                    }
                }
                
                last_print_time = current_time;
            }
            
            // Run visualization
            if (!gl_sim->RunSimulationStep(soccer_objects, dt)) {
                break;
            }
            
            frame_count++;
            
            // Small delay to match real-time playback
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        
        // Final results
        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        std::cout << "\nProcessed " << frame_count << " frames" << std::endl;
        std::cout << "Total sensor readings: " << readings.size() << std::endl;
        
        Eigen::Vector3d ideal_final = ideal_robot->GetPoseInWorldFrame();
        Eigen::Vector3d replay_final = sensor_replay_robot->GetPoseInWorldFrame();
        
        std::cout << "\nFinal positions:" << std::endl;
        std::cout << "Ideal B-spline:       " << ideal_final.transpose() << std::endl;
        std::cout << "With sensor feedback: " << replay_final.transpose() << std::endl;
        std::cout << "True sensor position: " << true_position_from_sensors.transpose() << std::endl;
        
        std::cout << "\nDistance traveled:" << std::endl;
        std::cout << "Expected:            1.0m" << std::endl;
        std::cout << "Ideal B-spline:      " << ideal_final[0] << "m (" 
                  << (ideal_final[0]/1.0)*100 << "%)" << std::endl;
        std::cout << "From sensor data:    " << true_position_from_sensors[0] << "m (" 
                  << (true_position_from_sensors[0]/1.0)*100 << "%)" << std::endl;
        
        std::cout << "\nThis demonstrates the real robot behavior based on actual sensor data." << std::endl;
        
        std::cout << "\n[Demo Complete]" << std::endl;
    }
    
    // Clean up
    gl_sim.reset();
    
    std::cout << "Exiting cleanly." << std::endl;
    return 0;
}