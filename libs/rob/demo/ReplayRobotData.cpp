#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <regex>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "StateEstimator.h"
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
                // Extract pose
                std::regex pose_regex("Pose: ([0-9.-]+) ([0-9.-]+) ([0-9.-]+)");
                std::smatch match;
                if (std::regex_search(line, match, pose_regex)) {
                    reading.camera_pose = Eigen::Vector3d(
                        std::stod(match[1]), std::stod(match[2]), std::stod(match[3])
                    );
                }
            }
            else if (line.find("Body Velocity:") != std::string::npos) {
                // Extract commanded velocity
                std::regex vel_regex("Body Velocity: ([0-9.-]+) ([0-9.-]+) ([0-9.-]+)");
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

int main(int argc, char* argv[]) {
    std::cout << "[ReplayRobotData] Replaying robot sensor data to debug B-spline issues" << std::endl;
    
    // Load log file
    std::string log_file = (argc > 1) ? argv[1] : "/home/kodek/my_project/soccer/robot0_odometry.log";
    LogDataParser parser;
    if (!parser.LoadLogFile(log_file)) {
        return 1;
    }
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Create test robots
    rob::RobotManager simulated_robot;  // Uses log data
    rob::RobotManager ideal_robot;      // Perfect sensors
    
    // Test waypoints (1m forward movement)
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    
    // Configure both robots with B-spline
    simulated_robot.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    simulated_robot.SetBSplinePath(waypoints, 0.0);
    
    ideal_robot.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    ideal_robot.SetBSplinePath(waypoints, 0.0);
    
    // Replay sensor data
    size_t reading_index = 0;
    double start_time = util::GetCurrentTime();
    const auto& readings = parser.GetReadings();
    
    std::cout << "\nReplaying " << readings.size() << " sensor readings..." << std::endl;
    std::cout << "Red robot: Simulated with real sensor data" << std::endl;
    std::cout << "Blue robot: Ideal sensors\n" << std::endl;
    
    while (reading_index < readings.size()) {
        double current_time = util::GetCurrentTime();
        double elapsed_time = current_time - start_time;
        
        // Get current sensor reading
        const auto& reading = readings[reading_index];
        
        // Feed sensor data to simulated robot
        if (reading.wheel_rpms) {
            simulated_robot.NewMotorsData(reading.wheel_rpms.value());
        }
        if (reading.gyro_data) {
            simulated_robot.NewGyroData(reading.gyro_data.value());
        }
        if (reading.camera_pose) {
            simulated_robot.NewCameraData(reading.camera_pose.value());
        }
        
        // Update both robots
        simulated_robot.ControlLogic();
        simulated_robot.SenseLogic();
        
        ideal_robot.ControlLogic();
        ideal_robot.SenseLogic();
        
        // Update visualization
        soccer_objects[0].position = simulated_robot.GetPoseInWorldFrame();
        soccer_objects[1].position = ideal_robot.GetPoseInWorldFrame();
        
        // Print comparison
        if (reading_index % 60 == 0) { // Every ~1 second
            Eigen::Vector3d sim_pose = simulated_robot.GetPoseInWorldFrame();
            Eigen::Vector3d ideal_pose = ideal_robot.GetPoseInWorldFrame();
            Eigen::Vector3d error = sim_pose - ideal_pose;
            
            std::cout << "[t=" << elapsed_time << "s]" << std::endl;
            std::cout << "  Simulated: " << sim_pose.transpose() << std::endl;
            std::cout << "  Ideal:     " << ideal_pose.transpose() << std::endl;
            std::cout << "  Error:     " << error.head<2>().norm() << "m" << std::endl;
            
            if (reading.commanded_velocity) {
                std::cout << "  Cmd Vel:   " << reading.commanded_velocity.value().transpose() << std::endl;
            }
            std::cout << std::endl;
        }
        
        // Run simulation
        if (!gl_simulation.RunSimulationStep(soccer_objects, 0.016)) {
            break;
        }
        
        reading_index++;
        
        // Sync with real time
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    // Final summary
    Eigen::Vector3d final_sim = simulated_robot.GetPoseInWorldFrame();
    Eigen::Vector3d final_ideal = ideal_robot.GetPoseInWorldFrame();
    
    std::cout << "\n=== Final Results ===" << std::endl;
    std::cout << "Simulated robot position: " << final_sim.transpose() << std::endl;
    std::cout << "Ideal robot position:     " << final_ideal.transpose() << std::endl;
    std::cout << "Position error:           " << (final_sim - final_ideal).head<2>().norm() << "m" << std::endl;
    std::cout << "\nExpected: 1.0m forward" << std::endl;
    std::cout << "Simulated achieved: " << final_sim[0] << "m (" << (final_sim[0]/1.0)*100 << "%)" << std::endl;
    std::cout << "Ideal achieved:     " << final_ideal[0] << "m (" << (final_ideal[0]/1.0)*100 << "%)" << std::endl;
    
    return 0;
}