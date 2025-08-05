/**
 * Simulate Log File Demo
 * 
 * This demo reads robot odometry data from a log file and visualizes the path
 * using UniformBSplineTrajectoryPlanner to create smooth trajectories.
 * 
 * Features:
 * - Parses robot odometry log file
 * - Extracts waypoints from logged positions
 * - Creates smooth B-spline trajectory through waypoints
 * - Visualizes both original path and smoothed trajectory
 * - Shows trajectory evolution in real-time
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <numeric>
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "UniformBSplineTrajectoryPlanner.h"
#include "Utils.h"

// Structure to hold odometry data
struct OdometryPoint {
    double x;
    double y;
    double theta;
    double timestamp;
};

// Parse odometry log file
std::vector<OdometryPoint> ParseOdometryLog(const std::string& filename) {
    std::vector<OdometryPoint> points;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "[Error] Could not open file: " << filename << std::endl;
        return points;
    }
    
    std::string line;
    double start_time = 0.0;
    bool first_point = true;
    
    // Regex to match odometry lines: [RobotNode::PublishOdometry] Pose: x y theta
    std::regex odometry_regex(R"(\[RobotNode::PublishOdometry\] Pose:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+))");
    
    while (std::getline(file, line)) {
        std::smatch matches;
        if (std::regex_search(line, matches, odometry_regex)) {
            OdometryPoint point;
            point.x = std::stod(matches[1]);
            point.y = std::stod(matches[2]);
            point.theta = std::stod(matches[3]);
            
            // Normalize theta to [-pi, pi] range
            while (point.theta > M_PI) point.theta -= 2.0 * M_PI;
            while (point.theta < -M_PI) point.theta += 2.0 * M_PI;
            
            // Simple timestamp based on line number (can be improved)
            point.timestamp = points.size() * 0.1; // Assume 10Hz logging
            
            points.push_back(point);
        }
    }
    
    file.close();
    std::cout << "[ParseOdometryLog] Loaded " << points.size() << " odometry points" << std::endl;
    return points;
}

// Filter odometry data to remove outliers and noise
std::vector<OdometryPoint> FilterOdometry(const std::vector<OdometryPoint>& raw_data) {
    std::vector<OdometryPoint> filtered;
    if (raw_data.empty()) return filtered;
    
    filtered.push_back(raw_data[0]);
    
    for (size_t i = 1; i < raw_data.size(); ++i) {
        const auto& last = filtered.back();
        const auto& current = raw_data[i];
        
        // Calculate position and angle changes
        double dx = current.x - last.x;
        double dy = current.y - last.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        double dtheta = std::abs(current.theta - last.theta);
        
        // Normalize angle difference
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        dtheta = std::abs(dtheta);
        
        // Reject physically impossible jumps (>5mm or >10 degrees at 10Hz)
        if (dist < 0.005 && dtheta < 0.174) { // 0.174 rad = 10 degrees
            filtered.push_back(current);
        } else {
            std::cout << "[FilterOdometry] Rejected outlier at index " << i 
                      << ": dist=" << dist*1000 << "mm, dtheta=" << dtheta*180/M_PI << "°" << std::endl;
        }
    }
    
    std::cout << "[FilterOdometry] Filtered " << raw_data.size() << " points to " 
              << filtered.size() << " points" << std::endl;
    return filtered;
}

// Smooth orientation data using moving average
void SmoothOrientation(std::vector<OdometryPoint>& data, int window_size = 3) {
    if (data.size() < window_size) return;
    
    std::vector<double> smoothed_theta;
    
    // Apply moving average
    for (size_t i = 0; i < data.size(); ++i) {
        int start = std::max(0, (int)i - window_size/2);
        int end = std::min((int)data.size() - 1, (int)i + window_size/2);
        
        // Use circular mean for angles
        double sin_sum = 0.0, cos_sum = 0.0;
        for (int j = start; j <= end; ++j) {
            sin_sum += std::sin(data[j].theta);
            cos_sum += std::cos(data[j].theta);
        }
        
        smoothed_theta.push_back(std::atan2(sin_sum, cos_sum));
    }
    
    // Apply smoothed values
    for (size_t i = 0; i < data.size(); ++i) {
        data[i].theta = smoothed_theta[i];
    }
    
    std::cout << "[SmoothOrientation] Applied orientation smoothing with window size " << window_size << std::endl;
}

// Convert odometry points to waypoints (with downsampling)
std::vector<Eigen::Vector3d> OdometryToWaypoints(const std::vector<OdometryPoint>& odometry, 
                                                  double min_distance = 0.1) {
    std::vector<Eigen::Vector3d> waypoints;
    
    if (odometry.empty()) return waypoints;
    
    // Always include first point
    waypoints.push_back(Eigen::Vector3d(odometry[0].x, odometry[0].y, odometry[0].theta));
    
    // Downsample based on distance
    for (size_t i = 1; i < odometry.size(); ++i) {
        const auto& last_wp = waypoints.back();
        double dx = odometry[i].x - last_wp[0];
        double dy = odometry[i].y - last_wp[1];
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist >= min_distance) {
            waypoints.push_back(Eigen::Vector3d(odometry[i].x, odometry[i].y, odometry[i].theta));
        }
    }
    
    // Always include last point if not already included
    if (waypoints.size() > 1) {
        const auto& last_odom = odometry.back();
        const auto& last_wp = waypoints.back();
        if (std::abs(last_odom.x - last_wp[0]) > 1e-6 || 
            std::abs(last_odom.y - last_wp[1]) > 1e-6) {
            waypoints.push_back(Eigen::Vector3d(last_odom.x, last_odom.y, last_odom.theta));
        }
    }
    
    std::cout << "[OdometryToWaypoints] Generated " << waypoints.size() 
              << " waypoints from " << odometry.size() << " odometry points" << std::endl;
    return waypoints;
}

// Simple robot visualization using trajectory planner directly
class SimulatedRobot {
private:
    ctrl::UniformBSplineTrajectoryPlanner trajectory_planner;
    Eigen::Vector3d current_pose;
    
public:
    SimulatedRobot() {
        current_pose = Eigen::Vector3d::Zero();
    }
    
    // Initialize trajectory
    bool SetPath(const std::vector<Eigen::Vector3d>& waypoints) {
        return trajectory_planner.SetPath(waypoints, 0.0);
    }
    
    // Set trajectory parameters
    void SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
        trajectory_planner.SetLimits(v_max, a_max, omega_max, alpha_max);
    }
    
    void SetFeedbackGains(double kp, double kd) {
        trajectory_planner.SetFeedbackGains(kp, kd);
    }
    
    // Set field boundaries
    // void SetFieldBoundaries(double min_x, double max_x, double min_y, double max_y) {
    //     trajectory_planner.SetFieldBoundaries(min_x, max_x, min_y, max_y);
    // }
    
    // Get position at time
    Eigen::Vector3d GetPositionAtTime(double time) {
        return trajectory_planner.GetDesiredPositionAtTime(time);
    }
    
    // Get current pose
    Eigen::Vector3d GetPose() const {
        return current_pose;
    }
    
    // Set pose
    void SetPose(const Eigen::Vector3d& pose) {
        current_pose = pose;
    }
    
    // Get trajectory info
    double GetTrajectoryDuration() const {
        return trajectory_planner.GetTrajectoryDuration();
    }
    
    double GetTotalArcLength() const {
        return trajectory_planner.GetTotalArcLength();
    }
    
    Eigen::Vector3d EvaluateAtParameter(double u) const {
        return trajectory_planner.EvaluateBSplineAtParameter(u);
    }
    
    std::vector<Eigen::Vector3d> GetControlPoints() const {
        return trajectory_planner.GetControlPoints();
    }
};

int main(int argc, char* argv[]) {
    std::cout << "\n=== Simulate Log File Demo ===" << std::endl;
    std::cout << "Visualizing robot trajectory from odometry log file\n" << std::endl;
    
    // Get log file path
    std::string log_file = "../robot0_odometry.log"; // Default path
    if (argc > 1) {
        log_file = argv[1];
    }
    
    // Parse odometry data
    std::cout << "Loading odometry data from: " << log_file << std::endl;
    auto raw_odometry_data = ParseOdometryLog(log_file);
    
    if (raw_odometry_data.empty()) {
        std::cerr << "No odometry data found!" << std::endl;
        return 1;
    }
    
    // Filter and smooth the odometry data
    std::cout << "\nFiltering odometry data..." << std::endl;
    auto filtered_data = FilterOdometry(raw_odometry_data);
    
    std::cout << "\nSmoothing orientation data..." << std::endl;
    SmoothOrientation(filtered_data, 5); // Use window size of 5
    
    // Use filtered data for trajectory generation
    auto odometry_data = filtered_data;
    
    // Convert to waypoints
    double waypoint_spacing = 0.002; // 2mm between waypoints (for small scale data)
    if (argc > 2) {
        waypoint_spacing = std::atof(argv[2]);
    }
    auto waypoints = OdometryToWaypoints(odometry_data, waypoint_spacing);
    
    if (waypoints.size() < 2) {
        std::cerr << "Not enough waypoints for trajectory!" << std::endl;
        return 1;
    }
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Create simulated robot
    SimulatedRobot robot;
    
    // Set initial pose
    robot.SetPose(waypoints[0]);
    
    // Analyze data range to set appropriate field boundaries
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    
    for (const auto& wp : waypoints) {
        min_x = std::min(min_x, wp[0]);
        max_x = std::max(max_x, wp[0]);
        min_y = std::min(min_y, wp[1]);
        max_y = std::max(max_y, wp[1]);
    }
    
    std::cout << "\nData range: X[" << min_x << ", " << max_x 
              << "], Y[" << min_y << ", " << max_y << "]" << std::endl;
    
    // Calculate scale-appropriate parameters
    double x_range = max_x - min_x;
    double y_range = max_y - min_y;
    double max_range = std::max(x_range, y_range);
    
    // Adaptive velocity limits based on data scale
    double v_max = std::min(0.05, max_range * 0.5); // Max 5cm/s or half the range per second
    double a_max = v_max * 0.5; // Gentle acceleration
    double omega_max = 0.5; // 0.5 rad/s for orientation
    double alpha_max = 0.25; // Gentle angular acceleration
    
    std::cout << "Setting adaptive limits: v_max=" << v_max << "m/s, a_max=" << a_max << "m/s²" << std::endl;
    
    robot.SetLimits(v_max, a_max, omega_max, alpha_max);
    robot.SetFeedbackGains(0.02, 0.0); // Gentle feedback gains
    
    // Set adaptive field boundaries with margin
    double margin = 0.1 * max_range; // 10% margin
    //robot.SetFieldBoundaries(min_x - margin, max_x + margin, min_y - margin, max_y + margin);
    
    // Create B-spline trajectory
    std::cout << "\nGenerating smooth B-spline trajectory..." << std::endl;
    if (!robot.SetPath(waypoints)) {
        std::cerr << "Failed to create trajectory!" << std::endl;
        return 1;
    }
    
    // Print trajectory info
    std::cout << "Trajectory duration: " << robot.GetTrajectoryDuration() 
              << " seconds" << std::endl;
    std::cout << "Total arc length: " << robot.GetTotalArcLength() 
              << " meters" << std::endl;
    
    // Visualization settings
    bool show_original_path = true;
    bool show_control_points = true;
    bool show_trajectory = true;
    bool animate_robot = true;
    double playback_speed = 1.0;
    
    // Simulation variables
    double sim_time = 0.0;
    int frame_count = 0;
    bool paused = false;
    
    // Store trajectory points for visualization
    std::vector<Eigen::Vector3d> trajectory_points;
    const int num_traj_samples = 100;
    for (int i = 0; i <= num_traj_samples; ++i) {
        double u = static_cast<double>(i) / num_traj_samples;
        trajectory_points.push_back(robot.EvaluateAtParameter(u));
    }
    
    std::cout << "\nControls:" << std::endl;
    std::cout << "  SPACE: Play/Pause animation" << std::endl;
    std::cout << "  O: Toggle original path" << std::endl;
    std::cout << "  C: Toggle control points" << std::endl;
    std::cout << "  T: Toggle trajectory" << std::endl;
    std::cout << "  R: Reset animation" << std::endl;
    std::cout << "  +/-: Increase/decrease playback speed" << std::endl;
    std::cout << "  ESC: Exit\n" << std::endl;
    
    // Main simulation loop
    while (true) {
        double dt = util::CalculateDt();
        
        // Run visualization step
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            std::cout << "\n[Demo] Simulation finished" << std::endl;
            break;
        }
        
        // Process input
        auto* window = gl_simulation.GetRawGLFW();
        
        // Check for key presses
        static bool space_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !space_pressed) {
            paused = !paused;
            space_pressed = true;
            std::cout << (paused ? "Paused" : "Playing") << std::endl;
        } else if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE) {
            space_pressed = false;
        }
        
        static bool o_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS && !o_pressed) {
            show_original_path = !show_original_path;
            o_pressed = true;
            std::cout << "Original path: " << (show_original_path ? "ON" : "OFF") << std::endl;
        } else if (glfwGetKey(window, GLFW_KEY_O) == GLFW_RELEASE) {
            o_pressed = false;
        }
        
        static bool c_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS && !c_pressed) {
            show_control_points = !show_control_points;
            c_pressed = true;
            std::cout << "Control points: " << (show_control_points ? "ON" : "OFF") << std::endl;
        } else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_RELEASE) {
            c_pressed = false;
        }
        
        static bool t_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !t_pressed) {
            show_trajectory = !show_trajectory;
            t_pressed = true;
            std::cout << "Trajectory: " << (show_trajectory ? "ON" : "OFF") << std::endl;
        } else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE) {
            t_pressed = false;
        }
        
        static bool r_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && !r_pressed) {
            sim_time = 0.0;
            robot.SetPose(waypoints[0]);
            r_pressed = true;
            std::cout << "Animation reset" << std::endl;
        } else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE) {
            r_pressed = false;
        }
        
        if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS) {
            playback_speed = std::min(playback_speed + 0.01, 5.0);
            std::cout << "Playback speed: " << playback_speed << "x" << std::endl;
        }
        
        if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) {
            playback_speed = std::max(playback_speed - 0.01, 0.1);
            std::cout << "Playback speed: " << playback_speed << "x" << std::endl;
        }
        
        // Update animation
        if (!paused && animate_robot) {
            sim_time += dt * playback_speed;
            
            // Get desired position from trajectory
            if (sim_time <= robot.GetTrajectoryDuration()) {
                Eigen::Vector3d desired_pos = robot.GetPositionAtTime(sim_time);
                robot.SetPose(desired_pos);
            } else {
                // Loop animation
                sim_time = 0.0;
            }
        }
        
        // Update robot position in visualization
        soccer_objects[0].position = robot.GetPose();
        
        // Draw additional visualization elements
        // TODO: Add custom rendering for paths, control points, etc.
        // This would require extending the GLSimulation class
        
        // Display info every second
        if (frame_count % 60 == 0) {
            Eigen::Vector3d pos = robot.GetPose();
            std::cout << "[" << std::fixed << std::setprecision(1) << sim_time 
                      << "s] Robot at: (" << std::setprecision(3) 
                      << pos[0] << ", " << pos[1] << "), θ=" 
                      << pos[2] * 180.0 / M_PI << "°" << std::endl;
        }
        
        frame_count++;
    }
    
    return 0;
}