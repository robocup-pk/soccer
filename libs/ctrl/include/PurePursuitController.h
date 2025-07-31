#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <Eigen/Dense>

namespace ctrl {

/**
 * Pure Pursuit Path Tracking Controller for Omnidirectional SSL Robots
 * 
 * Based on the classic Pure Pursuit algorithm but adapted for holonomic drive systems.
 * This implementation is based on AtsushiSakai's PythonRobotics Pure Pursuit algorithm
 * and modified for SSL (Small Size League) robots.
 * 
 * Key differences from traditional Pure Pursuit:
 * - No steering constraints (omnidirectional movement)
 * - Direct velocity commands in x, y, and angular
 * - Decoupled translation and rotation control
 */
class PurePursuitController {
public:
    struct Config {
        double look_ahead_gain = 0.5;          // Look forward gain
        double base_look_ahead_distance = 0.1; // [m] Base look-ahead distance for SSL robots  
        double speed_proportional_gain = 2.0;  // Speed proportional gain
        double position_tolerance = 0.02;      // [m] Position tolerance for waypoint reached
        double angular_tolerance = 0.1;        // [rad] Angular tolerance
        double max_linear_velocity = 0.6;      // [m/s] Maximum linear velocity
        double max_angular_velocity = 2.0;     // [rad/s] Maximum angular velocity
        double dt = 0.02;                      // [s] Control time step (50Hz)
    };

    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;           // Robot orientation
        double vx = 0.0;            // Linear velocity x
        double vy = 0.0;            // Linear velocity y  
        double vyaw = 0.0;          // Angular velocity
        
        // Convert to/from Eigen::Vector3d for compatibility
        Eigen::Vector3d position() const { return Eigen::Vector3d(x, y, yaw); }
        Eigen::Vector3d velocity() const { return Eigen::Vector3d(vx, vy, vyaw); }
        
        void setPosition(const Eigen::Vector3d& pos) { x = pos.x(); y = pos.y(); yaw = pos.z(); }
        void setVelocity(const Eigen::Vector3d& vel) { vx = vel.x(); vy = vel.y(); vyaw = vel.z(); }
    };

    struct ControlOutput {
        double vx_cmd = 0.0;        // Commanded linear velocity x [m/s]
        double vy_cmd = 0.0;        // Commanded linear velocity y [m/s] 
        double vyaw_cmd = 0.0;      // Commanded angular velocity [rad/s]
        bool path_finished = false; // True when robot reaches final waypoint
        int current_target_index = -1; // Index of current target waypoint
        
        // Convert to Eigen::Vector3d for compatibility
        Eigen::Vector3d velocity() const { return Eigen::Vector3d(vx_cmd, vy_cmd, vyaw_cmd); }
    };

private:
    Config config_;
    std::vector<Eigen::Vector3d> path_;
    int current_target_index_;
    bool path_finished_;

public:
    PurePursuitController();
    explicit PurePursuitController(const Config& config);
    
    // Set the path to follow (sequence of waypoints)
    void setPath(const std::vector<Eigen::Vector3d>& path);
    
    // Update control and get velocity commands
    ControlOutput update(const RobotState& robot_state);
    
    // Get current configuration
    const Config& getConfig() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }
    
    // Path information
    bool isPathFinished() const { return path_finished_; }
    int getCurrentTargetIndex() const { return current_target_index_; }
    const std::vector<Eigen::Vector3d>& getPath() const { return path_; }
    
    // Reset controller state
    void reset();

private:
    // Core Pure Pursuit functions
    int searchTargetIndex(const RobotState& robot_state) const;
    double calculateLookAheadDistance(const RobotState& robot_state) const;
    ControlOutput calculateControlOutput(const RobotState& robot_state, int target_index) const;
    
    // Utility functions
    double normalizeAngle(double angle) const;
    double calculateDistance(double x1, double y1, double x2, double y2) const;
    bool isWaypointReached(const RobotState& robot_state, const Eigen::Vector3d& waypoint) const;
};

} // namespace ctrl