#ifndef M_TRAJECTORY_CONTROLLER_H
#define M_TRAJECTORY_CONTROLLER_H

#include "M_TrajectoryPlanner.h"
#include <Eigen/Dense>
#include <memory>
#include <queue>
#include <mutex>
#include <string>

namespace ctrl {

/**
 * M_TrajectoryController - Advanced controller for executing Bang-Bang trajectories
 * Manages trajectory execution, switching, and velocity output
 */

// Trajectory execution state
enum class M_TrajectoryState {
    IDLE,           // No active trajectory
    EXECUTING,      // Currently executing trajectory
    FINISHED,       // Trajectory completed
    FAILED          // Trajectory execution failed
};

// Controller configuration
struct M_ControllerConfig {
    double position_tolerance = 0.01;      // Position error tolerance [m]
    double velocity_tolerance = 0.1;       // Velocity error tolerance [m/s]
    double angle_tolerance = 0.05;         // Angle error tolerance [rad]
    double lookahead_time = 0.05;          // Lookahead time for trajectory following [s]
    double max_velocity_correction = 0.5;  // Maximum velocity correction [m/s]
    double proportional_gain = 0.8;        // Proportional gain for position error correction
    bool enable_velocity_feedforward = true; // Enable velocity feedforward
    bool enable_position_correction = true;  // Enable position error correction
};

// Trajectory execution info
struct M_TrajectoryInfo {
    M_TrajectoryState state = M_TrajectoryState::IDLE;
    double current_time = 0.0;              // Current execution time [s]
    double total_time = 0.0;                // Total trajectory time [s]
    double progress = 0.0;                  // Execution progress [0-1]
    Eigen::Vector3d position_error;         // Current position error
    Eigen::Vector3d velocity_error;         // Current velocity error
    std::string status_message;             // Human-readable status
};

// Trajectory queue entry
struct M_TrajectoryQueueEntry {
    std::shared_ptr<M_Trajectory> trajectory;
    double start_time = 0.0;               // Absolute start time
    bool immediate_execution = false;       // Execute immediately
    std::string id;                        // Trajectory identifier
};

/**
 * Main trajectory controller class
 */
class M_TrajectoryController {
public:
    M_TrajectoryController();
    explicit M_TrajectoryController(const M_ControllerConfig& config);
    ~M_TrajectoryController() = default;
    
    // Configuration
    void setConfig(const M_ControllerConfig& config);
    const M_ControllerConfig& getConfig() const { return config_; }
    
    // Trajectory management
    void setTrajectory(std::shared_ptr<M_Trajectory> trajectory, bool immediate = true);
    void addTrajectoryToQueue(std::shared_ptr<M_Trajectory> trajectory, 
                              double start_time = -1.0, const std::string& id = "");
    void clearTrajectoryQueue();
    void stopCurrentTrajectory();
    
    // State updates
    void updateRobotState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    void updateTime(double current_time);
    
    // Control output
    Eigen::Vector3d getDesiredVelocity() const;
    Eigen::Vector3d getDesiredPosition() const;
    Eigen::Vector3d getDesiredAcceleration() const;
    
    // Status and monitoring
    M_TrajectoryInfo getTrajectoryInfo() const;
    M_TrajectoryState getState() const { return current_state_; }
    bool isTrajectoryActive() const { return current_state_ == M_TrajectoryState::EXECUTING; }
    bool isTrajectoryFinished() const { return current_state_ == M_TrajectoryState::FINISHED; }
    
    // Debugging and logging
    void printStatus() const;
    void enableLogging(bool enable) { logging_enabled_ = enable; }
    
    // Advanced features
    void setEmergencyStop(bool stop);
    bool getEmergencyStop() const { return emergency_stop_; }
    void resetController();

private:
    // Configuration
    M_ControllerConfig config_;
    
    // Current state
    M_TrajectoryState current_state_;
    std::shared_ptr<M_Trajectory> current_trajectory_;
    double trajectory_start_time_;
    double current_time_;
    
    // Robot state
    Eigen::Vector3d robot_position_;
    Eigen::Vector3d robot_velocity_;
    
    // Control variables
    Eigen::Vector3d desired_velocity_;
    Eigen::Vector3d desired_position_;
    Eigen::Vector3d desired_acceleration_;
    
    // Error tracking
    Eigen::Vector3d position_error_;
    Eigen::Vector3d velocity_error_;
    
    // Trajectory queue
    std::queue<M_TrajectoryQueueEntry> trajectory_queue_;
    mutable std::mutex queue_mutex_;
    
    // Control flags
    bool emergency_stop_;
    bool logging_enabled_;
    
    // Internal methods
    void updateTrajectoryExecution();
    void processTrajectoryQueue();
    void calculateDesiredOutputs();
    void calculatePositionError();
    void calculateVelocityError();
    void logTrajectoryInfo() const;
    
    // Trajectory switching
    void startTrajectory(std::shared_ptr<M_Trajectory> trajectory);
    void finishTrajectory();
    void switchToNextTrajectory();
    
    // Control calculations
    Eigen::Vector3d calculateVelocityFeedforward(double t) const;
    Eigen::Vector3d calculatePositionCorrection() const;
    Eigen::Vector3d limitVelocityCorrection(const Eigen::Vector3d& correction) const;
};

/**
 * M_TrajectoryManager - High-level trajectory management (similar to our existing TrajectoryManager)
 * Integrates M_TrajectoryController with our existing robot management system
 */
class M_TrajectoryManager {
public:
    M_TrajectoryManager();
    ~M_TrajectoryManager() = default;
    
    // Path management (compatibility with existing system)
    bool CreateTrajectoriesFromPath(const std::vector<Eigen::Vector3d>& path_fWorld, 
                                    double t_start_s);
    
    // Robot state updates
    void UpdateRobotState(const Eigen::Vector3d& pose_est, const Eigen::Vector3d& velocity_est);
    
    // Control output (compatibility with existing TrajectoryManager interface)
    std::pair<bool, Eigen::Vector3d> Update(const Eigen::Vector3d& pose_est);
    Eigen::Vector3d GetVelocityAtT(double current_time_s);
    Eigen::Vector3d GetPositionAtT(double current_time_s);
    
    // Configuration
    void SetMoveConstraints(const M_MoveConstraints& constraints);
    const M_MoveConstraints& GetMoveConstraints() const { return move_constraints_; }
    
    void SetControllerConfig(const M_ControllerConfig& config);
    const M_ControllerConfig& GetControllerConfig() const { return controller_.getConfig(); }
    
    // Status
    bool IsTrajectoryActive() const { return controller_.isTrajectoryActive(); }
    bool IsTrajectoryFinished() const { return controller_.isTrajectoryFinished(); }
    M_TrajectoryInfo GetTrajectoryInfo() const { return controller_.getTrajectoryInfo(); }
    
    // Debugging
    void Print() const;
    void EnableLogging(bool enable) { controller_.enableLogging(enable); }
    
    // Emergency controls
    void EmergencyStop() { controller_.setEmergencyStop(true); }
    void ResumeExecution() { controller_.setEmergencyStop(false); }

private:
    M_TrajectoryController controller_;
    M_TrajectoryPlanner planner_;
    M_MoveConstraints move_constraints_;
    
    // Current robot state
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_velocity_;
    double current_time_;
    
    // Path tracking
    std::vector<Eigen::Vector3d> current_path_;
    size_t current_waypoint_index_;
    bool path_active_;
    
    // Internal methods
    void generateTrajectoryFromPath(const std::vector<Eigen::Vector3d>& path, 
                                    double start_time);
    void updateCurrentTime();
    M_RobotState createRobotState() const;
};

} // namespace ctrl

#endif // M_TRAJECTORY_CONTROLLER_H