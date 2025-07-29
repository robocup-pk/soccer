#ifndef BANG_BANG_TRAJECTORY_H
#define BANG_BANG_TRAJECTORY_H

#include "RobotDescription.h"
#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <memory>
#include "AccelerationEnvelope.h"
namespace kin {
    class RobotDescription;
}

// Forward declarations
namespace rob {
    class RobotManager;
}

namespace ctrl {

// Use RobotParams from AccelerationEnvelope
using RobotParams = ctrl::RobotParams;

/**
 * @brief Represents one segment of bang-bang control
 */
enum class BangBangCase {
    CASE_1,     // w_dot_0 < 0: reverse direction first
    CASE_2_1,   // accelerate to max velocity
    CASE_2_2,   // cruise at max velocity  
    CASE_2_3,   // decelerate to destination
    CASE_3      // w_dot_0 > v_max: decelerate first
};

/**
 * @brief Single DOF bang-bang trajectory segment
 */
struct TrajectorySegment {
    BangBangCase case_type;
    double control_effort;      // qw(t): acceleration command
    double duration;            // t': segment duration
    double distance_traveled;   // w': distance covered in segment
    double final_velocity;      // w_dot': velocity at end of segment
};

/**
 * @brief Complete trajectory for one degree of freedom
 */
struct DOFTrajectory {
    std::vector<TrajectorySegment> segments;
    double total_time;
    double total_distance;
};

/**
 * @brief 2D trajectory combining x and y DOFs with synchronization
 */
struct Trajectory2D {
    DOFTrajectory x_trajectory;
    DOFTrajectory y_trajectory;
    double synchronized_time;
    double alpha_sync;          // synchronization parameter α ∈ (0, π/2)
};

/**
 * @brief Complete 3DOF trajectory including rotation
 */
struct TrajectoryComplete {
    Trajectory2D translation;
    DOFTrajectory rotation;
    double total_execution_time;
};

/**
 * @brief State for trajectory generation and execution
 */
struct TrajectoryState {
    // Position and velocity
    Eigen::Vector3d position;     // [x, y, θ]
    Eigen::Vector3d velocity;     // [ẋ, ẏ, θ̇]
    
    // Target
    Eigen::Vector3d target_position;
    Eigen::Vector3d target_velocity;  // Usually zero for full stop
    
    // Constraints
    double v_max;                 // Maximum linear velocity
    double a_max;                 // Maximum linear acceleration  
    double theta_dot_max;         // Maximum angular velocity
    double theta_ddot_max;        // Maximum angular acceleration
};

/**
 * @brief Bang-Bang Trajectory Generator implementing Purwin & D'Andrea algorithm
 * 
 * This class implements minimum-time trajectory generation for four-wheeled
 * omnidirectional vehicles as described in:
 * "Trajectory generation and control for four wheeled omnidirectional vehicles"
 * by Oliver Purwin and Raffaello D'Andrea (2006)
 * 
 * Full implementation including:
 * - Complete vehicle dynamics (Section 2-3)
 * - Monte Carlo acceleration envelope (Section 3)
 * - 5-case bang-bang control (Section 4) 
 * - X-Y synchronization via bisection (Section 5)
 * - Rotation trajectory generation (Section 6-7)
 * - Integration with RobotManager SetPath() and AddGoal()
 */
class BangBangTrajectory {
public:
    explicit BangBangTrajectory();
    ~BangBangTrajectory() = default;
    
    // === Integration with existing system ===
    
    /**
     * @brief Set path for RobotManager integration (replaces current SetPath)
     * @param path_fWorld Sequence of waypoints in world frame
     * @param t_start_s Start time (default: current time)
     * @return true if trajectory generation successful
     */
    bool SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s = 0.0);
    
    /**
     * @brief Add single goal for RobotManager integration (replaces current AddGoal)
     * @param goal Single waypoint [x, y, θ] in world frame
     * @return true if goal added successfully
     */
    bool AddGoal(const Eigen::Vector3d& goal);
    
    /**
     * @brief Update trajectory execution and get current velocity command
     * @param current_pose Current robot pose [x, y, θ]
     * @param current_time Current time since trajectory start
     * @return Velocity command [vx, vy, ω] for MotionController
     */
    Eigen::Vector3d Update(const Eigen::Vector3d& current_pose, double current_time);
    
    /**
     * @brief Check if trajectory execution is finished
     */
    bool IsFinished() const { return is_trajectory_finished_; }
    
    /**
     * @brief Get remaining execution time
     */
    double GetRemainingTime() const;
    
    // === Core trajectory generation ===
    
    /**
     * @brief Generate complete trajectory from current state to target
     * @param current_state Current robot state [x, y, θ, ẋ, ẏ, θ̇]
     * @param target_state Target robot state [x, y, θ, ẋ, ẏ, θ̇]
     * @return Complete 3DOF trajectory
     */
    TrajectoryComplete GenerateTrajectory(
        const TrajectoryState& current_state,
        const TrajectoryState& target_state
    );
    
    /**
     * @brief Generate trajectory for waypoint sequence
     * @param waypoints Sequence of waypoints [x, y, θ]
     * @param initial_velocity Initial velocity [ẋ, ẏ, θ̇]
     * @return Complete trajectory through all waypoints
     */
    std::vector<TrajectoryComplete> GenerateWaypointTrajectory(
        const std::vector<Eigen::Vector3d>& waypoints,
        const Eigen::Vector3d& initial_velocity = Eigen::Vector3d::Zero()
    );
    
    /**
     * @brief Evaluate trajectory at specific time
     * @param trajectory The trajectory to evaluate
     * @param time Time since trajectory start
     * @return State [position, velocity, acceleration]
     */
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> 
    EvaluateTrajectory(const TrajectoryComplete& trajectory, double time);
    
    // === Configuration and status ===
    
    /**
     * @brief Set maximum velocities and accelerations
     */
    void SetLimits(double v_max, double a_max, double omega_max, double alpha_max);
    
    /**
     * @brief Get acceleration envelope information
     */
    void PrintAccelerationEnvelope() const;
    
    /**
     * @brief Initialize with current robot state from RobotManager
     */
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);

private:
    // === Core components ===
    std::unique_ptr<AccelerationEnvelope> acceleration_envelope_;
    kin::RobotDescription robot_description_;
    
    // === Current trajectory state ===
    TrajectoryComplete current_trajectory_;
    double trajectory_start_time_;
    bool is_trajectory_active_;
    bool is_trajectory_finished_;
    std::vector<Eigen::Vector3d> remaining_waypoints_;
    
    // === Limits ===
    double v_max_ = 2.0;        // Maximum linear velocity (m/s)
    double a_max_ = 3.0;        // Maximum linear acceleration (m/s²) 
    double omega_max_ = 10.0;   // Maximum angular velocity (rad/s)
    double alpha_max_ = 20.0;   // Maximum angular acceleration (rad/s²)
    
    // === Section 4: Single DOF trajectory generation (5 cases) ===
    DOFTrajectory GenerateSingleDOF(
        double w0, double wf, double w_dot_0, double w_dot_f,
        double v_max, double a_max
    );
    
    // Individual case handlers from Section 4
    TrajectorySegment HandleCase1(double w_dot_0, double a_max);
    TrajectorySegment HandleCase2_1(double w_dot_0, double wf, double v_max, double a_max);
    TrajectorySegment HandleCase2_2(double w_dot_0, double wf, double v_max, double a_max);
    TrajectorySegment HandleCase2_3(double w_dot_0, double wf, double v_max, double a_max);
    TrajectorySegment HandleCase3(double w_dot_0, double v_max, double a_max);
    
    // === Section 5: X-Y Synchronization via α-parameterization ===
    Trajectory2D SynchronizeXY(
        const DOFTrajectory& x_traj, 
        const DOFTrajectory& y_traj,
        double a_max_total
    );
    
    // Bisection algorithm for finding optimal α (Section 5)
    double FindOptimalAlpha(
        double xf, double yf, double x_dot_0, double y_dot_0,
        double v_max, double a_max
    );
    
    // === Section 6-7: Rotation trajectory generation ===
    DOFTrajectory GenerateRotationTrajectory(
        double theta_0, double theta_f, double theta_dot_0, double theta_dot_f,
        double theta_dot_max, double theta_ddot_max
    );
    
    // === Vehicle dynamics and acceleration envelope (Section 2-3) ===
    void InitializeAccelerationEnvelope();
    Eigen::Vector3d ComputeMaxAcceleration(double direction_rad);
    bool IsAccelerationFeasible(const Eigen::Vector3d& acceleration);
    
    // === Utility functions ===
    double NormalizeAngle(double angle);
    bool IsTrajectoryValid(const TrajectoryComplete& trajectory);
    void ResetTrajectory();
    
    // === Integration helpers ===
    TrajectoryState GetCurrentStateFromPose(const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity);
    TrajectoryState CreateTargetState(const Eigen::Vector3d& target_pose);
};

} // namespace ctrl

#endif // BANG_BANG_TRAJECTORY_H