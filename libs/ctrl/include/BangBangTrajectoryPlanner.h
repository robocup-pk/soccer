#ifndef BANGBANG_TRAJECTORY_PLANNER_H
#define BANGBANG_TRAJECTORY_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <memory>

namespace ctrl {

// Represents a single segment of a bang-bang trajectory
struct BangBangSegment {
    double duration;      // Duration of this segment
    double acceleration;  // Acceleration during this segment (can be negative)
    double initial_vel;   // Initial velocity at start of segment
    double initial_pos;   // Initial position at start of segment
    
    // Get position at time t within this segment (t=0 is segment start)
    double GetPosition(double t) const {
        return initial_pos + initial_vel * t + 0.5 * acceleration * t * t;
    }
    
    // Get velocity at time t within this segment
    double GetVelocity(double t) const {
        return initial_vel + acceleration * t;
    }
};

// 1D Bang-Bang trajectory (for a single axis)
class BangBangTrajectory1D {
public:
    BangBangTrajectory1D() = default;
    
    // Generate optimal bang-bang trajectory
    void Generate(double initial_pos, double target_pos, 
                  double initial_vel, double max_vel, double max_acc);
    
    // Query methods
    double GetPosition(double t) const;
    double GetVelocity(double t) const;
    double GetAcceleration(double t) const;
    double GetTotalTime() const;
    
    // Get the segments for debugging/visualization
    const std::vector<BangBangSegment>& GetSegments() const { return segments_; }
    
private:
    std::vector<BangBangSegment> segments_;
    double total_time_ = 0.0;
    
    // Find which segment contains time t
    int FindSegmentIndex(double t) const;
    
    // Helper functions for trajectory generation
    double ComputeBrakingDistance(double vel, double max_acc) const;
    void GenerateThreeSegmentProfile(double initial_pos, double target_pos,
                                     double initial_vel, double max_vel, double max_acc);
};

// 2D Bang-Bang trajectory (synchronized X and Y)
class BangBangTrajectory2D {
public:
    BangBangTrajectory2D() = default;
    
    // Generate synchronized 2D trajectory
    void Generate(const Eigen::Vector2d& initial_pos,
                  const Eigen::Vector2d& target_pos,
                  const Eigen::Vector2d& initial_vel,
                  double max_vel,
                  double max_acc);
    
    // Query methods
    Eigen::Vector2d GetPosition(double t) const;
    Eigen::Vector2d GetVelocity(double t) const;
    Eigen::Vector2d GetAcceleration(double t) const;
    double GetTotalTime() const;
    
    // Access individual axis trajectories
    const BangBangTrajectory1D& GetXTrajectory() const { return x_trajectory_; }
    const BangBangTrajectory1D& GetYTrajectory() const { return y_trajectory_; }
    
private:
    BangBangTrajectory1D x_trajectory_;
    BangBangTrajectory1D y_trajectory_;
    
    // Synchronize the two trajectories to finish at the same time
    void SynchronizeTrajectories(double max_vel, double max_acc);
};

// Obstacle representation for collision checking
struct Obstacle {
    Eigen::Vector2d position;
    double radius;
    Eigen::Vector2d velocity;  // For moving obstacles
    bool is_static;
    
    // Get obstacle position at time t
    Eigen::Vector2d GetPositionAtTime(double t) const {
        if (is_static) return position;
        return position + velocity * t;
    }
};

// Main Bang-Bang trajectory planner with obstacle avoidance
class BangBangTrajectoryPlanner {
public:
    BangBangTrajectoryPlanner();
    
    // Set robot parameters
    void SetRobotRadius(double radius) { robot_radius_ = radius; }
    void SetLimits(double max_vel, double max_acc);
    void SetFeedbackGains(double kp, double kd) { kp_ = kp; kd_ = kd; }
    
    // Set field boundaries
    void SetFieldBoundaries(double min_x, double max_x, double min_y, double max_y);
    
    // Main planning interface
    bool SetPath(const std::vector<Eigen::Vector3d>& waypoints, double start_time = 0.0);
    
    // Update method called in control loop
    Eigen::Vector3d Update(const Eigen::Vector3d& current_pose, double current_time);
    
    // Check if trajectory is finished
    bool IsFinished() const { return is_finished_; }
    
    // Obstacle management
    void ClearObstacles() { obstacles_.clear(); }
    void AddObstacle(const Obstacle& obstacle) { obstacles_.push_back(obstacle); }
    
    // Get current trajectory for visualization
    const BangBangTrajectory2D& GetCurrentTrajectory() const { return current_trajectory_; }
    
private:
    // Robot parameters
    double robot_radius_ = 0.09;  // 90mm default
    double max_vel_ = 2.0;        // m/s
    double max_acc_ = 3.0;        // m/s^2
    double kp_ = 10.0;
    double kd_ = 2.0;
    
    // Field boundaries
    double field_min_x_ = -4.5;
    double field_max_x_ = 4.5;
    double field_min_y_ = -3.0;
    double field_max_y_ = 3.0;
    
    // Trajectory state
    BangBangTrajectory2D current_trajectory_;
    std::vector<Eigen::Vector3d> waypoints_;
    size_t current_waypoint_index_ = 0;
    double trajectory_start_time_ = 0.0;
    bool is_finished_ = true;
    bool has_trajectory_ = false;
    
    // Obstacles
    std::vector<Obstacle> obstacles_;
    
    // Collision checking
    bool CheckTrajectoryCollision(const BangBangTrajectory2D& trajectory,
                                 double time_offset = 0.0) const;
    double GetFirstCollisionTime(const BangBangTrajectory2D& trajectory,
                                double time_offset = 0.0) const;
    
    // Sub-destination generation for obstacle avoidance
    std::vector<Eigen::Vector2d> GenerateSubDestinations(const Eigen::Vector2d& start,
                                                         const Eigen::Vector2d& goal) const;
    
    // Path planning with obstacle avoidance
    bool PlanTrajectory(const Eigen::Vector2d& start_pos,
                       const Eigen::Vector2d& start_vel,
                       const Eigen::Vector2d& goal_pos);
    
    // Helper to check if position is within field boundaries
    bool IsWithinField(const Eigen::Vector2d& pos) const;
    
    // Compute feedback control for trajectory tracking
    Eigen::Vector3d ComputeFeedbackControl(const Eigen::Vector3d& current_pose,
                                          const Eigen::Vector2d& desired_pos,
                                          const Eigen::Vector2d& desired_vel) const;
};

} // namespace ctrl

#endif // BANGBANG_TRAJECTORY_PLANNER_H