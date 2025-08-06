#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include "RobotDescription.h"
#include "Utils.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

/**
 * BezierTrajectoryPlanner - RoboJackets-style trajectory planner
 * 
 * This implementation uses:
 * 1. Cubic Bezier curves for path smoothing
 * 2. Separate handling of position and orientation
 * 3. Trapezoidal velocity profiles
 * 4. Proper corner handling with intermediate points
 */
class BezierTrajectoryPlanner {
public:
    BezierTrajectoryPlanner();
    ~BezierTrajectoryPlanner() = default;
    
    /**
     * Control points for a cubic Bezier curve segment
     */
    struct CubicBezierControlPoints {
        Eigen::Vector2d p0, p1, p2, p3;
        
        CubicBezierControlPoints() = default;
        CubicBezierControlPoints(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
                                const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
            : p0(p0), p1(p1), p2(p2), p3(p3) {}
    };
    
    /**
     * Robot state at a specific time
     */
    struct RobotInstant {
        Eigen::Vector3d pose;      // x, y, theta
        Eigen::Vector3d velocity;  // vx, vy, omega
        double timestamp;
        
        RobotInstant() : pose(Eigen::Vector3d::Zero()), 
                        velocity(Eigen::Vector3d::Zero()), 
                        timestamp(0.0) {}
        
        RobotInstant(const Eigen::Vector3d& p, const Eigen::Vector3d& v, double t)
            : pose(p), velocity(v), timestamp(t) {}
    };
    
    // Main interface
    bool SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s = 0.0);
    Eigen::Vector3d Update(const Eigen::Vector3d& current_pose, double current_time);
    
    // Configuration
    void SetLimits(double v_max, double a_max, double omega_max, double alpha_max);
    void SetFeedbackGains(double kp, double kd);
    void SetFieldBoundaries(double min_x, double max_x, double min_y, double max_y);
    void SetCornerCutDistance(double distance) { corner_cut_distance_ = distance; }
    
    // Initialize from RobotManager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // State queries
    bool IsActive() const { return is_trajectory_active_; }
    bool IsFinished() const { return is_trajectory_finished_; }
    double GetTrajectoryDuration() const { return trajectory_duration_; }
    
    // Reset trajectory
    void ResetTrajectory();
    
    // Get current desired position
    Eigen::Vector3d GetCurrentDesiredPosition() const;
    Eigen::Vector3d GetDesiredPositionAtTime(double time) const;
    
private:
    // Preprocess waypoints to handle sharp corners
    std::vector<Eigen::Vector2d> PreprocessWaypoints(const std::vector<Eigen::Vector3d>& waypoints);
    
    // Fit cubic Bezier curves through 2D points
    void FitCubicBezier(const std::vector<Eigen::Vector2d>& points,
                       const Eigen::Vector2d& vi, const Eigen::Vector2d& vf);
    
    // Evaluate Bezier path at parameter s in [0,1]
    void EvaluateBezier(double s, Eigen::Vector2d* position = nullptr,
                       Eigen::Vector2d* tangent = nullptr,
                       double* curvature = nullptr) const;
    
    // Profile velocity along the path
    void ProfileVelocity(double initial_speed, double final_speed);
    
    // Plan angles separately (after position trajectory is created)
    void PlanAngles(const std::vector<Eigen::Vector3d>& waypoints);
    
    // Get trapezoidal motion time
    double GetTrapezoidalTime(double distance, double v_max, double a_max,
                            double v_start, double v_end) const;
    
    // Find the instant index for a given time
    int FindInstantIndex(double time) const;
    
    // Normalize angle to [-π, π]
    double NormalizeAngle(double angle) const;
    
    // Check if corner is sharp
    bool IsSharpCorner(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, 
                      const Eigen::Vector2d& p2) const;
    
private:
    // Robot description
    kin::RobotDescription robot_description_;
    
    // Original waypoints (with angles)
    std::vector<Eigen::Vector3d> waypoints_;
    
    // Preprocessed 2D points
    std::vector<Eigen::Vector2d> path_points_;
    
    // Bezier control points
    std::vector<CubicBezierControlPoints> bezier_segments_;
    
    // Trajectory instants
    std::vector<RobotInstant> trajectory_instants_;
    
    // Velocity and acceleration limits
    double v_max_ = 0.8;        // m/s
    double a_max_ = 0.5;        // m/s²
    double omega_max_ = 2.5;    // rad/s
    double alpha_max_ = 3.0;    // rad/s²
    
    // Feedback control gains
    double kp_ = 0.05;  // Proportional gain
    double kd_ = 0.3;   // Derivative gain
    
    // Field boundaries
    double field_min_x_ = -4.5;
    double field_max_x_ = 4.5;
    double field_min_y_ = -3.0;
    double field_max_y_ = 3.0;
    
    // Corner handling
    double corner_cut_distance_ = 0.1;  // Distance to cut corners (10cm default)
    double sharp_corner_angle_ = 2.356; // 135 degrees in radians
    
    // Trajectory timing
    double trajectory_start_time_;
    double trajectory_duration_;
    
    // Trajectory state
    bool is_trajectory_active_;
    bool is_trajectory_finished_;
    
    // Interpolation parameters
    static constexpr int kInterpolationsPerBezier = 40;
};

} // namespace ctrl