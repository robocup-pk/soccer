#pragma once

#include <vector>
#include <memory>
#include <tuple>
#include <Eigen/Dense>
#include <random>
#include "RobotDescription.h"
#include "Utils.h"

namespace rob {
    class RobotManager;
}

namespace ctrl {

/**
 * DBRRTTrajectoryPlanner - Dynamically feasible B-spline based RRT
 * 
 * Based on the paper: "Collision-free and dynamically feasible trajectory planning 
 * for omnidirectional mobile robots using a novel B-spline based rapidly exploring random tree"
 * by Sun et al., 2021
 * 
 * Key features:
 * 1. Combines B-spline properties with RRT exploration
 * 2. Generates dynamically feasible trajectories (respects velocity/acceleration limits)
 * 3. Exploits convex hull property for collision-free paths
 * 4. Specifically designed for omnidirectional robots
 */
class DBRRTTrajectoryPlanner {
public:
    DBRRTTrajectoryPlanner();
    ~DBRRTTrajectoryPlanner() = default;
    
    // Configuration from robot manager
    void InitializeFromRobotManager(rob::RobotManager* robot_manager);
    
    // Set dynamic limits for omnidirectional robot
    void SetDynamicLimits(double v_max, double a_max, double omega_max, double alpha_max);
    
    // Set B-spline parameters
    void SetBSplineParameters(int degree = 3, double time_interval = 0.1);
    
    // Set control gains for trajectory tracking
    void SetControlGains(double kp_pos, double kd_pos, double kp_angle, double kd_angle);
    
    // Main planning interface - plans from current state to goal
    bool PlanTrajectory(const Eigen::Vector3d& start_state, 
                       const Eigen::Vector3d& goal_state,
                       double planning_time_limit = 1.0);
    
    // Update function - returns velocity command
    Eigen::Vector3d Update(const Eigen::Vector3d& current_pose, double current_time);
    
    // Check if trajectory is valid and collision-free
    bool IsTrajectoryValid() const { return trajectory_valid_; }
    
    // Get trajectory duration
    double GetTrajectoryDuration() const { return trajectory_duration_; }
    
    // Replanning support for dynamic environments
    bool Replan(const Eigen::Vector3d& current_pose, double current_time);
    
private:
    // RRT Node structure
    struct RRTNode {
        Eigen::Vector3d state;           // [x, y, theta]
        Eigen::Vector3d velocity;        // [vx, vy, omega]
        double cost;                     // Cost from root
        std::shared_ptr<RRTNode> parent;
        std::vector<Eigen::Vector3d> control_points; // B-spline control points to this node
        
        RRTNode(const Eigen::Vector3d& s) : state(s), velocity(Eigen::Vector3d::Zero()), 
                                           cost(0.0), parent(nullptr) {}
    };
    
    // B-spline trajectory segment
    struct BSplineSegment {
        std::vector<Eigen::Vector3d> control_points;
        double start_time;
        double duration;
        int degree;
    };
    
    // Core RRT functions
    std::shared_ptr<RRTNode> SampleRandomNode();
    std::shared_ptr<RRTNode> FindNearestNode(const std::shared_ptr<RRTNode>& random_node);
    std::shared_ptr<RRTNode> SteerDynamicallyFeasible(const std::shared_ptr<RRTNode>& from_node,
                                                      const std::shared_ptr<RRTNode>& to_node);
    bool IsCollisionFree(const BSplineSegment& segment);
    bool IsDynamicallyFeasible(const BSplineSegment& segment);
    
    // B-spline functions
    Eigen::Vector3d EvaluateBSpline(const BSplineSegment& segment, double t);
    Eigen::Vector3d EvaluateBSplineDerivative(const BSplineSegment& segment, double t, int order);
    double BSplineBasis(int i, int p, double u, const std::vector<double>& knots);
    std::vector<double> GenerateUniformKnots(int num_control_points, int degree);
    
    // Dynamic feasibility checking
    bool CheckVelocityConstraints(const Eigen::Vector3d& velocity);
    bool CheckAccelerationConstraints(const Eigen::Vector3d& acceleration);
    
    // Trajectory generation from RRT path
    void GenerateBSplineTrajectory(const std::vector<std::shared_ptr<RRTNode>>& path);
    
    // Control point optimization for smoothness
    void OptimizeControlPoints(std::vector<Eigen::Vector3d>& control_points);
    
    // Convex hull property exploitation
    bool IsInsideConvexHull(const Eigen::Vector3d& point, 
                           const std::vector<Eigen::Vector3d>& control_points);
    
    // Fast marching guidance (for FMDB-RRT variant)
    std::vector<Eigen::Vector2d> ComputeFastMarchingPath(const Eigen::Vector2d& start,
                                                         const Eigen::Vector2d& goal);
    
    // Trajectory tracking control
    Eigen::Vector3d ComputeTrackingControl(const Eigen::Vector3d& current_pose, double t);
    
private:
    // Robot parameters
    kin::RobotDescription robot_description_;
    
    // Dynamic limits
    double v_max_ = 1.0;        // Maximum velocity (m/s)
    double a_max_ = 0.8;        // Maximum acceleration (m/s²)
    double omega_max_ = 3.0;    // Maximum angular velocity (rad/s)
    double alpha_max_ = 4.0;    // Maximum angular acceleration (rad/s²)
    
    // B-spline parameters
    int spline_degree_ = 3;     // Cubic B-spline by default
    double time_interval_ = 0.1; // Time between control points
    
    // Control gains
    double kp_position_ = 2.5;
    double kd_position_ = 0.8;
    double kp_angular_ = 2.0;
    double kd_angular_ = 0.5;
    
    // RRT parameters
    double rrt_step_size_ = 0.1;
    double goal_bias_ = 0.1;
    int max_iterations_ = 5000;
    double goal_tolerance_ = 0.05;
    
    // Current trajectory
    std::vector<BSplineSegment> trajectory_segments_;
    double trajectory_start_time_;
    double trajectory_duration_;
    bool trajectory_valid_ = false;
    
    // RRT tree
    std::vector<std::shared_ptr<RRTNode>> tree_nodes_;
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_x_;
    std::uniform_real_distribution<> dis_y_;
    std::uniform_real_distribution<> dis_theta_;
    std::uniform_real_distribution<> dis_goal_;
    
    // Field boundaries
    double field_min_x_ = -2.0;
    double field_max_x_ = 2.0;
    double field_min_y_ = -2.0;
    double field_max_y_ = 2.0;
    
    // Previous state for derivative computation
    Eigen::Vector3d previous_error_ = Eigen::Vector3d::Zero();
    double previous_time_ = 0.0;
    bool has_previous_state_ = false;
    
    // Replanning
    bool enable_replanning_ = true;
    double replan_threshold_ = 0.1;  // Deviation threshold for replanning
    
public:
    // Debug and visualization helpers
    std::vector<Eigen::Vector3d> GetPlannedPath() const;
    std::vector<Eigen::Vector3d> GetCurrentControlPoints() const;
    double GetCurrentProgress() const;
    Eigen::Vector3d GetDesiredPositionAtTime(double time) const;
};

} // namespace ctrl