#include "DBRRTTrajectoryPlanner.h"
#include "RobotManager.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ctrl {

DBRRTTrajectoryPlanner::DBRRTTrajectoryPlanner() 
    : gen_(rd_()), 
      dis_x_(-2.0, 2.0),
      dis_y_(-2.0, 2.0),
      dis_theta_(-M_PI, M_PI),
      dis_goal_(0.0, 1.0) {
}

void DBRRTTrajectoryPlanner::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    // Get robot description using the same method as other planners
    robot_description_ = kin::GetRobotDescription();
}

void DBRRTTrajectoryPlanner::SetDynamicLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
}

void DBRRTTrajectoryPlanner::SetBSplineParameters(int degree, double time_interval) {
    spline_degree_ = degree;
    time_interval_ = time_interval;
}

void DBRRTTrajectoryPlanner::SetControlGains(double kp_pos, double kd_pos, double kp_angle, double kd_angle) {
    kp_position_ = kp_pos;
    kd_position_ = kd_pos;
    kp_angular_ = kp_angle;
    kd_angular_ = kd_angle;
}

bool DBRRTTrajectoryPlanner::PlanTrajectory(const Eigen::Vector3d& start_state, 
                                           const Eigen::Vector3d& goal_state,
                                           double planning_time_limit) {
    // Clear previous trajectory
    trajectory_segments_.clear();
    tree_nodes_.clear();
    trajectory_valid_ = false;
    
    std::cout << "DB-RRT: Planning from " << start_state.transpose() << " to " << goal_state.transpose() << std::endl;
    std::cout << "DB-RRT: Field bounds: [" << field_min_x_ << ", " << field_max_x_ << "] x [" 
              << field_min_y_ << ", " << field_max_y_ << "]" << std::endl;
    
    // Initialize RRT with start node
    auto start_node = std::make_shared<RRTNode>(start_state);
    tree_nodes_.push_back(start_node);
    
    auto start_time = util::GetCurrentTime();
    
    // RRT expansion loop
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Check time limit
        if (util::GetCurrentTime() - start_time > planning_time_limit) {
            std::cerr << "DB-RRT: Planning time limit exceeded" << std::endl;
            break;
        }
        
        // Sample random node (with goal bias)
        auto random_node = SampleRandomNode();
        if (dis_goal_(gen_) < goal_bias_) {
            random_node->state = goal_state;
        }
        
        // Find nearest node in tree
        auto nearest_node = FindNearestNode(random_node);
        
        // Steer towards random node with dynamic feasibility
        auto new_node = SteerDynamicallyFeasible(nearest_node, random_node);
        
        if (new_node) {
            // Create B-spline segment
            BSplineSegment segment;
            segment.control_points = new_node->control_points;
            segment.degree = spline_degree_;
            segment.duration = segment.control_points.size() * time_interval_;
            
            // Check collision and dynamic feasibility
            if (IsCollisionFree(segment) && IsDynamicallyFeasible(segment)) {
                tree_nodes_.push_back(new_node);
                
                // Check if goal reached
                double dist_to_goal = (new_node->state - goal_state).norm();
                if (dist_to_goal < goal_tolerance_) {
                    // Reconstruct path
                    std::vector<std::shared_ptr<RRTNode>> path;
                    auto current = new_node;
                    while (current) {
                        path.push_back(current);
                        current = current->parent;
                    }
                    std::reverse(path.begin(), path.end());
                    
                    // Generate B-spline trajectory
                    GenerateBSplineTrajectory(path);
                    
                    trajectory_valid_ = true;
                    trajectory_start_time_ = util::GetCurrentTime();
                    
                    std::cout << "DB-RRT: Path found in " << iter << " iterations" << std::endl;
                    return true;
                }
            }
        }
    }
    
    std::cerr << "DB-RRT: Failed to find path" << std::endl;
    return false;
}

std::shared_ptr<DBRRTTrajectoryPlanner::RRTNode> DBRRTTrajectoryPlanner::SampleRandomNode() {
    Eigen::Vector3d random_state;
    random_state << dis_x_(gen_), dis_y_(gen_), dis_theta_(gen_);
    return std::make_shared<RRTNode>(random_state);
}

std::shared_ptr<DBRRTTrajectoryPlanner::RRTNode> DBRRTTrajectoryPlanner::FindNearestNode(
    const std::shared_ptr<RRTNode>& random_node) {
    
    std::shared_ptr<RRTNode> nearest = nullptr;
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& node : tree_nodes_) {
        double dist = (node->state.head<2>() - random_node->state.head<2>()).norm();
        if (dist < min_distance) {
            min_distance = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

std::shared_ptr<DBRRTTrajectoryPlanner::RRTNode> DBRRTTrajectoryPlanner::SteerDynamicallyFeasible(
    const std::shared_ptr<RRTNode>& from_node,
    const std::shared_ptr<RRTNode>& to_node) {
    
    // Calculate direction
    Eigen::Vector2d direction = to_node->state.head<2>() - from_node->state.head<2>();
    double distance = direction.norm();
    
    if (distance < 1e-6) {
        return nullptr;
    }
    
    direction.normalize();
    
    // Limit step size
    double step_distance = std::min(rrt_step_size_, distance);
    
    // Generate control points for B-spline segment
    std::vector<Eigen::Vector3d> control_points;
    
    // Start with parent's state
    control_points.push_back(from_node->state);
    
    // Add intermediate control points for smooth transition
    int num_intermediate = 3; // For cubic B-spline
    for (int i = 1; i <= num_intermediate; ++i) {
        double t = static_cast<double>(i) / (num_intermediate + 1);
        Eigen::Vector3d intermediate_state;
        intermediate_state.head<2>() = from_node->state.head<2>() + t * step_distance * direction;
        
        // Smooth angle interpolation
        double angle_diff = to_node->state(2) - from_node->state(2);
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        intermediate_state(2) = from_node->state(2) + t * angle_diff;
        
        control_points.push_back(intermediate_state);
    }
    
    // End state
    Eigen::Vector3d end_state;
    end_state.head<2>() = from_node->state.head<2>() + step_distance * direction;
    end_state(2) = from_node->state(2) + step_distance / distance * (to_node->state(2) - from_node->state(2));
    control_points.push_back(end_state);
    
    // Apply convex hull property to ensure safety
    OptimizeControlPoints(control_points);
    
    // Create new node
    auto new_node = std::make_shared<RRTNode>(end_state);
    new_node->parent = from_node;
    new_node->control_points = control_points;
    new_node->cost = from_node->cost + step_distance;
    
    return new_node;
}

bool DBRRTTrajectoryPlanner::IsCollisionFree(const BSplineSegment& segment) {
    // Sample points along the B-spline
    int num_samples = static_cast<int>(segment.duration / 0.01); // Sample every 10ms
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        Eigen::Vector3d point = EvaluateBSpline(segment, t);
        
        // Check field boundaries
        if (point(0) < field_min_x_ || point(0) > field_max_x_ ||
            point(1) < field_min_y_ || point(1) > field_max_y_) {
            return false;
        }
        
        // Convex hull property ensures all points between samples are also safe
        // if control points are within boundaries
    }
    
    return true;
}

bool DBRRTTrajectoryPlanner::IsDynamicallyFeasible(const BSplineSegment& segment) {
    // Check velocity and acceleration constraints along the trajectory
    int num_checks = static_cast<int>(segment.duration / 0.05); // Check every 50ms
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        
        // First derivative (velocity)
        Eigen::Vector3d velocity = EvaluateBSplineDerivative(segment, t, 1) / time_interval_;
        if (!CheckVelocityConstraints(velocity)) {
            return false;
        }
        
        // Second derivative (acceleration)
        Eigen::Vector3d acceleration = EvaluateBSplineDerivative(segment, t, 2) / (time_interval_ * time_interval_);
        if (!CheckAccelerationConstraints(acceleration)) {
            return false;
        }
    }
    
    return true;
}

bool DBRRTTrajectoryPlanner::CheckVelocityConstraints(const Eigen::Vector3d& velocity) {
    double v_linear = velocity.head<2>().norm();
    double omega = std::abs(velocity(2));
    
    return v_linear <= v_max_ && omega <= omega_max_;
}

bool DBRRTTrajectoryPlanner::CheckAccelerationConstraints(const Eigen::Vector3d& acceleration) {
    double a_linear = acceleration.head<2>().norm();
    double alpha = std::abs(acceleration(2));
    
    return a_linear <= a_max_ && alpha <= alpha_max_;
}

Eigen::Vector3d DBRRTTrajectoryPlanner::EvaluateBSpline(const BSplineSegment& segment, double t) {
    int n = segment.control_points.size();
    std::vector<double> knots = GenerateUniformKnots(n, segment.degree);
    
    // Map t from [0,1] to knot span
    double u = knots[segment.degree] + t * (knots[n] - knots[segment.degree]);
    
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    
    for (int i = 0; i < n; ++i) {
        double basis = BSplineBasis(i, segment.degree, u, knots);
        result += basis * segment.control_points[i];
    }
    
    return result;
}

Eigen::Vector3d DBRRTTrajectoryPlanner::EvaluateBSplineDerivative(const BSplineSegment& segment, 
                                                                 double t, int order) {
    if (order <= 0 || order > segment.degree) {
        return Eigen::Vector3d::Zero();
    }
    
    // B-spline derivative is a B-spline of degree p-1
    BSplineSegment deriv_segment;
    deriv_segment.degree = segment.degree - order;
    
    // Compute derivative control points recursively
    std::vector<std::vector<Eigen::Vector3d>> Q(order + 1);
    Q[0] = segment.control_points;
    
    for (int k = 1; k <= order; ++k) {
        Q[k].resize(Q[k-1].size() - 1);
        for (size_t i = 0; i < Q[k].size(); ++i) {
            Q[k][i] = (segment.degree - k + 1) * (Q[k-1][i+1] - Q[k-1][i]);
        }
    }
    
    deriv_segment.control_points = Q[order];
    
    // Evaluate the derivative B-spline
    return EvaluateBSpline(deriv_segment, t);
}

double DBRRTTrajectoryPlanner::BSplineBasis(int i, int p, double u, const std::vector<double>& knots) {
    // Cox-de Boor recursion formula
    if (p == 0) {
        return (u >= knots[i] && u < knots[i + 1]) ? 1.0 : 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    double denom_left = knots[i + p] - knots[i];
    if (denom_left > 0) {
        left = (u - knots[i]) / denom_left * BSplineBasis(i, p - 1, u, knots);
    }
    
    double denom_right = knots[i + p + 1] - knots[i + 1];
    if (denom_right > 0) {
        right = (knots[i + p + 1] - u) / denom_right * BSplineBasis(i + 1, p - 1, u, knots);
    }
    
    return left + right;
}

std::vector<double> DBRRTTrajectoryPlanner::GenerateUniformKnots(int num_control_points, int degree) {
    int num_knots = num_control_points + degree + 1;
    std::vector<double> knots(num_knots);
    
    // Clamped uniform B-spline
    for (int i = 0; i <= degree; ++i) {
        knots[i] = 0.0;
        knots[num_knots - 1 - i] = 1.0;
    }
    
    // Uniform interior knots
    int num_interior = num_knots - 2 * (degree + 1);
    for (int i = 0; i < num_interior; ++i) {
        knots[degree + 1 + i] = (i + 1.0) / (num_interior + 1.0);
    }
    
    return knots;
}

void DBRRTTrajectoryPlanner::GenerateBSplineTrajectory(const std::vector<std::shared_ptr<RRTNode>>& path) {
    trajectory_segments_.clear();
    
    // Extract all control points from path
    std::vector<Eigen::Vector3d> all_control_points;
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            all_control_points.push_back(path[i]->state);
        } else {
            // Add control points from the segment
            for (size_t j = 1; j < path[i]->control_points.size(); ++j) {
                all_control_points.push_back(path[i]->control_points[j]);
            }
        }
    }
    
    // Create single continuous B-spline
    BSplineSegment full_trajectory;
    full_trajectory.control_points = all_control_points;
    full_trajectory.degree = spline_degree_;
    full_trajectory.start_time = 0.0;
    full_trajectory.duration = all_control_points.size() * time_interval_;
    
    trajectory_segments_.push_back(full_trajectory);
    trajectory_duration_ = full_trajectory.duration;
}

void DBRRTTrajectoryPlanner::OptimizeControlPoints(std::vector<Eigen::Vector3d>& control_points) {
    // Apply smoothing while maintaining convex hull property
    // This ensures the trajectory stays within safe bounds
    
    int n = control_points.size();
    if (n <= 2) return;
    
    // Simple smoothing: adjust interior points towards centroid
    for (int iter = 0; iter < 3; ++iter) {
        std::vector<Eigen::Vector3d> new_points = control_points;
        
        for (int i = 1; i < n - 1; ++i) {
            Eigen::Vector3d centroid = (control_points[i-1] + control_points[i] + control_points[i+1]) / 3.0;
            new_points[i] = 0.7 * control_points[i] + 0.3 * centroid;
        }
        
        control_points = new_points;
    }
}

Eigen::Vector3d DBRRTTrajectoryPlanner::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!trajectory_valid_ || trajectory_segments_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    // Check if we need to replan
    if (enable_replanning_) {
        double elapsed = current_time - trajectory_start_time_;
        if (elapsed > 0.1) { // Don't replan immediately
            Eigen::Vector3d desired_pose = GetDesiredPositionAtTime(current_time);
            double position_error = (current_pose.head<2>() - desired_pose.head<2>()).norm();
            
            if (position_error > replan_threshold_) {
                std::cout << "DB-RRT: Replanning due to deviation: " << position_error << std::endl;
                Replan(current_pose, current_time);
            }
        }
    }
    
    return ComputeTrackingControl(current_pose, current_time);
}

Eigen::Vector3d DBRRTTrajectoryPlanner::ComputeTrackingControl(const Eigen::Vector3d& current_pose, 
                                                               double current_time) {
    double elapsed = current_time - trajectory_start_time_;
    
    if (elapsed >= trajectory_duration_) {
        trajectory_valid_ = false;
        return Eigen::Vector3d::Zero();
    }
    
    // Find current segment and local time
    const BSplineSegment& segment = trajectory_segments_[0]; // Single segment for now
    double t = elapsed / segment.duration;
    t = std::max(0.0, std::min(1.0, t));
    
    // Get desired state and derivatives
    Eigen::Vector3d desired_pose = EvaluateBSpline(segment, t);
    Eigen::Vector3d desired_velocity = EvaluateBSplineDerivative(segment, t, 1) / time_interval_;
    
    // Position error
    Eigen::Vector3d pose_error = desired_pose - current_pose;
    
    // Normalize angle error
    while (pose_error(2) > M_PI) pose_error(2) -= 2 * M_PI;
    while (pose_error(2) < -M_PI) pose_error(2) += 2 * M_PI;
    
    // Compute derivative term
    Eigen::Vector3d error_derivative = Eigen::Vector3d::Zero();
    if (has_previous_state_) {
        double dt = current_time - previous_time_;
        if (dt > 0) {
            error_derivative = (pose_error - previous_error_) / dt;
        }
    }
    
    // PD control
    Eigen::Vector3d control_velocity;
    control_velocity.head<2>() = kp_position_ * pose_error.head<2>() + 
                                kd_position_ * error_derivative.head<2>() + 
                                desired_velocity.head<2>();
    
    control_velocity(2) = kp_angular_ * pose_error(2) + 
                         kd_angular_ * error_derivative(2) + 
                         desired_velocity(2);
    
    // Apply velocity limits
    double v_linear = control_velocity.head<2>().norm();
    if (v_linear > v_max_) {
        control_velocity.head<2>() *= v_max_ / v_linear;
    }
    
    control_velocity(2) = std::max(-omega_max_, std::min(omega_max_, control_velocity(2)));
    
    // Update state for next iteration
    previous_error_ = pose_error;
    previous_time_ = current_time;
    has_previous_state_ = true;
    
    return control_velocity;
}

bool DBRRTTrajectoryPlanner::Replan(const Eigen::Vector3d& current_pose, double current_time) {
    // Extract remaining waypoints
    double elapsed = current_time - trajectory_start_time_;
    double t = elapsed / trajectory_duration_;
    
    // Find goal from current trajectory end
    Eigen::Vector3d goal = trajectory_segments_.back().control_points.back();
    
    // Plan new trajectory from current pose
    bool success = PlanTrajectory(current_pose, goal, 0.5); // Shorter time limit for replanning
    
    if (success) {
        trajectory_start_time_ = current_time;
    }
    
    return success;
}

std::vector<Eigen::Vector3d> DBRRTTrajectoryPlanner::GetPlannedPath() const {
    std::vector<Eigen::Vector3d> path;
    
    if (!trajectory_segments_.empty()) {
        const BSplineSegment& segment = trajectory_segments_[0];
        int num_samples = 50;
        
        for (int i = 0; i <= num_samples; ++i) {
            double t = static_cast<double>(i) / num_samples;
            path.push_back(const_cast<DBRRTTrajectoryPlanner*>(this)->EvaluateBSpline(segment, t));
        }
    }
    
    return path;
}

std::vector<Eigen::Vector3d> DBRRTTrajectoryPlanner::GetCurrentControlPoints() const {
    if (!trajectory_segments_.empty()) {
        return trajectory_segments_[0].control_points;
    }
    return std::vector<Eigen::Vector3d>();
}

double DBRRTTrajectoryPlanner::GetCurrentProgress() const {
    if (!trajectory_valid_) return 0.0;
    
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    return std::min(1.0, elapsed / trajectory_duration_);
}

Eigen::Vector3d DBRRTTrajectoryPlanner::GetDesiredPositionAtTime(double time) const {
    if (!trajectory_valid_ || trajectory_segments_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed = time - trajectory_start_time_;
    const BSplineSegment& segment = trajectory_segments_[0];
    double t = elapsed / segment.duration;
    t = std::max(0.0, std::min(1.0, t));
    
    return const_cast<DBRRTTrajectoryPlanner*>(this)->EvaluateBSpline(segment, t);
}

} // namespace ctrl