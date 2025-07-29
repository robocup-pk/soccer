#include "BangBangTrajectory.h"
#include "RobotManager.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

namespace ctrl {

BangBangTrajectory::BangBangTrajectory() 
    : robot_description_(kin::GetRobotDescription()),
      trajectory_start_time_(0.0),
      is_trajectory_active_(false),
      is_trajectory_finished_(true) {
    
    InitializeAccelerationEnvelope();
    ResetTrajectory();
    
    std::cout << "[BangBangTrajectory] Initialized with " 
              << robot_description_.wheel_angles_rad.size() << " wheels" << std::endl;
}

void BangBangTrajectory::InitializeAccelerationEnvelope() {
    // Create RobotParams from RobotDescription
    RobotParams params;
    params.mass_kg = 2.3;
    params.inertia_z = 0.0085;
    params.friction_coeff = 0.8;
    params.wheel_force_max = 1.5;
    
    // Copy wheel configuration
    params.wheel_angles_rad = robot_description_.wheel_angles_rad;
    params.wheel_positions_m = robot_description_.wheel_positions_m;
    
    // Initialize acceleration envelope with Monte Carlo lookup table
    acceleration_envelope_ = std::make_unique<AccelerationEnvelope>(params);
    
    if (acceleration_envelope_->IsInitialized()) {
        alpha_max_ = acceleration_envelope_->GetMaxAngularAcceleration();
        std::cout << "[BangBangTrajectory] Acceleration envelope initialized" << std::endl;
        acceleration_envelope_->PrintEnvelopeInfo();
    } else {
        std::cerr << "[BangBangTrajectory] Warning: Failed to initialize acceleration envelope, using defaults" << std::endl;
    }
}

bool BangBangTrajectory::SetPath(const std::vector<Eigen::Vector3d>& path_fWorld, double t_start_s) {
    if (path_fWorld.size() < 2) {
        std::cerr << "[BangBangTrajectory::SetPath] Error: Path must have at least 2 waypoints" << std::endl;
        return false;
    }
    
    std::cout << "[BangBangTrajectory::SetPath] Generating trajectory for " << path_fWorld.size() << " waypoints" << std::endl;
    
    // Print the path
    for (size_t i = 0; i < path_fWorld.size() - 1; ++i) {
        std::cout << "  " << path_fWorld[i].transpose() << " -> ";
    }
    std::cout << path_fWorld.back().transpose() << std::endl;
    
    try {
        // Generate complete trajectory through all waypoints
        auto trajectories = GenerateWaypointTrajectory(path_fWorld);
        
        if (trajectories.empty()) {
            std::cerr << "[BangBangTrajectory::SetPath] Error: Failed to generate trajectory" << std::endl;
            return false;
        }
        
        // For now, use the first trajectory segment
        // In full implementation, we'd chain all segments
        current_trajectory_ = trajectories[0];
        remaining_waypoints_.assign(path_fWorld.begin() + 2, path_fWorld.end());
        
        trajectory_start_time_ = (t_start_s > 0) ? t_start_s : util::GetCurrentTime();
        is_trajectory_active_ = true;
        is_trajectory_finished_ = false;
        
        std::cout << "[BangBangTrajectory::SetPath] Trajectory generated successfully. Duration: " 
                  << current_trajectory_.total_execution_time << "s" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[BangBangTrajectory::SetPath] Exception: " << e.what() << std::endl;
        return false;
    }
}

bool BangBangTrajectory::AddGoal(const Eigen::Vector3d& goal) {
    std::cout << "[BangBangTrajectory::AddGoal] Adding goal: " << goal.transpose() << std::endl;
    
    if (!is_trajectory_active_) {
        // Start new trajectory from current position to goal
        std::vector<Eigen::Vector3d> path = {Eigen::Vector3d::Zero(), goal};
        return SetPath(path);
    } else {
        // Add to remaining waypoints
        remaining_waypoints_.push_back(goal);
        std::cout << "[BangBangTrajectory::AddGoal] Goal queued. Queue size: " << remaining_waypoints_.size() << std::endl;
        return true;
    }
}

Eigen::Vector3d BangBangTrajectory::Update(const Eigen::Vector3d& current_pose, double current_time) {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return Eigen::Vector3d::Zero();
    }
    
    double elapsed_time = current_time - trajectory_start_time_;
    
    // Check if current trajectory is finished
    if (elapsed_time >= current_trajectory_.total_execution_time) {
        if (!remaining_waypoints_.empty()) {
            // Start next trajectory segment
            std::vector<Eigen::Vector3d> next_path = {current_pose, remaining_waypoints_[0]};
            remaining_waypoints_.erase(remaining_waypoints_.begin());
            SetPath(next_path, current_time);
            elapsed_time = 0.0;
        } else {
            // All trajectories completed
            is_trajectory_finished_ = true;
            is_trajectory_active_ = false;
            std::cout << "[BangBangTrajectory::Update] Trajectory execution completed" << std::endl;
            return Eigen::Vector3d::Zero();
        }
    }
    
    // Evaluate trajectory at current time
    auto [position, velocity, acceleration] = EvaluateTrajectory(current_trajectory_, elapsed_time);
    
    return velocity; // Return velocity command for MotionController
}

double BangBangTrajectory::GetRemainingTime() const {
    if (!is_trajectory_active_ || is_trajectory_finished_) {
        return 0.0;
    }
    
    double elapsed = util::GetCurrentTime() - trajectory_start_time_;
    return std::max(0.0, current_trajectory_.total_execution_time - elapsed);
}

TrajectoryComplete BangBangTrajectory::GenerateTrajectory(
    const TrajectoryState& current_state,
    const TrajectoryState& target_state) {
    
    TrajectoryComplete result;
    
    // Extract states
    double x0 = current_state.position[0];
    double y0 = current_state.position[1];
    double theta0 = current_state.position[2];
    double x_dot_0 = current_state.velocity[0];
    double y_dot_0 = current_state.velocity[1];
    double theta_dot_0 = current_state.velocity[2];
    
    double xf = target_state.position[0] - x0;  // Relative displacement
    double yf = target_state.position[1] - y0;
    double thetaf = NormalizeAngle(target_state.position[2] - theta0);
    
    // Use target constraints or defaults
    double v_max = (current_state.v_max > 0) ? current_state.v_max : v_max_;
    double a_max = (current_state.a_max > 0) ? current_state.a_max : a_max_;
    double omega_max = (current_state.theta_dot_max > 0) ? current_state.theta_dot_max : omega_max_;
    double alpha_max = (current_state.theta_ddot_max > 0) ? current_state.theta_ddot_max : alpha_max_;
    
    // Generate X and Y trajectories independently first (Section 4)
    DOFTrajectory x_traj = GenerateSingleDOF(0.0, xf, x_dot_0, 0.0, v_max, a_max);
    DOFTrajectory y_traj = GenerateSingleDOF(0.0, yf, y_dot_0, 0.0, v_max, a_max);
    
    // Synchronize X-Y trajectories using bisection (Section 5)
    result.translation = SynchronizeXY(x_traj, y_traj, a_max);
    
    // Generate rotation trajectory (Section 7)
    result.rotation = GenerateRotationTrajectory(0.0, thetaf, theta_dot_0, 0.0, omega_max, alpha_max);
    
    // Set total execution time
    result.total_execution_time = std::max(
        result.translation.synchronized_time,
        result.rotation.total_time
    );
    
    return result;
}

std::vector<TrajectoryComplete> BangBangTrajectory::GenerateWaypointTrajectory(
    const std::vector<Eigen::Vector3d>& waypoints,
    const Eigen::Vector3d& initial_velocity) {
    
    std::vector<TrajectoryComplete> trajectories;
    
    if (waypoints.size() < 2) {
        return trajectories;
    }
    
    Eigen::Vector3d current_velocity = initial_velocity;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        TrajectoryState current_state;
        current_state.position = waypoints[i];
        current_state.velocity = current_velocity;
        current_state.v_max = v_max_;
        current_state.a_max = a_max_;
        current_state.theta_dot_max = omega_max_;
        current_state.theta_ddot_max = alpha_max_;
        
        TrajectoryState target_state;
        target_state.position = waypoints[i + 1];
        target_state.target_velocity = Eigen::Vector3d::Zero(); // Stop at each waypoint
        
        TrajectoryComplete trajectory = GenerateTrajectory(current_state, target_state);
        trajectories.push_back(trajectory);
        
        // Next trajectory starts with zero velocity (stopped at waypoint)
        current_velocity = Eigen::Vector3d::Zero();
    }
    
    return trajectories;
}

DOFTrajectory BangBangTrajectory::GenerateSingleDOF(
    double w0, double wf, double w_dot_0, double w_dot_f,
    double v_max, double a_max) {
    
    DOFTrajectory trajectory;
    trajectory.total_time = 0.0;
    trajectory.total_distance = 0.0;
    
    // Normalize problem: wf >= 0 (Section 4 from paper)
    double displacement = wf - w0;
    double sign = (displacement >= 0) ? 1.0 : -1.0;
    double w_normalized = std::abs(displacement);
    double w_dot_0_normalized = sign * w_dot_0;
    double w_dot_f_normalized = sign * w_dot_f;
    
    double current_pos = 0.0;
    double current_vel = w_dot_0_normalized;
    
    // Iterate through cases until destination reached (Section 4)
    while (std::abs(current_pos - w_normalized) > 1e-6 && trajectory.segments.size() < 10) {
        TrajectorySegment segment;
        
        // Determine which case applies (Section 4 cases)
        if (current_vel < 0) {
            // Case 1: Moving away from destination
            segment = HandleCase1(current_vel, a_max);
        } else if (current_vel > v_max) {
            // Case 3: Moving too fast
            segment = HandleCase3(current_vel, v_max, a_max);
        } else {
            // Cases 2.1, 2.2, 2.3: Normal operation
            double remaining_distance = w_normalized - current_pos;
            double decel_distance = (current_vel * current_vel) / (2.0 * a_max);
            
            if (remaining_distance > decel_distance && current_vel < v_max) {
                // Case 2.1: Can accelerate
                segment = HandleCase2_1(current_vel, remaining_distance, v_max, a_max);
            } else if (current_vel >= v_max && remaining_distance > decel_distance) {
                // Case 2.2: Cruise at max velocity
                segment = HandleCase2_2(current_vel, remaining_distance, v_max, a_max);
            } else {
                // Case 2.3: Must decelerate
                segment = HandleCase2_3(current_vel, remaining_distance, v_max, a_max);
            }
        }
        
        // Update state
        current_pos += segment.distance_traveled;
        current_vel = segment.final_velocity;
        trajectory.total_time += segment.duration;
        trajectory.segments.push_back(segment);
    }
    
    trajectory.total_distance = current_pos * sign;
    
    // Convert back to original coordinate system
    if (sign < 0) {
        for (auto& seg : trajectory.segments) {
            seg.control_effort *= -1;
            seg.final_velocity *= -1;
        }
    }
    
    return trajectory;
}

TrajectorySegment BangBangTrajectory::HandleCase1(double w_dot_0, double a_max) {
    // Case 1: w_dot_0 < 0 (Section 4, Case 1)
    TrajectorySegment segment;
    segment.case_type = BangBangCase::CASE_1;
    segment.control_effort = a_max;
    segment.duration = -w_dot_0 / a_max;
    segment.distance_traveled = -(w_dot_0 * w_dot_0) / (2.0 * a_max);
    segment.final_velocity = 0.0;
    return segment;
}

TrajectorySegment BangBangTrajectory::HandleCase2_1(
    double w_dot_0, double remaining_distance, double v_max, double a_max) {
    // Case 2.1: Accelerate (Section 4, Case 2.1)
    TrajectorySegment segment;
    segment.case_type = BangBangCase::CASE_2_1;
    segment.control_effort = a_max;
    
    // Check if we can reach v_max before needing to decelerate
    double w1 = std::sqrt(remaining_distance * a_max + (w_dot_0 * w_dot_0) / 2.0);
    
    if (w1 <= v_max) {
        // Subcase II: Accelerate then immediately decelerate
        segment.duration = (w1 - w_dot_0) / a_max;
        segment.distance_traveled = (w1 * w1 - w_dot_0 * w_dot_0) / (2.0 * a_max);
        segment.final_velocity = w1;
    } else {
        // Subcase I: Accelerate to v_max
        segment.duration = (v_max - w_dot_0) / a_max;
        segment.distance_traveled = (v_max * v_max - w_dot_0 * w_dot_0) / (2.0 * a_max);
        segment.final_velocity = v_max;
    }
    
    return segment;
}

TrajectorySegment BangBangTrajectory::HandleCase2_2(
    double w_dot_0, double remaining_distance, double v_max, double a_max) {
    // Case 2.2: Cruise at maximum velocity (Section 4, Case 2.2)
    TrajectorySegment segment;
    segment.case_type = BangBangCase::CASE_2_2;
    segment.control_effort = 0.0;
    
    double decel_distance = (v_max * v_max) / (2.0 * a_max);
    double cruise_distance = remaining_distance - decel_distance;
    
    segment.duration = cruise_distance / v_max;
    segment.distance_traveled = cruise_distance;
    segment.final_velocity = v_max;
    
    return segment;
}

TrajectorySegment BangBangTrajectory::HandleCase2_3(
    double w_dot_0, double remaining_distance, double v_max, double a_max) {
    // Case 2.3: Decelerate to destination (Section 4, Case 2.3)
    TrajectorySegment segment;
    segment.case_type = BangBangCase::CASE_2_3;
    segment.control_effort = -a_max;
    segment.duration = w_dot_0 / a_max;
    segment.distance_traveled = (w_dot_0 * w_dot_0) / (2.0 * a_max);
    segment.final_velocity = 0.0;
    
    return segment;
}

TrajectorySegment BangBangTrajectory::HandleCase3(double w_dot_0, double v_max, double a_max) {
    // Case 3: w_dot_0 > v_max (Section 4, Case 3)
    TrajectorySegment segment;
    segment.case_type = BangBangCase::CASE_3;
    segment.control_effort = -a_max;
    segment.duration = (w_dot_0 - v_max) / a_max;
    segment.distance_traveled = (w_dot_0 * w_dot_0 - v_max * v_max) / (2.0 * a_max);
    segment.final_velocity = v_max;
    
    return segment;
}

Trajectory2D BangBangTrajectory::SynchronizeXY(
    const DOFTrajectory& x_traj, 
    const DOFTrajectory& y_traj,
    double a_max_total) {
    // Section 5: Synchronization via α-parameterization
    Trajectory2D result;
    
    double tf_x = x_traj.total_time;
    double tf_y = y_traj.total_time;
    
    if (std::abs(tf_x - tf_y) < 1e-6) {
        // Already synchronized
        result.x_trajectory = x_traj;
        result.y_trajectory = y_traj;
        result.synchronized_time = tf_x;
        result.alpha_sync = M_PI / 4.0; // 45 degrees
        return result;
    }
    
    // Use bisection to find optimal α (Section 5)
    double alpha_min = 1e-6;
    double alpha_max = M_PI / 2.0 - 1e-6;
    double alpha_opt = M_PI / 4.0;
    
    const int max_iterations = 20;
    const double tolerance = 1e-4;
    
    for (int i = 0; i < max_iterations; ++i) {
        // Compute accelerations for current α
        double ax_max = a_max_total * std::cos(alpha_opt);
        double ay_max = a_max_total * std::sin(alpha_opt);
        double vx_max = v_max_ * std::cos(alpha_opt);
        double vy_max = v_max_ * std::sin(alpha_opt);
        
        // Estimate execution times with scaled parameters
        double time_scale_x = std::sqrt(a_max_total / ax_max);
        double time_scale_y = std::sqrt(a_max_total / ay_max);
        
        double tf_x_scaled = tf_x * time_scale_x;
        double tf_y_scaled = tf_y * time_scale_y;
        
        double time_diff = tf_x_scaled - tf_y_scaled;
        
        if (std::abs(time_diff) < tolerance) {
            break;
        }
        
        if (time_diff > 0) {
            // X takes longer, increase α (more acceleration for Y)
            alpha_min = alpha_opt;
        } else {
            // Y takes longer, decrease α (more acceleration for X)
            alpha_max = alpha_opt;  
        }
        
        alpha_opt = (alpha_min + alpha_max) / 2.0;
    }
    
    result.alpha_sync = alpha_opt;
    result.synchronized_time = std::max(tf_x, tf_y);
    result.x_trajectory = x_traj;
    result.y_trajectory = y_traj;
    
    return result;
}

DOFTrajectory BangBangTrajectory::GenerateRotationTrajectory(
    double theta_0, double theta_f, double theta_dot_0, double theta_dot_f,
    double theta_dot_max, double theta_ddot_max) {
    // Section 7: Rotation trajectory generation
    
    // Normalize angle difference
    double theta_diff = NormalizeAngle(theta_f - theta_0);
    
    // Use same bang-bang logic as linear case
    return GenerateSingleDOF(
        0.0, theta_diff, theta_dot_0, theta_dot_f,
        theta_dot_max, theta_ddot_max
    );
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> 
BangBangTrajectory::EvaluateTrajectory(const TrajectoryComplete& trajectory, double time) {
    
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    
    // Helper function to evaluate single DOF trajectory
    auto evaluateDOF = [](const DOFTrajectory& traj, double t) -> std::tuple<double, double, double> {
        double pos = 0.0, vel = 0.0, acc = 0.0;
        double t_remaining = t;
        double initial_vel = 0.0;
        
        for (const auto& segment : traj.segments) {
            if (t_remaining <= segment.duration) {
                // Current segment
                acc = segment.control_effort;
                vel = initial_vel + acc * t_remaining;
                pos += initial_vel * t_remaining + 0.5 * acc * t_remaining * t_remaining;
                break;
            } else {
                // Complete segment
                pos += segment.distance_traveled;
                t_remaining -= segment.duration;
                initial_vel = segment.final_velocity;
            }
        }
        
        return std::make_tuple(pos, vel, acc);
    };
    
    // Evaluate X trajectory
    auto [x_pos, x_vel, x_acc] = evaluateDOF(trajectory.translation.x_trajectory, time);
    
    // Evaluate Y trajectory  
    auto [y_pos, y_vel, y_acc] = evaluateDOF(trajectory.translation.y_trajectory, time);
    
    // Evaluate theta trajectory
    auto [theta_pos, theta_vel, theta_acc] = evaluateDOF(trajectory.rotation, time);
    
    position << x_pos, y_pos, theta_pos;
    velocity << x_vel, y_vel, theta_vel;
    acceleration << x_acc, y_acc, theta_acc;
    
    return std::make_tuple(position, velocity, acceleration);
}

Eigen::Vector3d BangBangTrajectory::ComputeMaxAcceleration(double direction_rad) {
    if (acceleration_envelope_ && acceleration_envelope_->IsInitialized()) {
        return acceleration_envelope_->ComputeMaxAcceleration(direction_rad);
    } else {
        // Fallback to default values
        return Eigen::Vector3d(a_max_ * std::cos(direction_rad), a_max_ * std::sin(direction_rad), alpha_max_);
    }
}

bool BangBangTrajectory::IsAccelerationFeasible(const Eigen::Vector3d& acceleration) {
    if (acceleration_envelope_ && acceleration_envelope_->IsInitialized()) {
        return acceleration_envelope_->IsAccelerationFeasible(acceleration);
    } else {
        // Fallback to simple circular constraint
        double linear_mag = std::sqrt(acceleration[0]*acceleration[0] + acceleration[1]*acceleration[1]);
        return linear_mag <= a_max_ && std::abs(acceleration[2]) <= alpha_max_;
    }
}

void BangBangTrajectory::SetLimits(double v_max, double a_max, double omega_max, double alpha_max) {
    v_max_ = v_max;
    a_max_ = a_max;
    omega_max_ = omega_max;
    alpha_max_ = alpha_max;
    
    std::cout << "[BangBangTrajectory] Updated limits: v_max=" << v_max_ 
              << ", a_max=" << a_max_ << ", ω_max=" << omega_max_ 
              << ", α_max=" << alpha_max_ << std::endl;
}

void BangBangTrajectory::PrintAccelerationEnvelope() const {
    if (acceleration_envelope_ && acceleration_envelope_->IsInitialized()) {
        acceleration_envelope_->PrintEnvelopeInfo();
    } else {
        std::cout << "Acceleration envelope not initialized" << std::endl;
    }
}

void BangBangTrajectory::InitializeFromRobotManager(rob::RobotManager* robot_manager) {
    if (!robot_manager) {
        std::cerr << "[BangBangTrajectory] Error: RobotManager is null" << std::endl;
        return;
    }
    
    // Get current robot state
    Eigen::Vector3d current_pose = robot_manager->GetPoseInWorldFrame();
    Eigen::Vector3d current_velocity = robot_manager->GetVelocityInWorldFrame();
    
    std::cout << "[BangBangTrajectory] Initialized from RobotManager:" << std::endl;
    std::cout << "  Current pose: " << current_pose.transpose() << std::endl;
    std::cout << "  Current velocity: " << current_velocity.transpose() << std::endl;
}

double BangBangTrajectory::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool BangBangTrajectory::IsTrajectoryValid(const TrajectoryComplete& trajectory) {
    return trajectory.total_execution_time > 0 && 
           !trajectory.translation.x_trajectory.segments.empty() &&
           !trajectory.translation.y_trajectory.segments.empty();
}

void BangBangTrajectory::ResetTrajectory() {
    is_trajectory_active_ = false;
    is_trajectory_finished_ = true;
    trajectory_start_time_ = 0.0;
    remaining_waypoints_.clear();
    current_trajectory_ = TrajectoryComplete{};
}

TrajectoryState BangBangTrajectory::GetCurrentStateFromPose(
    const Eigen::Vector3d& pose, const Eigen::Vector3d& velocity) {
    
    TrajectoryState state;
    state.position = pose;
    state.velocity = velocity;
    state.target_velocity = Eigen::Vector3d::Zero();
    state.v_max = v_max_;
    state.a_max = a_max_;
    state.theta_dot_max = omega_max_;
    state.theta_ddot_max = alpha_max_;
    
    return state;
}

TrajectoryState BangBangTrajectory::CreateTargetState(const Eigen::Vector3d& target_pose) {
    TrajectoryState state;
    state.position = target_pose;
    state.velocity = Eigen::Vector3d::Zero();
    state.target_velocity = Eigen::Vector3d::Zero();
    state.v_max = v_max_;
    state.a_max = a_max_;
    state.theta_dot_max = omega_max_;
    state.theta_ddot_max = alpha_max_;
    
    return state;
}

double BangBangTrajectory::FindOptimalAlpha(
    double xf, double yf, double x_dot_0, double y_dot_0,
    double v_max, double a_max) {
    
    // Bisection algorithm implementation for finding optimal α (Section 5)
    double alpha_min = 1e-6;
    double alpha_max = M_PI / 2.0 - 1e-6;
    
    const int max_iterations = 20;
    const double tolerance = 1e-4;
    
    for (int i = 0; i < max_iterations; ++i) {
        double alpha = (alpha_min + alpha_max) / 2.0;
        
        double ax_max = a_max * std::cos(alpha);
        double ay_max = a_max * std::sin(alpha);
        double vx_max = v_max * std::cos(alpha);
        double vy_max = v_max * std::sin(alpha);
        
        DOFTrajectory x_traj = GenerateSingleDOF(0, xf, x_dot_0, 0, vx_max, ax_max);
        DOFTrajectory y_traj = GenerateSingleDOF(0, yf, y_dot_0, 0, vy_max, ay_max);
        
        double time_diff = x_traj.total_time - y_traj.total_time;
        
        if (std::abs(time_diff) < tolerance) {
            return alpha;
        }
        
        if (time_diff > 0) {
            alpha_min = alpha;
        } else {
            alpha_max = alpha;
        }
    }
    
    return (alpha_min + alpha_max) / 2.0;
}

} // namespace ctrl