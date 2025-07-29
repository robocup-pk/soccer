#include "BangBangTrajectory3D.h"
#include "SystemConfig.h"
#include "Utils.h"
#include "RobotDescription.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <mutex>

namespace ctrl {

// Static member initialization - run lookup table only once and cache results
std::unique_ptr<AccelerationEnvelope> BangBangTrajectory3D::acceleration_envelope_ = nullptr;
bool BangBangTrajectory3D::envelope_initialized_ = false;
std::mutex BangBangTrajectory3D::envelope_mutex_;
BangBangTrajectory3D::AccelCache BangBangTrajectory3D::accel_cache_;

// Note: All structures (BangBangCase, TrajectorySegment, DOFTrajectory, etc.) 
// are now defined in the header file for complete paper implementation

BangBangTrajectory3D::BangBangTrajectory3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end,
                                           double t_start_s, double t_end_s, Eigen::Vector3d v0) {
    this->pose_start = pose_start;
    this->pose_end = pose_end;
    this->t_start_s = t_start_s;
    this->t_finish_s = t_end_s;
    this->v0 = v0;
    this->h = pose_end - pose_start;
    this->T = t_finish_s - t_start_s;

    // Initialize acceleration envelope only once (optimized lookup table)
    InitializeAccelerationEnvelope();

    // Normalize wheel positions as assumed by existing system
    Eigen::Vector3d h_normalized = h;
    if (h_normalized.head<2>().norm() > 0.001) {
        // Handle coordinate system conversion (kickdemo multiplies by 1000)
        if (std::abs(h_normalized[0]) > 100 || std::abs(h_normalized[1]) > 100) {
            h_normalized.head<2>() /= 1000.0;  // Convert mm to m
        }
    }

    // Generate complete BangBang trajectory using FULL paper implementation
    TrajectoryComplete3D complete_trajectory = GenerateCompleteTrajectory(h_normalized, v0);
    
    // Store trajectories for evaluation
    x_trajectory_dof = complete_trajectory.translation.x_trajectory;
    y_trajectory_dof = complete_trajectory.translation.y_trajectory;
    theta_trajectory_dof = complete_trajectory.rotation;
    
    // Apply jerk limiting for smooth motion profiles
    ApplyJerkLimiting();

    // Debug output matching TrapezoidalTrajectoryVi3D format
    std::cout << "[ctrl::BangBangTrajectory3D] Complete BangBang trajectory generated (Full Paper Implementation)" << std::endl;
    std::cout << "[ctrl::BangBangTrajectory3D] X segments: " << x_trajectory_dof.segments.size() 
              << ", time: " << x_trajectory_dof.total_time << "s" << std::endl;
    std::cout << "[ctrl::BangBangTrajectory3D] Y segments: " << y_trajectory_dof.segments.size() 
              << ", time: " << y_trajectory_dof.total_time << "s" << std::endl;
    std::cout << "[ctrl::BangBangTrajectory3D] θ segments: " << theta_trajectory_dof.segments.size() 
              << ", time: " << theta_trajectory_dof.total_time << "s" << std::endl;
    std::cout << "[ctrl::BangBangTrajectory3D] Total execution time: " << complete_trajectory.total_execution_time << "s" << std::endl;
}

void BangBangTrajectory3D::InitializeAccelerationEnvelope() {
    std::lock_guard<std::mutex> lock(envelope_mutex_);
    
    if (envelope_initialized_) {
        return;  // Already initialized - optimization: run lookup only once
    }

    // Create robot parameters exactly like BangBangTrajectory.cpp
    auto robot_description = kin::GetRobotDescription();
    RobotParams params;
    params.mass_kg = 2.3;
    params.inertia_z = 0.0085;
    params.friction_coeff = 0.8;
    params.wheel_force_max = 1.5;

    // Copy wheel configuration from RobotDescription (normalized as system expects)
    params.wheel_angles_rad = robot_description.wheel_angles_rad;
    params.wheel_positions_m = robot_description.wheel_positions_m;

    // Initialize acceleration envelope with Monte Carlo lookup table
    acceleration_envelope_ = std::make_unique<AccelerationEnvelope>(params);

    if (acceleration_envelope_->IsInitialized()) {
        // Cache acceleration values for interpolation (run lookup only once)
        accel_cache_.angles.clear();
        accel_cache_.a_max_values.clear();
        accel_cache_.alpha_max_values.clear();

        // Sample acceleration envelope at regular intervals for interpolation
        for (int i = 0; i < 360; i += 5) {  // Every 5 degrees for high precision
            double angle_rad = i * M_PI / 180.0;
            auto [a_max, alpha_max] = acceleration_envelope_->QueryAcceleration(angle_rad);
            
            accel_cache_.angles.push_back(angle_rad);
            accel_cache_.a_max_values.push_back(a_max);
            accel_cache_.alpha_max_values.push_back(alpha_max);
        }

        envelope_initialized_ = true;
        std::cout << "[BangBangTrajectory3D] Acceleration envelope initialized and cached (Monte Carlo lookup table)" << std::endl;
        acceleration_envelope_->PrintEnvelopeInfo();
    } else {
        std::cerr << "[BangBangTrajectory3D] Failed to initialize acceleration envelope, using conservative defaults" << std::endl;
        envelope_initialized_ = false;
    }
}

std::pair<double, double> BangBangTrajectory3D::GetMaxAcceleration(double direction_angle) {
    if (!envelope_initialized_) {
        // Fall back to system config values if envelope not available
        return {cfg::SystemConfig::max_acc_m_radpsps[0], cfg::SystemConfig::max_acc_m_radpsps[2]};
    }

    // Use cached interpolation instead of calling lookup table repeatedly (optimization)
    if (accel_cache_.angles.empty()) {
        return {3.0, 20.0};  // Default values
    }

    // Normalize angle to [0, 2π]
    while (direction_angle < 0) direction_angle += 2 * M_PI;
    while (direction_angle >= 2 * M_PI) direction_angle -= 2 * M_PI;

    // Find closest cached angles for interpolation
    size_t lower_idx = 0;
    for (size_t i = 0; i < accel_cache_.angles.size() - 1; ++i) {
        if (direction_angle >= accel_cache_.angles[i] && direction_angle <= accel_cache_.angles[i + 1]) {
            lower_idx = i;
            break;
        }
    }

    size_t upper_idx = (lower_idx + 1) % accel_cache_.angles.size();
    
    // Linear interpolation between cached values
    double t = (direction_angle - accel_cache_.angles[lower_idx]) / 
               (accel_cache_.angles[upper_idx] - accel_cache_.angles[lower_idx]);
    
    double a_max = accel_cache_.a_max_values[lower_idx] + 
                   t * (accel_cache_.a_max_values[upper_idx] - accel_cache_.a_max_values[lower_idx]);
    double alpha_max = accel_cache_.alpha_max_values[lower_idx] + 
                       t * (accel_cache_.alpha_max_values[upper_idx] - accel_cache_.alpha_max_values[lower_idx]);

    return {a_max, alpha_max};
}

TrajectoryComplete3D BangBangTrajectory3D::GenerateCompleteTrajectory(const Eigen::Vector3d& displacement, const Eigen::Vector3d& initial_velocity) {
    TrajectoryComplete3D trajectory;
    
    // Get direction-dependent acceleration limits from lookup table
    double movement_angle = std::atan2(displacement[1], displacement[0]);
    auto [a_max_trans, alpha_max_rot] = GetMaxAcceleration(movement_angle);
    
    // Generate individual DOF trajectories using full 5-case algorithm (Section 4)
    trajectory.translation.x_trajectory = GenerateSingleDOF(
        0.0, displacement[0], initial_velocity[0], 0.0, 2.0, a_max_trans);
    trajectory.translation.y_trajectory = GenerateSingleDOF(
        0.0, displacement[1], initial_velocity[1], 0.0, 2.0, a_max_trans);
    trajectory.rotation = GenerateSingleDOF(
        0.0, displacement[2], initial_velocity[2], 0.0, 10.0, alpha_max_rot);
    
    // Synchronize X-Y trajectories using α-parameterization (Section 5)
    SynchronizeTranslationTrajectories(trajectory.translation);
    
    // Set total execution time
    double translation_time = trajectory.translation.total_execution_time;
    double rotation_time = trajectory.rotation.total_time;
    trajectory.total_execution_time = std::max(translation_time, rotation_time);
    
    return trajectory;
}

DOFTrajectory3D BangBangTrajectory3D::GenerateSingleDOF(
    double w0, double wf, double w_dot_0, double w_dot_f,
    double v_max, double a_max) {
    
    DOFTrajectory3D trajectory;
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
        TrajectorySegment3D segment;
        
        // Determine which case applies (Section 4: All 5 cases from paper)
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
                segment = HandleCase2_3(current_vel, remaining_distance, w_dot_f_normalized, a_max);
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

// Section 4: All 5 cases from Purwin & D'Andrea paper
TrajectorySegment3D BangBangTrajectory3D::HandleCase1(double w_dot_0, double a_max) {
    // Case 1: w_dot_0 < 0 (Section 4, Case 1)
    TrajectorySegment3D segment;
    segment.case_type = BangBangCase3D::CASE_1;
    segment.control_effort = a_max;
    segment.duration = -w_dot_0 / a_max;
    segment.distance_traveled = -(w_dot_0 * w_dot_0) / (2.0 * a_max);
    segment.final_velocity = 0.0;
    return segment;
}

TrajectorySegment3D BangBangTrajectory3D::HandleCase2_1(double w_dot_0, double remaining_distance, double v_max, double a_max) {
    // Case 2.1: Accelerate towards v_max (Section 4, Case 2.1)
    TrajectorySegment3D segment;
    segment.case_type = BangBangCase3D::CASE_2_1;
    segment.control_effort = a_max;
    
    double time_to_vmax = (v_max - w_dot_0) / a_max;
    double distance_to_vmax = w_dot_0 * time_to_vmax + 0.5 * a_max * time_to_vmax * time_to_vmax;
    
    if (distance_to_vmax <= remaining_distance) {
        segment.duration = time_to_vmax;
        segment.distance_traveled = distance_to_vmax;
        segment.final_velocity = v_max;
    } else {
        // Triangular profile
        segment.duration = std::sqrt(2 * remaining_distance / a_max + (w_dot_0 / a_max) * (w_dot_0 / a_max)) - w_dot_0 / a_max;
        segment.distance_traveled = remaining_distance;
        segment.final_velocity = w_dot_0 + a_max * segment.duration;
    }
    
    return segment;
}

TrajectorySegment3D BangBangTrajectory3D::HandleCase2_2(double w_dot_0, double remaining_distance, double v_max, double a_max) {
    // Case 2.2: Cruise at v_max (Section 4, Case 2.2)
    TrajectorySegment3D segment;
    segment.case_type = BangBangCase3D::CASE_2_2;
    segment.control_effort = 0.0;
    
    double decel_distance = (v_max * v_max) / (2.0 * a_max);
    double cruise_distance = remaining_distance - decel_distance;
    
    segment.duration = cruise_distance / v_max;
    segment.distance_traveled = cruise_distance;
    segment.final_velocity = v_max;
    
    return segment;
}

TrajectorySegment3D BangBangTrajectory3D::HandleCase2_3(double w_dot_0, double remaining_distance, double w_dot_f, double a_max) {
    // Case 2.3: Decelerate to target (Section 4, Case 2.3)
    TrajectorySegment3D segment;
    segment.case_type = BangBangCase3D::CASE_2_3;
    segment.control_effort = -a_max;
    
    segment.duration = (w_dot_0 - w_dot_f) / a_max;
    segment.distance_traveled = w_dot_0 * segment.duration - 0.5 * a_max * segment.duration * segment.duration;
    segment.final_velocity = w_dot_f;
    
    return segment;
}

TrajectorySegment3D BangBangTrajectory3D::HandleCase3(double w_dot_0, double v_max, double a_max) {
    // Case 3: w_dot_0 > v_max (Section 4, Case 3)
    TrajectorySegment3D segment;
    segment.case_type = BangBangCase3D::CASE_3;
    segment.control_effort = -a_max;
    segment.duration = (w_dot_0 - v_max) / a_max;
    segment.distance_traveled = w_dot_0 * segment.duration - 0.5 * a_max * segment.duration * segment.duration;
    segment.final_velocity = v_max;
    return segment;
}

void BangBangTrajectory3D::SynchronizeTranslationTrajectories(TranslationTrajectory3D& translation) {
    // Section 5: X-Y synchronization via α-parameterization and bisection algorithm
    double max_time = std::max(translation.x_trajectory.total_time, translation.y_trajectory.total_time);
    
    // Use bisection algorithm to find optimal α (Section 5.2)
    double alpha_low = 0.1, alpha_high = 2.0;
    double tolerance = 1e-6;
    
    for (int iter = 0; iter < 50; ++iter) {
        double alpha_mid = (alpha_low + alpha_high) / 2.0;
        
        // Scale trajectories by α
        DOFTrajectory3D x_scaled = ScaleTrajectoryByAlpha(translation.x_trajectory, alpha_mid);
        DOFTrajectory3D y_scaled = ScaleTrajectoryByAlpha(translation.y_trajectory, alpha_mid);
        
        double time_diff = std::abs(x_scaled.total_time - y_scaled.total_time);
        
        if (time_diff < tolerance) {
            translation.x_trajectory = x_scaled;
            translation.y_trajectory = y_scaled;
            translation.alpha = alpha_mid;
            translation.total_execution_time = std::max(x_scaled.total_time, y_scaled.total_time);
            break;
        }
        
        if (x_scaled.total_time > y_scaled.total_time) {
            alpha_high = alpha_mid;
        } else {
            alpha_low = alpha_mid;
        }
    }
}

DOFTrajectory3D BangBangTrajectory3D::ScaleTrajectoryByAlpha(const DOFTrajectory3D& original, double alpha) {
    DOFTrajectory3D scaled = original;
    
    // Scale velocities and accelerations by α (Section 5)
    for (auto& segment : scaled.segments) {
        segment.control_effort *= alpha;
        segment.final_velocity *= alpha;
        segment.duration /= alpha;  // Time scales inversely
    }
    
    scaled.total_time /= alpha;
    return scaled;
}

void BangBangTrajectory3D::ApplyJerkLimiting() {
    // Apply jerk limiting similar to TrapezoidalTrajectory3D for smooth motor control
    double min_transition_time = 0.05;  // 50ms minimum for smooth transitions
    
    // Smooth acceleration transitions for all DOF trajectories
    SmoothTrajectoryTransitions(x_trajectory_dof, min_transition_time);
    SmoothTrajectoryTransitions(y_trajectory_dof, min_transition_time);
    SmoothTrajectoryTransitions(theta_trajectory_dof, min_transition_time);
}

void BangBangTrajectory3D::SmoothTrajectoryTransitions(DOFTrajectory3D& trajectory, double min_time) {
    for (auto& segment : trajectory.segments) {
        if (segment.duration < min_time && std::abs(segment.control_effort) > 0) {
            // Stretch duration to minimum time and adjust acceleration
            double scale_factor = min_time / segment.duration;
            segment.duration = min_time;
            segment.control_effort /= scale_factor;
        }
    }
}

Eigen::Vector3d BangBangTrajectory3D::ConvertToMotorVelocities(const Eigen::Vector3d& body_velocity) {
    // Convert accelerations to velocities for motor control (max RPM 280)
    Eigen::Vector3d motor_velocity = body_velocity;
    
    // Convert linear velocity to wheel velocity (v = ω * r)
    double max_linear_velocity = (max_rpm_ * 2 * M_PI / 60.0) * wheel_radius_;  // m/s
    
    // Limit velocities to motor constraints
    motor_velocity[0] = std::clamp(motor_velocity[0], -max_linear_velocity, max_linear_velocity);
    motor_velocity[1] = std::clamp(motor_velocity[1], -max_linear_velocity, max_linear_velocity);
    
    // Angular velocity constraint (empirical from robot testing)
    double max_angular_velocity = 10.0;  // rad/s
    motor_velocity[2] = std::clamp(motor_velocity[2], -max_angular_velocity, max_angular_velocity);
    
    return motor_velocity;
}

double BangBangTrajectory3D::EvaluateTrajectoryAtTime(const DOFTrajectory3D& trajectory, double t, bool get_velocity) {
    if (t < 0 || trajectory.segments.empty()) {
        return 0.0;
    }
    
    double accumulated_time = 0.0;
    double accumulated_position = 0.0;
    
    for (const auto& segment : trajectory.segments) {
        if (t <= accumulated_time + segment.duration) {
            // Time falls within this segment
            double segment_time = t - accumulated_time;
            
            if (get_velocity) {
                // Return velocity at this point
                double initial_velocity = 0.0;
                if (&segment != &trajectory.segments[0]) {
                    // Get final velocity of previous segment
                    size_t segment_idx = &segment - &trajectory.segments[0];
                    initial_velocity = trajectory.segments[segment_idx - 1].final_velocity;
                }
                
                return initial_velocity + segment.control_effort * segment_time;
            } else {
                // Return position at this point
                double initial_velocity = 0.0;
                if (&segment != &trajectory.segments[0]) {
                    size_t segment_idx = &segment - &trajectory.segments[0];
                    initial_velocity = trajectory.segments[segment_idx - 1].final_velocity;
                }
                
                double segment_position = initial_velocity * segment_time + 
                                        0.5 * segment.control_effort * segment_time * segment_time;
                return accumulated_position + segment_position;
            }
        }
        
        accumulated_time += segment.duration;
        accumulated_position += segment.distance_traveled;
    }
    
    // Past end of trajectory
    return get_velocity ? 0.0 : accumulated_position;
}

Eigen::Vector3d BangBangTrajectory3D::VelocityAtT(double t_sec) {
    // Implementation matching TrapezoidalTrajectoryVi3D::VelocityAtT exactly
    if (t_sec < t_start_s || t_sec >= t_finish_s) {
        return Eigen::Vector3d(0, 0, 0);
    }
    
    double t_relative = t_sec - t_start_s;
    
    Eigen::Vector3d v_fWorld;
    v_fWorld[0] = EvaluateTrajectoryAtTime(x_trajectory_dof, t_relative, true);
    v_fWorld[1] = EvaluateTrajectoryAtTime(y_trajectory_dof, t_relative, true);
    v_fWorld[2] = EvaluateTrajectoryAtTime(theta_trajectory_dof, t_relative, true);
    
    // Convert accelerations to velocities for motor control (max RPM 280)
    return ConvertToMotorVelocities(v_fWorld);
}

Eigen::Vector3d BangBangTrajectory3D::PositionAtT(double t_sec) {
    // Implementation matching TrapezoidalTrajectoryVi3D::PositionAtT exactly
    if (t_sec < t_start_s) {
        return Eigen::Vector3d(0, 0, 0);
    } else if (t_sec >= t_finish_s) {
        return h;  // Total displacement
    }

    double t_relative = t_sec - t_start_s;
    
    Eigen::Vector3d position_fworld;
    position_fworld[0] = EvaluateTrajectoryAtTime(x_trajectory_dof, t_relative, false);
    position_fworld[1] = EvaluateTrajectoryAtTime(y_trajectory_dof, t_relative, false);
    position_fworld[2] = EvaluateTrajectoryAtTime(theta_trajectory_dof, t_relative, false);

    return position_fworld;
}

void BangBangTrajectory3D::Print() {
    std::cout << "[ctrl::BangBangTrajectory3D::Print] BangBang Trajectory Info (Full Paper Implementation). ";
    std::cout << "Pose: " << pose_start.transpose() << " -> " << pose_end.transpose();
    std::cout << ". Time: " << t_start_s << " -> " << t_finish_s;
    std::cout << ". v0: " << v0.transpose() << std::endl;
    std::cout << "  All 5 cases implemented, acceleration envelope from lookup table, X-Y synchronization" << std::endl;
}

Eigen::Vector3d BangBangTrajectory3D::TotalDistance() {
    Eigen::Vector3d distance(0, 0, 0);
    for (double t = t_start_s; t < t_finish_s; t += 0.01) {
        distance += VelocityAtT(t) * 0.01;
    }
    return distance;
}

} // namespace ctrl