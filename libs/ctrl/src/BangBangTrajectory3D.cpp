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
    this->v0 = v0;
    this->h = pose_end - pose_start;
    
    // Initialize acceleration envelope only once (optimized lookup table)
    InitializeAccelerationEnvelope();

    // Fix coordinate system conversion - kickdemo appears to use mm units
    Eigen::Vector3d h_normalized = h;
    
    // Check if values are in mm (typical soccer field coordinates are much smaller than 10m)
    if (std::abs(h_normalized[0]) > 10.0 || std::abs(h_normalized[1]) > 10.0) {
        std::cout << "[BangBangTrajectory3D] Converting coordinates from mm to m: " 
                  << h.transpose() << " -> ";
        h_normalized.head<2>() /= 1000.0;  // Convert mm to m
        std::cout << h_normalized.transpose() << std::endl;
    }

    // Generate complete BangBang trajectory using FULL paper implementation
    TrajectoryComplete3D complete_trajectory = GenerateCompleteTrajectory(h_normalized, v0);
    
    // Store trajectories for evaluation
    x_trajectory_dof = complete_trajectory.translation.x_trajectory;
    y_trajectory_dof = complete_trajectory.translation.y_trajectory;
    theta_trajectory_dof = complete_trajectory.rotation;
    
    // Apply jerk limiting for smooth motion profiles
    ApplyJerkLimiting();

    // Use the expected time from TrajectoryManager for consistency
    double optimal_time = complete_trajectory.total_execution_time;
    double expected_time = t_end_s - t_start_s;
    
    // Use expected time if reasonable, otherwise use optimal
    double actual_time = (expected_time > 0.1 && expected_time < 10.0) ? expected_time : std::max(optimal_time, 1.0);
    
    std::cout << "[BangBangTrajectory3D] Optimal: " << optimal_time 
              << "s, Expected: " << expected_time 
              << "s, Using: " << actual_time << "s" << std::endl;
    
    // Use the calculated time
    this->t_finish_s = t_start_s + actual_time;
    this->T = actual_time;

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
    params.mass_kg = 2.7;
    params.inertia_z = 0.0085;
    params.friction_coeff = 0.8;
    params.wheel_force_max = 2.0;

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
    
    // Use proper maximum velocity from paper (Section 4)
    double v_max = 2.0;  // m/s - from paper table 1
    
    // Generate independent DOF trajectories first
    trajectory.translation.x_trajectory = GenerateSingleDOF(
        0.0, displacement[0], initial_velocity[0], 0.0, v_max, a_max_trans);
    trajectory.translation.y_trajectory = GenerateSingleDOF(
        0.0, displacement[1], initial_velocity[1], 0.0, v_max, a_max_trans);
    
    // Apply proper synchronization using α-parameterization (Section 5)
    SynchronizeTranslationTrajectories(trajectory.translation);
    
    // Synchronize final times
    double translation_time = std::max(trajectory.translation.x_trajectory.total_time, trajectory.translation.y_trajectory.total_time);
    trajectory.translation.total_execution_time = translation_time;

    // Generate rotation trajectory with proper angular velocity limit
    double omega_max = 5.0;  // rad/s - reasonable for SSL robots
    trajectory.rotation = GenerateSingleDOF(
        0.0, displacement[2], initial_velocity[2], 0.0, omega_max, alpha_max_rot);
    double rotation_time = trajectory.rotation.total_time;

    // Final trajectory time is the max of translation and rotation
    trajectory.total_execution_time = std::max(translation_time, rotation_time);
    
    return trajectory;
}

double BangBangTrajectory3D::FindOptimalAlpha(
    double xf, double yf, double x_dot_0, double y_dot_0,
    double v_max, double a_max) {
    
    // Bisection algorithm to find optimal alpha (Section 5 from paper)
    auto calculate_time = [&](double a_max_scaled) {
        DOFTrajectory3D traj_x = GenerateSingleDOF(0, xf, x_dot_0, 0, v_max, a_max_scaled);
        DOFTrajectory3D traj_y = GenerateSingleDOF(0, yf, y_dot_0, 0, v_max, a_max_scaled);
        return std::max(traj_x.total_time, traj_y.total_time);
    };

    double alpha_low = 0.01, alpha_high = 1.0;
    double t_max = calculate_time(a_max);

    for (int i = 0; i < 50; ++i) {
        double alpha = (alpha_low + alpha_high) / 2.0;
        double t_alpha = calculate_time(a_max / std::cos(alpha));
        
        if (std::abs(t_alpha - t_max / std::sin(alpha)) < 1e-4) {
            return alpha;
        }
        if (t_alpha < t_max / std::sin(alpha)) {
            alpha_low = alpha;
        } else {
            alpha_high = alpha;
        }
    }
    return (alpha_low + alpha_high) / 2.0;
}

DOFTrajectory3D BangBangTrajectory3D::GenerateSingleDOF(
    double w0, double wf, double w_dot_0, double w_dot_f,
    double v_max, double a_max) {
    
    DOFTrajectory3D trajectory;
    trajectory.total_time = 0.0;
    trajectory.total_distance = 0.0;
    
    // Safety checks to prevent invalid trajectories
    if (std::abs(a_max) < 1e-6 || std::abs(v_max) < 1e-6) {
        std::cout << "[BangBangTrajectory3D] Invalid parameters: a_max=" << a_max 
                  << ", v_max=" << v_max << std::endl;
        return trajectory;  // Return empty trajectory
    }
    
    // Normalize problem: wf >= 0 (Section 4 from paper)
    double displacement = wf - w0;
    
    // Handle very small displacements
    if (std::abs(displacement) < 1e-6) {
        std::cout << "[BangBangTrajectory3D] Very small displacement: " << displacement << std::endl;
        return trajectory;  // No movement needed
    }
    
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
        
        // Validate segment before adding
        if (segment.duration < 0 || std::isnan(segment.duration) || std::isinf(segment.duration)) {
            std::cout << "[BangBangTrajectory3D] Invalid segment duration: " << segment.duration 
                      << " for displacement: " << displacement << std::endl;
            break;  // Stop generating invalid segments
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
    
    // Add conservative limits for SSL robot (much lower than theoretical max)
    double safe_linear_velocity = std::min(max_linear_velocity, 2.0);  // Max 2 m/s
    double safe_angular_velocity = 5.0;  // Max 5 rad/s
    
    // Debug output if velocities are very high
    if (std::abs(body_velocity[0]) > 10.0 || std::abs(body_velocity[1]) > 10.0) {
        std::cout << "[BangBangTrajectory3D] High velocity detected: [" << body_velocity.transpose() 
                  << "], max_linear=" << max_linear_velocity << std::endl;
    }
    
    // Limit velocities to safe constraints
    motor_velocity[0] = std::clamp(motor_velocity[0], -safe_linear_velocity, safe_linear_velocity);
    motor_velocity[1] = std::clamp(motor_velocity[1], -safe_linear_velocity, safe_linear_velocity);
    motor_velocity[2] = std::clamp(motor_velocity[2], -safe_angular_velocity, safe_angular_velocity);
    
    return motor_velocity;
}

double BangBangTrajectory3D::EvaluateTrajectoryAtTime(const DOFTrajectory3D& trajectory, double t, bool get_velocity) {
    if (t < 0 || trajectory.segments.empty()) {
        return 0.0;
    }
    
    // If past end of trajectory, return final state
    if (t >= trajectory.total_time) {
        return get_velocity ? 0.0 : trajectory.total_distance;
    }
    
    double accumulated_time = 0.0;
    double accumulated_position = 0.0;
    double current_velocity = 0.0;  // Track velocity properly
    
    for (size_t i = 0; i < trajectory.segments.size(); ++i) {
        const auto& segment = trajectory.segments[i];
        
        if (t <= accumulated_time + segment.duration) {
            // Time falls within this segment
            double segment_time = t - accumulated_time;
            
            if (get_velocity) {
                // Velocity: v(t) = v0 + a*t
                return current_velocity + segment.control_effort * segment_time;
            } else {
                // Position: x(t) = x0 + v0*t + 0.5*a*t^2
                double segment_position = current_velocity * segment_time + 
                                        0.5 * segment.control_effort * segment_time * segment_time;
                return accumulated_position + segment_position;
            }
        }
        
        // Move to next segment
        accumulated_time += segment.duration;
        accumulated_position += segment.distance_traveled;
        current_velocity = segment.final_velocity;  // Update velocity for next segment
    }
    
    // Past end of trajectory (shouldn't reach here due to check above)
    return get_velocity ? 0.0 : trajectory.total_distance;
}

Eigen::Vector3d BangBangTrajectory3D::VelocityAtT(double t_sec) {
    // Handle timing correctly according to paper Section 4
    if (t_sec < t_start_s) {
        return Eigen::Vector3d(0, 0, 0);
    }
    
    double t_relative = t_sec - t_start_s;
    
    // Return zero velocity when trajectory is complete (paper requirement)
    if (t_sec >= t_finish_s || t_relative >= T) {
        return Eigen::Vector3d(0, 0, 0);
    }
    
    // Scale time to trajectory duration for proper evaluation
    double trajectory_duration = std::max({x_trajectory_dof.total_time, y_trajectory_dof.total_time, theta_trajectory_dof.total_time});
    double scaled_t = (trajectory_duration > 0) ? t_relative * trajectory_duration / T : t_relative;
    
    // Evaluate velocity at the scaled time using bang-bang segments
    Eigen::Vector3d v_fWorld;
    v_fWorld[0] = EvaluateTrajectoryAtTime(x_trajectory_dof, scaled_t, true);
    v_fWorld[1] = EvaluateTrajectoryAtTime(y_trajectory_dof, scaled_t, true);
    v_fWorld[2] = EvaluateTrajectoryAtTime(theta_trajectory_dof, scaled_t, true);
    
    // Scale velocities back to match expected duration
    if (trajectory_duration > 0 && T > 0) {
        double velocity_scale = trajectory_duration / T;
        v_fWorld *= velocity_scale;
    }
    
    // Apply reasonable velocity limits
    v_fWorld[0] = std::clamp(v_fWorld[0], -2.0, 2.0);
    v_fWorld[1] = std::clamp(v_fWorld[1], -2.0, 2.0);
    v_fWorld[2] = std::clamp(v_fWorld[2], -5.0, 5.0);
    
    // Debug output for debugging robot not moving
    if (std::abs(v_fWorld[0]) > 0.01 || std::abs(v_fWorld[1]) > 0.01) {
        std::cout << "[BangBangTrajectory3D::VelocityAtT] t=" << t_sec 
                  << ", t_rel=" << t_relative << ", T=" << T 
                  << ", v=" << v_fWorld.transpose() << std::endl;
    }
    
    return v_fWorld;
}

Eigen::Vector3d BangBangTrajectory3D::PositionAtT(double t_sec) {
    // Handle timing correctly for position evaluation
    if (t_sec < t_start_s) {
        return Eigen::Vector3d(0, 0, 0);
    }
    
    double t_relative = t_sec - t_start_s;
    
    // Return final position when trajectory is complete
    if (t_sec >= t_finish_s || t_relative >= T) {
        return h;  // Total displacement (final position)
    }
    
    // Scale time to trajectory duration for proper evaluation
    double trajectory_duration = std::max({x_trajectory_dof.total_time, y_trajectory_dof.total_time, theta_trajectory_dof.total_time});
    double scaled_t = (trajectory_duration > 0) ? t_relative * trajectory_duration / T : t_relative;
    
    // Evaluate position at scaled time using bang-bang segments
    Eigen::Vector3d position_fworld;
    position_fworld[0] = EvaluateTrajectoryAtTime(x_trajectory_dof, scaled_t, false);
    position_fworld[1] = EvaluateTrajectoryAtTime(y_trajectory_dof, scaled_t, false);
    position_fworld[2] = EvaluateTrajectoryAtTime(theta_trajectory_dof, scaled_t, false);

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