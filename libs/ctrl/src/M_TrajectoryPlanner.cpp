#include "M_TrajectoryPlanner.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace ctrl {

#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif
constexpr double MAX_VEL_TOLERANCE = 0.2;

// Utility function to normalize angles
double M_TrajectoryPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Adapt velocity to respect maximum velocity constraints
Eigen::Vector2d M_TrajectoryPlanner::adaptVelocity(const Eigen::Vector2d& v0, double v_max) {
    double curr_vel_abs = v0.norm();
    if (curr_vel_abs > v_max && curr_vel_abs < v_max + MAX_VEL_TOLERANCE) {
        return v0.normalized() * v_max;
    }
    return v0;
}

double M_TrajectoryPlanner::adaptVelocity(double v0, double v_max) {
    double curr_vel_abs = std::abs(v0);
    if (curr_vel_abs > v_max && curr_vel_abs < v_max + MAX_VEL_TOLERANCE) {
        return (v0 >= 0) ? v_max : -v_max;
    }
    return v0;
}

//=============================================================================
// M_BangBangTrajectory1D Implementation
//=============================================================================

M_BangBangTrajectory1D& M_BangBangTrajectory1D::generate(
    double s0, double s1, double v0, double v_max, double a_max) {
    
    s0_ = s0;
    s1_ = s1;
    v0_ = v0;
    v_max_ = v_max;
    a_max_ = a_max;
    
    calculateTrajectory();
    return *this;
}

void M_BangBangTrajectory1D::calculateTrajectory() {
    double displacement = s1_ - s0_;
    double sign = (displacement >= 0) ? 1.0 : -1.0;
    
    // Work in normalized coordinates
    double h = std::abs(displacement);
    double v0_norm = sign * v0_;
    
    // Handle zero displacement
    if (h < 1e-6) {
        total_time_ = 0.0;
        t1_ = t2_ = t3_ = 0.0;
        v_peak_ = 0.0;
        has_cruise_phase_ = false;
        return;
    }
    
    // Check if we can reach v_max
    double t_to_vmax = (v_max_ - std::abs(v0_norm)) / a_max_;
    double dist_to_vmax = std::abs(v0_norm) * t_to_vmax + 0.5 * a_max_ * t_to_vmax * t_to_vmax;
    double dist_from_vmax = v_max_ * v_max_ / (2.0 * a_max_);
    
    if (dist_to_vmax + dist_from_vmax <= h) {
        // Trapezoidal profile: accelerate, cruise, decelerate
        has_cruise_phase_ = true;
        v_peak_ = v_max_;
        
        t1_ = (v_max_ - std::abs(v0_norm)) / a_max_;
        double cruise_dist = h - dist_to_vmax - dist_from_vmax;
        double t_cruise = cruise_dist / v_max_;
        t2_ = t1_ + t_cruise;
        t3_ = t2_ + v_max_ / a_max_;
        
        total_time_ = t3_;
    } else {
        // Triangular profile: accelerate then decelerate
        has_cruise_phase_ = false;
        
        // Solve: v0*t1 + 0.5*a*t1^2 + v_peak*t2 - 0.5*a*t2^2 = h
        // where v_peak = v0 + a*t1 and t1 = t2 (symmetrical)
        double a = a_max_;
        double b = 2.0 * std::abs(v0_norm);
        double c = -2.0 * a_max_ * h;
        
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0) {
            t1_ = (-b + std::sqrt(discriminant)) / (2.0 * a);
            v_peak_ = std::abs(v0_norm) + a_max_ * t1_;
            t2_ = t1_;
            t3_ = t1_ + v_peak_ / a_max_;
            total_time_ = t3_;
        } else {
            // Fallback: simple time-optimal solution
            total_time_ = 2.0 * std::sqrt(h / a_max_);
            t1_ = total_time_ / 2.0;
            t2_ = t1_;
            t3_ = total_time_;
            v_peak_ = a_max_ * t1_;
        }
    }
}

double M_BangBangTrajectory1D::getPosition1D(double t) const {
    if (t <= 0) return s0_;
    if (t >= total_time_) return s1_;
    
    double sign = (s1_ >= s0_) ? 1.0 : -1.0;
    double pos = 0.0;
    
    if (t <= t1_) {
        // Acceleration phase
        pos = std::abs(v0_) * t + 0.5 * a_max_ * t * t;
    } else if (t <= t2_) {
        // Cruise phase (if exists)
        double pos1 = std::abs(v0_) * t1_ + 0.5 * a_max_ * t1_ * t1_;
        pos = pos1 + v_peak_ * (t - t1_);
    } else {
        // Deceleration phase
        double pos1 = std::abs(v0_) * t1_ + 0.5 * a_max_ * t1_ * t1_;
        double pos2 = has_cruise_phase_ ? pos1 + v_peak_ * (t2_ - t1_) : pos1;
        double t_decel = t - t2_;
        pos = pos2 + v_peak_ * t_decel - 0.5 * a_max_ * t_decel * t_decel;
    }
    
    return s0_ + sign * pos;
}

double M_BangBangTrajectory1D::getVelocity1D(double t) const {
    if (t <= 0 || t >= total_time_) return 0.0;
    
    double sign = (s1_ >= s0_) ? 1.0 : -1.0;
    double vel = 0.0;
    
    if (t <= t1_) {
        vel = std::abs(v0_) + a_max_ * t;
    } else if (t <= t2_) {
        vel = v_peak_;
    } else {
        double t_decel = t - t2_;
        vel = v_peak_ - a_max_ * t_decel;
    }
    
    return sign * vel;
}

double M_BangBangTrajectory1D::getAcceleration1D(double t) const {
    if (t <= 0 || t >= total_time_) return 0.0;
    
    double sign = (s1_ >= s0_) ? 1.0 : -1.0;
    
    if (t <= t1_) {
        return sign * a_max_;
    } else if (t <= t2_) {
        return 0.0;
    } else {
        return -sign * a_max_;
    }
}

Eigen::Vector3d M_BangBangTrajectory1D::getPosition(double t) const {
    return Eigen::Vector3d(getPosition1D(t), 0, 0);
}

Eigen::Vector3d M_BangBangTrajectory1D::getVelocity(double t) const {
    return Eigen::Vector3d(getVelocity1D(t), 0, 0);
}

Eigen::Vector3d M_BangBangTrajectory1D::getAcceleration(double t) const {
    return Eigen::Vector3d(getAcceleration1D(t), 0, 0);
}

void M_BangBangTrajectory1D::print() const {
    std::cout << "[M_BangBangTrajectory1D] " << s0_ << " -> " << s1_ 
              << " | time: " << total_time_ << "s | v_peak: " << v_peak_ 
              << " | cruise: " << (has_cruise_phase_ ? "yes" : "no") << std::endl;
}

//=============================================================================
// M_BangBangTrajectory2D Implementation  
//=============================================================================

M_BangBangTrajectory2D& M_BangBangTrajectory2D::generate(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1, 
    const Eigen::Vector2d& v0, double v_max, double a_max, double sync_accuracy) {
    
    is_async_ = false;
    rotation_ = 0.0;
    offset_ = s0;
    
    // Generate individual trajectories
    traj_x_.generate(s0.x(), s1.x(), v0.x(), v_max, a_max);
    traj_y_.generate(s0.y(), s1.y(), v0.y(), v_max, a_max);
    
    // Synchronize trajectories
    synchronizeTrajectories(sync_accuracy);
    
    return *this;
}

M_BangBangTrajectory2D& M_BangBangTrajectory2D::generateAsync(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1,
    const Eigen::Vector2d& v0, double v_max, double a_max,
    const Eigen::Vector2d& primary_dir, double sync_accuracy) {
    
    is_async_ = true;
    rotation_ = std::atan2(primary_dir.y(), primary_dir.x());
    offset_ = s0;
    
    // Transform to primary direction coordinate system
    Eigen::Vector2d displacement = s1 - s0;
    double cos_rot = std::cos(-rotation_);
    double sin_rot = std::sin(-rotation_);
    
    Eigen::Vector2d disp_rot(
        displacement.x() * cos_rot - displacement.y() * sin_rot,
        displacement.x() * sin_rot + displacement.y() * cos_rot
    );
    
    Eigen::Vector2d v0_rot(
        v0.x() * cos_rot - v0.y() * sin_rot,
        v0.x() * sin_rot + v0.y() * cos_rot
    );
    
    // Generate trajectories in rotated frame
    traj_x_.generate(0.0, disp_rot.x(), v0_rot.x(), v_max, a_max);
    traj_y_.generate(0.0, disp_rot.y(), v0_rot.y(), v_max, a_max);
    
    // Synchronize with emphasis on primary direction (x in rotated frame)
    synchronizeTrajectories(sync_accuracy);
    
    return *this;
}

void M_BangBangTrajectory2D::synchronizeTrajectories(double sync_accuracy) {
    double max_time = std::max(traj_x_.getTotalTime(), traj_y_.getTotalTime());
    
    if (std::abs(traj_x_.getTotalTime() - traj_y_.getTotalTime()) < sync_accuracy) {
        return; // Already synchronized
    }
    
    // Find synchronization factor using binary search
    double sync_factor = findSyncFactor(max_time, sync_accuracy);
    
    // Apply synchronization by scaling the faster trajectory
    if (traj_x_.getTotalTime() < max_time) {
        double scale = max_time / traj_x_.getTotalTime();
        // Re-generate X trajectory with reduced acceleration
        double new_acc = traj_x_.getAMax() / scale;
        traj_x_.generate(traj_x_.getS0(), traj_x_.getS1(), traj_x_.getV0(), traj_x_.getVMax(), new_acc);
    }
    
    if (traj_y_.getTotalTime() < max_time) {
        double scale = max_time / traj_y_.getTotalTime();
        // Re-generate Y trajectory with reduced acceleration
        double new_acc = traj_y_.getAMax() / scale;
        traj_y_.generate(traj_y_.getS0(), traj_y_.getS1(), traj_y_.getV0(), traj_y_.getVMax(), new_acc);
    }
}

double M_BangBangTrajectory2D::findSyncFactor(double target_time, double sync_accuracy) {
    // Binary search for synchronization factor
    double low = 0.1, high = 2.0;
    
    for (int i = 0; i < 50; ++i) {
        double mid = (low + high) / 2.0;
        
        // Test trajectories with scaled acceleration
        M_BangBangTrajectory1D test_x, test_y;
        test_x.generate(traj_x_.getS0(), traj_x_.getS1(), traj_x_.getV0(), traj_x_.getVMax(), traj_x_.getAMax() * mid);
        test_y.generate(traj_y_.getS0(), traj_y_.getS1(), traj_y_.getV0(), traj_y_.getVMax(), traj_y_.getAMax() * mid);
        
        double time_diff = std::abs(test_x.getTotalTime() - test_y.getTotalTime());
        
        if (time_diff < sync_accuracy) {
            return mid;
        }
        
        if (test_x.getTotalTime() > test_y.getTotalTime()) {
            high = mid;
        } else {
            low = mid;
        }
    }
    
    return (low + high) / 2.0;
}

Eigen::Vector2d M_BangBangTrajectory2D::getPosition2D(double t) const {
    Eigen::Vector2d pos(traj_x_.getPosition1D(t), traj_y_.getPosition1D(t));
    
    if (is_async_) {
        // Transform back from rotated frame
        double cos_rot = std::cos(rotation_);
        double sin_rot = std::sin(rotation_);
        
        Eigen::Vector2d pos_world(
            pos.x() * cos_rot - pos.y() * sin_rot,
            pos.x() * sin_rot + pos.y() * cos_rot
        );
        
        return offset_ + pos_world;
    }
    
    return pos;
}

Eigen::Vector2d M_BangBangTrajectory2D::getVelocity2D(double t) const {
    Eigen::Vector2d vel(traj_x_.getVelocity1D(t), traj_y_.getVelocity1D(t));
    
    if (is_async_) {
        // Transform back from rotated frame
        double cos_rot = std::cos(rotation_);
        double sin_rot = std::sin(rotation_);
        
        return Eigen::Vector2d(
            vel.x() * cos_rot - vel.y() * sin_rot,
            vel.x() * sin_rot + vel.y() * cos_rot
        );
    }
    
    return vel;
}

Eigen::Vector2d M_BangBangTrajectory2D::getAcceleration2D(double t) const {
    Eigen::Vector2d acc(traj_x_.getAcceleration1D(t), traj_y_.getAcceleration1D(t));
    
    if (is_async_) {
        // Transform back from rotated frame
        double cos_rot = std::cos(rotation_);
        double sin_rot = std::sin(rotation_);
        
        return Eigen::Vector2d(
            acc.x() * cos_rot - acc.y() * sin_rot,
            acc.x() * sin_rot + acc.y() * cos_rot
        );
    }
    
    return acc;
}

Eigen::Vector3d M_BangBangTrajectory2D::getPosition(double t) const {
    Eigen::Vector2d pos2d = getPosition2D(t);
    return Eigen::Vector3d(pos2d.x(), pos2d.y(), 0);
}

Eigen::Vector3d M_BangBangTrajectory2D::getVelocity(double t) const {
    Eigen::Vector2d vel2d = getVelocity2D(t);
    return Eigen::Vector3d(vel2d.x(), vel2d.y(), 0);
}

Eigen::Vector3d M_BangBangTrajectory2D::getAcceleration(double t) const {
    Eigen::Vector2d acc2d = getAcceleration2D(t);
    return Eigen::Vector3d(acc2d.x(), acc2d.y(), 0);
}

double M_BangBangTrajectory2D::getTotalTime() const {
    return std::max(traj_x_.getTotalTime(), traj_y_.getTotalTime());
}

double M_BangBangTrajectory2D::getMaxSpeed() const {
    return std::max(traj_x_.getMaxSpeed(), traj_y_.getMaxSpeed());
}

void M_BangBangTrajectory2D::print() const {
    std::cout << "[M_BangBangTrajectory2D] Total time: " << getTotalTime() 
              << "s | Async: " << (is_async_ ? "yes" : "no") << std::endl;
    std::cout << "  X: "; traj_x_.print();
    std::cout << "  Y: "; traj_y_.print();
}

//=============================================================================
// M_BangBangTrajectory3D Implementation
//=============================================================================

M_BangBangTrajectory3D& M_BangBangTrajectory3D::generate(
    const Eigen::Vector3d& s0, const Eigen::Vector3d& s1, 
    const Eigen::Vector3d& v0, const M_MoveConstraints& constraints) {
    
    // Generate 2D position trajectory
    Eigen::Vector2d pos0(s0.x(), s0.y());
    Eigen::Vector2d pos1(s1.x(), s1.y());
    Eigen::Vector2d vel0(v0.x(), v0.y());
    
    if (constraints.primary_direction.norm() > 0.001) {
        position_traj_.generateAsync(pos0, pos1, vel0, 
            constraints.vel_max, constraints.acc_max, constraints.primary_direction);
    } else {
        position_traj_.generate(pos0, pos1, vel0, 
            constraints.vel_max, constraints.acc_max);
    }
    
    // Generate rotation trajectory with angle normalization
    double angle_diff = M_TrajectoryPlanner::normalizeAngle(s1.z() - s0.z());
    double target_angle = s0.z() + angle_diff;
    
    rotation_traj_.generate(s0.z(), target_angle, v0.z(), 
        constraints.vel_max_w, constraints.acc_max_w);
    
    return *this;
}

Eigen::Vector3d M_BangBangTrajectory3D::getPosition(double t) const {
    Eigen::Vector2d pos2d = position_traj_.getPosition2D(t);
    double angle = rotation_traj_.getPosition1D(t);
    return Eigen::Vector3d(pos2d.x(), pos2d.y(), angle);
}

Eigen::Vector3d M_BangBangTrajectory3D::getVelocity(double t) const {
    Eigen::Vector2d vel2d = position_traj_.getVelocity2D(t);
    double angular_vel = rotation_traj_.getVelocity1D(t);
    return Eigen::Vector3d(vel2d.x(), vel2d.y(), angular_vel);
}

Eigen::Vector3d M_BangBangTrajectory3D::getAcceleration(double t) const {
    Eigen::Vector2d acc2d = position_traj_.getAcceleration2D(t);
    double angular_acc = rotation_traj_.getAcceleration1D(t);
    return Eigen::Vector3d(acc2d.x(), acc2d.y(), angular_acc);
}

double M_BangBangTrajectory3D::getTotalTime() const {
    return std::max(position_traj_.getTotalTime(), rotation_traj_.getTotalTime());
}

double M_BangBangTrajectory3D::getMaxSpeed() const {
    double max_linear = position_traj_.getMaxSpeed();
    double max_angular = rotation_traj_.getMaxSpeed();
    return std::max(max_linear, max_angular);
}

void M_BangBangTrajectory3D::print() const {
    std::cout << "[M_BangBangTrajectory3D] Total time: " << getTotalTime() << "s" << std::endl;
    std::cout << "  Position: "; position_traj_.print();
    std::cout << "  Rotation: "; rotation_traj_.print();
}

//=============================================================================
// M_TrajectoryFactory Implementation
//=============================================================================

std::shared_ptr<M_BangBangTrajectory2D> M_TrajectoryFactory::createSync(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1, 
    const Eigen::Vector2d& v0, double v_max, double a_max) {
    
    auto traj = std::make_shared<M_BangBangTrajectory2D>();
    Eigen::Vector2d adapted_v0 = M_TrajectoryPlanner::adaptVelocity(v0, v_max);
    traj->generate(s0, s1, adapted_v0, v_max, a_max);
    return traj;
}

std::shared_ptr<M_BangBangTrajectory2D> M_TrajectoryFactory::createAsync(
    const Eigen::Vector2d& s0, const Eigen::Vector2d& s1,
    const Eigen::Vector2d& v0, double v_max, double a_max,
    const Eigen::Vector2d& primary_dir) {
    
    auto traj = std::make_shared<M_BangBangTrajectory2D>();
    Eigen::Vector2d adapted_v0 = M_TrajectoryPlanner::adaptVelocity(v0, v_max);
    traj->generateAsync(s0, s1, adapted_v0, v_max, a_max, primary_dir);
    return traj;
}

std::shared_ptr<M_BangBangTrajectory1D> M_TrajectoryFactory::createSingle(
    double s0, double s1, double v0, double v_max, double a_max) {
    
    auto traj = std::make_shared<M_BangBangTrajectory1D>();
    double adapted_v0 = M_TrajectoryPlanner::adaptVelocity(v0, v_max);
    traj->generate(s0, s1, adapted_v0, v_max, a_max);
    return traj;
}

std::shared_ptr<M_BangBangTrajectory1D> M_TrajectoryFactory::createOrientation(
    double s0, double s1, double v0, double v_max, double a_max) {
    
    auto traj = std::make_shared<M_BangBangTrajectory1D>();
    double adapted_s1 = s0 + M_TrajectoryPlanner::normalizeAngle(s1 - s0);
    double adapted_v0 = M_TrajectoryPlanner::adaptVelocity(v0, v_max);
    traj->generate(s0, adapted_s1, adapted_v0, v_max, a_max);
    return traj;
}

std::shared_ptr<M_BangBangTrajectory3D> M_TrajectoryFactory::create3D(
    const Eigen::Vector3d& s0, const Eigen::Vector3d& s1,
    const Eigen::Vector3d& v0, const M_MoveConstraints& constraints) {
    
    auto traj = std::make_shared<M_BangBangTrajectory3D>();
    traj->generate(s0, s1, v0, constraints);
    return traj;
}

//=============================================================================
// M_TrajectoryPlanner Implementation
//=============================================================================

std::shared_ptr<M_Trajectory> M_TrajectoryPlanner::generatePositionTrajectory(
    const M_RobotState& robot_state, const Eigen::Vector3d& destination) {
    
    return generatePositionTrajectory(robot_state.constraints,
        robot_state.position, robot_state.velocity, destination);
}

std::shared_ptr<M_Trajectory> M_TrajectoryPlanner::generatePositionTrajectory(
    const M_MoveConstraints& constraints, const Eigen::Vector3d& current_pose,
    const Eigen::Vector3d& current_velocity, const Eigen::Vector3d& destination) {
    
    return M_TrajectoryFactory::create3D(current_pose, destination, current_velocity, constraints);
}

std::shared_ptr<M_Trajectory> M_TrajectoryPlanner::generateRotationTrajectory(
    const M_RobotState& robot_state, double target_angle) {
    
    return M_TrajectoryFactory::createOrientation(
        robot_state.position.z(), target_angle, robot_state.velocity.z(),
        robot_state.constraints.vel_max_w, robot_state.constraints.acc_max_w);
}

std::shared_ptr<M_Trajectory> M_TrajectoryPlanner::generateTrajectoryToReachPointInTime(
    const M_RobotState& robot_state, const Eigen::Vector3d& destination, double target_time) {
    
    // For now, generate optimal trajectory (time-constrained trajectories require more complex implementation)
    auto traj = generatePositionTrajectory(robot_state, destination);
    
    if (traj->getTotalTime() > target_time) {
        std::cout << "[M_TrajectoryPlanner] Warning: Optimal time " << traj->getTotalTime() 
                  << "s exceeds target time " << target_time << "s" << std::endl;
    }
    
    return traj;
}

M_PathFinderResult M_TrajectoryPlanner::findPath(const M_PathFinderInput& input) {
    M_PathFinderResult result;
    
    try {
        if (input.use_primary_direction && input.primary_direction.norm() > 0.001) {
            // Use async trajectory with primary direction
            M_MoveConstraints constraints = input.robot_state.constraints;
            constraints.primary_direction = input.primary_direction;
            
            result.trajectory = generatePositionTrajectory(constraints,
                input.robot_state.position, input.robot_state.velocity, input.destination);
        } else {
            // Use synchronized trajectory
            result.trajectory = generatePositionTrajectory(input.robot_state, input.destination);
        }
        
        result.path_found = true;
        result.total_time = result.trajectory->getTotalTime();
        result.info = "Path found successfully";
        
    } catch (const std::exception& e) {
        result.path_found = false;
        result.total_time = 0.0;
        result.info = std::string("Path finding failed: ") + e.what();
    }
    
    return result;
}

double M_TrajectoryPlanner::calculateOptimalTime(
    const Eigen::Vector3d& displacement, const Eigen::Vector3d& initial_velocity,
    const M_MoveConstraints& constraints) {
    
    // Create temporary trajectory to calculate optimal time
    M_BangBangTrajectory3D temp_traj;
    Eigen::Vector3d start = Eigen::Vector3d::Zero();
    Eigen::Vector3d end = displacement;
    
    temp_traj.generate(start, end, initial_velocity, constraints);
    return temp_traj.getTotalTime();
}

bool M_TrajectoryPlanner::isComeToStopFaster(
    const M_RobotState& robot_state, const Eigen::Vector3d& destination) {
    
    // Generate trajectory without stopping
    auto traj_direct = generatePositionTrajectory(robot_state, destination);
    
    // Simulate come-to-stop phase
    double look_ahead = 0.05; // 50ms
    double brake_acc = robot_state.constraints.brk_max * 0.9;
    
    Eigen::Vector2d vel_2d = robot_state.velocity.head<2>();
    double t_brake = std::min(vel_2d.norm() / brake_acc, look_ahead);
    
    // Calculate state after braking
    Eigen::Vector2d brake_acc_vec = -vel_2d.normalized() * brake_acc;
    Eigen::Vector2d vel_after = vel_2d + brake_acc_vec * t_brake;
    Eigen::Vector2d pos_after = robot_state.position.head<2>() + 
        vel_2d * t_brake + 0.5 * brake_acc_vec * t_brake * t_brake;
    
    // Generate trajectory after braking
    M_RobotState state_after = robot_state;
    state_after.position.head<2>() = pos_after;
    state_after.velocity.head<2>() = vel_after;
    
    auto traj_after_brake = generatePositionTrajectory(state_after, destination);
    
    return (traj_after_brake->getTotalTime() + look_ahead + 0.01) < traj_direct->getTotalTime();
}

} // namespace ctrl