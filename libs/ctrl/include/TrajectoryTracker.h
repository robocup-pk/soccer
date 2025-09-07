#pragma once

#include <Eigen/Dense>
#include <memory>

namespace ctrl {

// Forward declaration
class AdvancedMotionPlanner;

/**
 * @brief Advanced trajectory tracker with PID feedback control.
 *
 * This class implements the complete advanced pipeline:
 * 1. Receives a time-parameterized trajectory from AdvancedMotionPlanner
 * 2. Uses feedforward commands (desired position/velocity at current time)
 * 3. Adds PID feedback corrections based on actual vs desired position/orientation
 * 4. Combines feedforward + feedback for robust trajectory following
 */
class TrajectoryTracker {
public:
    TrajectoryTracker();

    /**
     * @brief Sets the time-parameterized trajectory to follow.
     * @param planner Shared pointer to AdvancedMotionPlanner containing the complete trajectory
     */
    void setTrajectory(std::shared_ptr<AdvancedMotionPlanner> planner);

    /**
     * @brief Main update loop combining feedforward + PID feedback.
     * @param current_pose Robot's current pose (x, y, theta) in world frame
     * @return Body-frame velocity command (vx, vy, omega)
     */
    Eigen::Vector3d update(const Eigen::Vector3d& current_pose);

    /**
     * @brief Checks if trajectory is completed.
     */
    bool isFinished() const;

private:
    // Internal PID controller structure
    struct PIDController {
        double kp = 0.0;
        double ki = 0.0; 
        double kd = 0.0;
        Eigen::Vector2d integral = Eigen::Vector2d::Zero();
        Eigen::Vector2d prev_error = Eigen::Vector2d::Zero();
        double integral_clamp = 1.0;

        Eigen::Vector2d calculate(const Eigen::Vector2d& error, double dt) {
            if (dt < 1e-6) return Eigen::Vector2d::Zero();
            
            integral += error * dt;
            
            // Clamp integral to prevent windup
            if (integral.norm() > integral_clamp) {
                integral = integral.normalized() * integral_clamp;
            }
            
            Eigen::Vector2d derivative = (error - prev_error) / dt;
            prev_error = error;
            
            return (kp * error) + (ki * integral) + (kd * derivative);
        }
        
        void reset() {
            integral.setZero();
            prev_error.setZero();
        }
    };
    
    std::shared_ptr<AdvancedMotionPlanner> motion_planner_;
    PIDController pos_pid_;   // PID for XY position
    PIDController angle_pid_; // PID for orientation
    
    double start_time_;
    double last_update_time_;
    bool is_finished_;
};

} // namespace ctrl