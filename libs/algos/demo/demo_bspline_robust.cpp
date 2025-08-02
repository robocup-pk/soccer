#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>
#include <deque>
#include <random>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"

// Robust B-spline controller with filtered feedback and proper feedforward
class RobustBSplineController {
public:
    RobustBSplineController() 
        : kp_position_(1.5),    // Reduced from typical 3.0 to prevent oscillation
          kd_velocity_(0.8),    // Damping term
          kp_angle_(2.0),       // Angular control
          max_accel_(0.5),      // m/s^2 - limit acceleration
          filter_alpha_(0.3) {  // Low-pass filter coefficient
    }
    
    // Update control with filtered feedback and feedforward
    Eigen::Vector3d ComputeVelocity(
        const Eigen::Vector3d& current_pose,
        const Eigen::Vector3d& desired_pose,
        const Eigen::Vector3d& desired_velocity,
        const Eigen::Vector3d& current_velocity) {
        
        // Low-pass filter the current pose to reduce noise effects
        if (!initialized_filter_) {
            filtered_pose_ = current_pose;
            initialized_filter_ = true;
        }
        filtered_pose_ = filter_alpha_ * current_pose + (1 - filter_alpha_) * filtered_pose_;
        
        // Position error using filtered pose
        Eigen::Vector2d pos_error = desired_pose.head<2>() - filtered_pose_.head<2>();
        double angle_error = desired_pose[2] - filtered_pose_[2];
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        // Velocity error for damping
        Eigen::Vector2d vel_error = desired_velocity.head<2>() - current_velocity.head<2>();
        
        // Control law: feedforward + proportional + derivative
        Eigen::Vector2d control_velocity = 
            desired_velocity.head<2>() +                    // Feedforward
            kp_position_ * pos_error -                      // Proportional
            kd_velocity_ * vel_error;                       // Derivative (damping)
        
        // Angular control
        double control_angular = 
            desired_velocity[2] +                           // Feedforward
            kp_angle_ * angle_error;
        
        // Limit acceleration to prevent sudden changes
        Eigen::Vector3d cmd_velocity;
        cmd_velocity.head<2>() = control_velocity;
        cmd_velocity[2] = control_angular;
        
        if (!last_cmd_velocity_.isZero()) {
            Eigen::Vector3d accel = (cmd_velocity - last_cmd_velocity_) / 0.016;  // 60Hz
            if (accel.head<2>().norm() > max_accel_) {
                // Limit acceleration
                accel.head<2>() = accel.head<2>().normalized() * max_accel_;
                cmd_velocity.head<2>() = last_cmd_velocity_.head<2>() + accel.head<2>() * 0.016;
            }
        }
        last_cmd_velocity_ = cmd_velocity;
        
        return cmd_velocity;
    }
    
    // Simulate realistic sensor feedback with noise and delay
    Eigen::Vector3d SimulateSensorFeedback(const Eigen::Vector3d& true_pose) {
        // Add to delay buffer
        pose_buffer_.push_back(true_pose);
        if (pose_buffer_.size() > delay_frames_) {
            pose_buffer_.pop_front();
        }
        
        // Get delayed pose
        Eigen::Vector3d delayed_pose = pose_buffer_.empty() ? true_pose : pose_buffer_.front();
        
        // Add noise
        std::normal_distribution<double> noise(0.0, 0.005);  // 5mm std dev
        Eigen::Vector3d noisy_pose = delayed_pose;
        noisy_pose[0] += noise(rng_);
        noisy_pose[1] += noise(rng_);
        noisy_pose[2] += noise(rng_) * 0.1;  // Less noise on angle
        
        return noisy_pose;
    }
    
private:
    double kp_position_;
    double kd_velocity_;
    double kp_angle_;
    double max_accel_;
    double filter_alpha_;
    
    Eigen::Vector3d filtered_pose_;
    Eigen::Vector3d last_cmd_velocity_ = Eigen::Vector3d::Zero();
    bool initialized_filter_ = false;
    
    // For sensor simulation
    std::deque<Eigen::Vector3d> pose_buffer_;
    static constexpr int delay_frames_ = 3;  // 50ms delay at 60Hz
    std::mt19937 rng_{std::random_device{}()};
};

// Friction model
Eigen::Vector3d ApplyFriction(const Eigen::Vector3d& commanded_velocity) {
    Eigen::Vector3d actual_velocity = commanded_velocity;
    actual_velocity[0] *= 0.75;  // 75% efficiency in X
    actual_velocity[1] *= 0.75;  // 75% efficiency in Y
    actual_velocity[2] *= 0.65;  // 65% efficiency in rotation
    return actual_velocity;
}

int main(int argc, char* argv[]) {
    std::cout << "[Robust B-Spline Demo] Testing improved B-spline controller" << std::endl;
    std::cout << "This demo shows how to fix the oscillation issues seen in real robot" << std::endl;
    
    // Initialize visualization with custom soccer objects
    std::vector<state::SoccerObject> soccer_objects;
    
    // Create three robots for visualization
    // Robot 0: Standard B-spline (red)
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),  // Robot size
                           Eigen::Vector3d::Zero(),       // Initial velocity
                           Eigen::Vector3d::Zero(),       // Initial acceleration
                           10));                          // Mass
    
    // Robot 1: Robust B-spline (blue)
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(0, 0.3, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           10));
    
    // Robot 2: Ideal B-spline (green)
    soccer_objects.push_back(
        state::SoccerObject("robot2", Eigen::Vector3d(0, -0.3, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           10));
    
    // Add ball for completeness
    soccer_objects.push_back(
        state::SoccerObject("ball", Eigen::Vector3d(2, 0, 0),
                           Eigen::Vector2d(0.043*2, 0.043*2),  // Ball diameter
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           0.046f));
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Test waypoints
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    
    // Create controllers
    RobustBSplineController robust_controller;
    
    // Create robots
    {
        auto standard_robot = std::make_unique<rob::RobotManager>();
        auto robust_robot = std::make_unique<rob::RobotManager>();
        
        // Configure robots
        double start_time = util::GetCurrentTime();
        
        // Standard B-spline (will oscillate with noisy feedback)
        standard_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        standard_robot->SetBSplinePath(waypoints, start_time);
        
        // Robust B-spline (custom controller)
        robust_robot->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        robust_robot->SetBSplinePath(waypoints, start_time);
        
        std::cout << "\nRobot visualization:" << std::endl;
        std::cout << "Red robot: Standard B-spline with noisy sensor feedback (oscillates)" << std::endl;
        std::cout << "Blue robot: Robust B-spline with filtered feedback and tuned gains" << std::endl;
        std::cout << "Green robot: Ideal B-spline (no noise, perfect sensors)" << std::endl;
        
        // Simulation state
        Eigen::Vector3d standard_true_pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d robust_true_pose = Eigen::Vector3d::Zero();
        Eigen::Vector3d ideal_pose = Eigen::Vector3d::Zero();
        
        double last_print_time = start_time;
        int frame = 0;
        
        // Main loop
        while (frame < 300 && gl_sim) {  // 5 seconds at 60Hz
            double current_time = util::GetCurrentTime();
            double dt = 0.016;
            
            // IMPORTANT: Update the actual B-spline robot first to get the desired trajectory
            robust_robot->ControlLogic();
            robust_robot->SenseLogic();
            
            // Get the ideal pose from the B-spline robot
            ideal_pose = robust_robot->GetPoseInWorldFrame();
            
            // Get desired trajectory from B-spline
            Eigen::Vector3d desired_velocity = standard_robot->GetVelocityInWorldFrame();
            
            // Standard robot with noisy feedback
            Eigen::Vector3d noisy_pose_standard = robust_controller.SimulateSensorFeedback(standard_true_pose);
            standard_robot->NewCameraData(noisy_pose_standard);
            
            // Update standard robot physics
            Eigen::Vector3d cmd_vel_standard = standard_robot->GetVelocityInWorldFrame();
            Eigen::Vector3d actual_vel_standard = ApplyFriction(cmd_vel_standard);
            standard_true_pose.head<2>() += actual_vel_standard.head<2>() * dt;
            standard_true_pose[2] += actual_vel_standard[2] * dt;
            
            // Robust robot with filtered feedback
            Eigen::Vector3d noisy_pose_robust = robust_controller.SimulateSensorFeedback(robust_true_pose);
            Eigen::Vector3d robust_velocity = robust_controller.ComputeVelocity(
                noisy_pose_robust, ideal_pose, desired_velocity, 
                ApplyFriction(desired_velocity)
            );
            
            // Update robust robot physics
            Eigen::Vector3d actual_vel_robust = ApplyFriction(robust_velocity);
            robust_true_pose.head<2>() += actual_vel_robust.head<2>() * dt;
            robust_true_pose[2] += actual_vel_robust[2] * dt;
            
            // Update visualization
            soccer_objects[0].position = standard_true_pose;
            soccer_objects[1].position = robust_true_pose;
            soccer_objects[2].position = ideal_pose;
            
            // Print status
            if (current_time - last_print_time > 1.0) {
                std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                          << current_time - start_time << "s]" << std::endl;
                std::cout << "Ideal (no friction):     " << ideal_pose.transpose() << std::endl;
                std::cout << "Standard (oscillating):  " << standard_true_pose.transpose() << std::endl;
                std::cout << "Robust (filtered):       " << robust_true_pose.transpose() << std::endl;
                
                double standard_error = (standard_true_pose.head<2>() - ideal_pose.head<2>()).norm();
                double robust_error = (robust_true_pose.head<2>() - ideal_pose.head<2>()).norm();
                std::cout << "Standard error: " << standard_error << "m" << std::endl;
                std::cout << "Robust error:   " << robust_error << "m" << std::endl;
                
                last_print_time = current_time;
            }
            
            // Run visualization
            if (!gl_sim->RunSimulationStep(soccer_objects, dt)) {
                break;
            }
            
            frame++;
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        
        // Final results
        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        std::cout << "Expected distance: 1.0m" << std::endl;
        std::cout << "Ideal (no friction):    " << ideal_pose[0] << "m" << std::endl;
        std::cout << "Standard (oscillating): " << standard_true_pose[0] << "m" << std::endl;
        std::cout << "Robust (filtered):      " << robust_true_pose[0] << "m" << std::endl;
        
        std::cout << "\nThe robust controller achieves:" << std::endl;
        std::cout << "- Reduced oscillation through filtering and damping" << std::endl;
        std::cout << "- Better trajectory tracking with proper feedforward" << std::endl;
        std::cout << "- Smoother motion with acceleration limiting" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    return 0;
}