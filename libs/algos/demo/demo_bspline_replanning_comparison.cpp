#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <iomanip>
#include <random>
#include <Eigen/Dense>

#include "RobotManager.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "Utils.h"
#include "BSplineTrajectoryManager.h"

// Simulate replanning logic
class ReplanningSimulator {
public:
    ReplanningSimulator() 
        : replan_threshold_(0.1),  // 10cm error threshold
          replan_interval_(0.5),   // Min 0.5s between replans
          last_replan_time_(0.0),
          replan_count_(0),
          noise_gen_(std::chrono::steady_clock::now().time_since_epoch().count()),
          sensor_noise_(0.0, 0.005) {  // 5mm standard deviation
    }
    
    bool ShouldReplan(const Eigen::Vector3d& actual_pose, 
                      const Eigen::Vector3d& desired_pose,
                      double current_time) {
        if (current_time - last_replan_time_ < replan_interval_) {
            return false;
        }
        
        double error = (actual_pose.head<2>() - desired_pose.head<2>()).norm();
        return error > replan_threshold_;
    }
    
    void RecordReplan(double current_time) {
        last_replan_time_ = current_time;
        replan_count_++;
        std::cout << "[Replan #" << replan_count_ << "] at t=" 
                  << std::fixed << std::setprecision(2) << current_time << "s" << std::endl;
    }
    
    int GetReplanCount() const { return replan_count_; }
    
    // Simulate sensor measurement with noise and delay
    Eigen::Vector3d GetSensorMeasurement(const Eigen::Vector3d& true_pose,
                                        std::deque<Eigen::Vector3d>& pose_buffer,
                                        int delay_frames = 3) {  // 50ms at 60Hz
        // Add to buffer
        pose_buffer.push_back(true_pose);
        if (pose_buffer.size() > delay_frames) {
            pose_buffer.pop_front();
        }
        
        // Get delayed pose
        Eigen::Vector3d delayed_pose = pose_buffer.empty() ? true_pose : pose_buffer.front();
        
        // Add noise
        Eigen::Vector3d noisy_pose = delayed_pose;
        noisy_pose[0] += sensor_noise_(noise_gen_);
        noisy_pose[1] += sensor_noise_(noise_gen_);
        noisy_pose[2] += sensor_noise_(noise_gen_) * 0.1;
        
        return noisy_pose;
    }
    
private:
    double replan_threshold_;
    double replan_interval_;
    double last_replan_time_;
    int replan_count_;
    std::mt19937 noise_gen_;
    std::normal_distribution<double> sensor_noise_;
};

// Apply friction to simulate real robot
Eigen::Vector3d ApplyFriction(const Eigen::Vector3d& commanded_velocity) {
    Eigen::Vector3d actual_velocity = commanded_velocity;
    actual_velocity[0] *= 0.75;  // 75% efficiency
    actual_velocity[1] *= 0.75;
    actual_velocity[2] *= 0.65;
    return actual_velocity;
}

int main(int argc, char* argv[]) {
    std::cout << "[B-Spline Replanning Comparison Demo]" << std::endl;
    std::cout << "Comparing trajectory following with and without replanning" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    
    // Robot 0: Without replanning (red)
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Robot 1: With replanning (blue)
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(0, 0.4, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Target ball
    soccer_objects.push_back(
        state::SoccerObject("ball", Eigen::Vector3d(1, 0, 0),
                           Eigen::Vector2d(0.086, 0.086),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 0.046f));
    
    auto gl_sim = std::make_unique<vis::GLSimulation>();
    gl_sim->InitGameObjects(soccer_objects);
    
    // Test waypoints
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    
    // Create robots
    auto robot_no_replan = std::make_unique<rob::RobotManager>();
    auto robot_with_replan = std::make_unique<rob::RobotManager>();
    
    // Configure both for B-spline
    robot_no_replan->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    robot_with_replan->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    
    double start_time = util::GetCurrentTime();
    robot_no_replan->SetBSplinePath(waypoints, start_time);
    robot_with_replan->SetBSplinePath(waypoints, start_time);
    
    std::cout << "\nRobot visualization:" << std::endl;
    std::cout << "Red robot: B-spline WITHOUT replanning" << std::endl;
    std::cout << "Blue robot: B-spline WITH replanning" << std::endl;
    
    // Simulation state
    Eigen::Vector3d true_pose_no_replan = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_pose_with_replan = Eigen::Vector3d(0, 0.4, 0);
    
    // Sensor delay buffers
    std::deque<Eigen::Vector3d> buffer_no_replan, buffer_with_replan;
    
    // Replanning logic
    ReplanningSimulator replan_sim;
    
    // Statistics
    double max_error_no_replan = 0.0;
    double max_error_with_replan = 0.0;
    double total_error_no_replan = 0.0;
    double total_error_with_replan = 0.0;
    int error_samples = 0;
    
    // Main loop
    double last_print_time = start_time;
    int frame = 0;
    const int max_frames = 600;  // 10 seconds
    
    while (frame < max_frames && gl_sim) {
        double current_time = util::GetCurrentTime();
        double dt = 0.016;
        
        // Get sensor measurements (with noise and delay)
        Eigen::Vector3d sensor_pose_no_replan = 
            replan_sim.GetSensorMeasurement(true_pose_no_replan, buffer_no_replan);
        Eigen::Vector3d sensor_pose_with_replan = 
            replan_sim.GetSensorMeasurement(true_pose_with_replan, buffer_with_replan);
        
        // Feed sensor data to robots
        robot_no_replan->NewCameraData(sensor_pose_no_replan);
        robot_with_replan->NewCameraData(sensor_pose_with_replan);
        
        // Update robot without replanning
        robot_no_replan->ControlLogic();
        robot_no_replan->SenseLogic();
        Eigen::Vector3d vel_no_replan = robot_no_replan->GetVelocityInWorldFrame();
        
        // Check if replanning is needed for second robot
        Eigen::Vector3d desired_pose = robot_with_replan->GetPoseInWorldFrame();
        if (replan_sim.ShouldReplan(sensor_pose_with_replan, desired_pose, current_time)) {
            // Create new path from current position
            std::vector<Eigen::Vector3d> new_waypoints;
            new_waypoints.push_back(sensor_pose_with_replan);
            // Add remaining waypoints (simplified - just go to goal)
            new_waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            
            robot_with_replan->SetBSplinePath(new_waypoints, current_time);
            replan_sim.RecordReplan(current_time);
        }
        
        // Update robot with replanning
        robot_with_replan->ControlLogic();
        robot_with_replan->SenseLogic();
        Eigen::Vector3d vel_with_replan = robot_with_replan->GetVelocityInWorldFrame();
        
        // Apply physics (friction)
        Eigen::Vector3d actual_vel_no_replan = ApplyFriction(vel_no_replan);
        Eigen::Vector3d actual_vel_with_replan = ApplyFriction(vel_with_replan);
        
        // Update true positions
        true_pose_no_replan.head<2>() += actual_vel_no_replan.head<2>() * dt;
        true_pose_no_replan[2] += actual_vel_no_replan[2] * dt;
        
        true_pose_with_replan.head<2>() += actual_vel_with_replan.head<2>() * dt;
        true_pose_with_replan[2] += actual_vel_with_replan[2] * dt;
        
        // Calculate errors from straight line
        double error_no_replan = std::abs(true_pose_no_replan[1]);
        double error_with_replan = std::abs(true_pose_with_replan[1]);
        
        max_error_no_replan = std::max(max_error_no_replan, error_no_replan);
        max_error_with_replan = std::max(max_error_with_replan, error_with_replan);
        total_error_no_replan += error_no_replan;
        total_error_with_replan += error_with_replan;
        error_samples++;
        
        // Update visualization
        soccer_objects[0].position = true_pose_no_replan;
        soccer_objects[1].position = true_pose_with_replan;
        
        // Print status
        if (current_time - last_print_time > 1.0) {
            std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                      << current_time - start_time << "s]" << std::endl;
            std::cout << "No replan:    pos=" << std::fixed << std::setprecision(3) 
                      << true_pose_no_replan.transpose() 
                      << ", Y-error=" << error_no_replan << "m" << std::endl;
            std::cout << "With replan:  pos=" << true_pose_with_replan.transpose() 
                      << ", Y-error=" << error_with_replan << "m" << std::endl;
            std::cout << "Velocity magnitudes: " << vel_no_replan.norm() 
                      << " vs " << vel_with_replan.norm() << " m/s" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Check if reached goal
        if (true_pose_no_replan[0] > 0.9 && true_pose_with_replan[0] > 0.9) {
            std::cout << "\nBoth robots reached goal region!" << std::endl;
            break;
        }
        
        // Run visualization
        if (!gl_sim->RunSimulationStep(soccer_objects, dt)) {
            break;
        }
        
        frame++;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    // Final analysis
    std::cout << "\n=== FINAL COMPARISON ===" << std::endl;
    std::cout << "\nWithout Replanning:" << std::endl;
    std::cout << "- Final position: " << true_pose_no_replan.transpose() << std::endl;
    std::cout << "- Max Y-error: " << max_error_no_replan << "m" << std::endl;
    std::cout << "- Avg Y-error: " << total_error_no_replan / error_samples << "m" << std::endl;
    std::cout << "- Replans: 0" << std::endl;
    
    std::cout << "\nWith Replanning:" << std::endl;
    std::cout << "- Final position: " << true_pose_with_replan.transpose() << std::endl;
    std::cout << "- Max Y-error: " << max_error_with_replan << "m" << std::endl;
    std::cout << "- Avg Y-error: " << total_error_with_replan / error_samples << "m" << std::endl;
    std::cout << "- Replans: " << replan_sim.GetReplanCount() << std::endl;
    
    std::cout << "\n=== CONCLUSION ===" << std::endl;
    if (max_error_with_replan > max_error_no_replan * 1.1) {
        std::cout << "Replanning made the trajectory WORSE due to:" << std::endl;
        std::cout << "- Acting on delayed sensor data" << std::endl;
        std::cout << "- Trajectory discontinuities from replanning" << std::endl;
        std::cout << "- Computational overhead" << std::endl;
    } else if (max_error_with_replan < max_error_no_replan * 0.9) {
        std::cout << "Replanning improved trajectory tracking by "
                  << (1.0 - max_error_with_replan/max_error_no_replan) * 100 << "%" << std::endl;
    } else {
        std::cout << "Replanning had minimal effect on trajectory tracking" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    return 0;
}