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
#include "StateEstimator.h"

// Replanning logic using state estimation
class StateEstimationReplanner {
public:
    StateEstimationReplanner() 
        : replan_threshold_(0.1),  // 10cm error threshold
          replan_interval_(0.5),   // Min 0.5s between replans
          last_replan_time_(0.0),
          replan_count_(0) {
        std::cout << "[StateEstimationReplanner] Using Kalman filtered state for replanning decisions" << std::endl;
    }
    
    bool ShouldReplan(const Eigen::Vector3d& estimated_pose, 
                      const Eigen::Vector3d& desired_pose,
                      double current_time) {
        if (current_time - last_replan_time_ < replan_interval_) {
            return false;
        }
        
        double error = (estimated_pose.head<2>() - desired_pose.head<2>()).norm();
        return error > replan_threshold_;
    }
    
    void RecordReplan(double current_time) {
        last_replan_time_ = current_time;
        replan_count_++;
        std::cout << "[Replan #" << replan_count_ << "] at t=" 
                  << std::fixed << std::setprecision(2) << current_time << "s" << std::endl;
    }
    
    int GetReplanCount() const { return replan_count_; }
    
private:
    double replan_threshold_;
    double replan_interval_;
    double last_replan_time_;
    int replan_count_;
};

// Apply realistic physics
Eigen::Vector3d ApplyRealRobotPhysics(const Eigen::Vector3d& commanded_velocity, 
                                      std::mt19937& rng,
                                      std::normal_distribution<double>& noise_dist) {
    Eigen::Vector3d actual_velocity = commanded_velocity;
    
    // Apply friction (75% efficiency)
    actual_velocity[0] *= 0.75;
    actual_velocity[1] *= 0.75;
    actual_velocity[2] *= 0.65;  // Even worse for rotation
    
    // Add small noise
    actual_velocity[0] += noise_dist(rng) * 0.01;
    actual_velocity[1] += noise_dist(rng) * 0.01;
    actual_velocity[2] += noise_dist(rng) * 0.005;
    
    return actual_velocity;
}

int main(int argc, char* argv[]) {
    std::cout << "[B-Spline Replanning with State Estimation Demo]" << std::endl;
    std::cout << "Comparing raw camera vs state estimation for replanning" << std::endl;
    
    // Initialize visualization
    std::vector<state::SoccerObject> soccer_objects;
    
    // Robot 0: Replanning with raw camera data (red)
    soccer_objects.push_back(
        state::SoccerObject("robot0", Eigen::Vector3d(0, 0, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Robot 1: Replanning with state estimation (blue)
    soccer_objects.push_back(
        state::SoccerObject("robot1", Eigen::Vector3d(0, 0.4, 0), 
                           Eigen::Vector2d(0.18, 0.18),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), 10));
    
    // Target
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
    auto robot_camera = std::make_unique<rob::RobotManager>();
    auto robot_state_est = std::make_unique<rob::RobotManager>();
    
    // Configure both for B-spline
    robot_camera->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    robot_state_est->SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
    
    double start_time = util::GetCurrentTime();
    robot_camera->SetBSplinePath(waypoints, start_time);
    
    // Adjust waypoints for second robot
    std::vector<Eigen::Vector3d> waypoints2;
    waypoints2.push_back(Eigen::Vector3d(0, 0.4, 0));
    waypoints2.push_back(Eigen::Vector3d(0.5, 0.4, 0));
    waypoints2.push_back(Eigen::Vector3d(1.0, 0.4, 0));
    robot_state_est->SetBSplinePath(waypoints2, start_time);
    
    std::cout << "\nRobot visualization:" << std::endl;
    std::cout << "Red robot: Replanning with RAW CAMERA data (50ms delay)" << std::endl;
    std::cout << "Blue robot: Replanning with STATE ESTIMATION (Kalman filtered)" << std::endl;
    
    // Create separate state estimators
    auto state_estimator_1 = std::make_unique<est::StateEstimator>();
    auto state_estimator_2 = std::make_unique<est::StateEstimator>();
    
    // Initialize state estimators
    state_estimator_1->InitializePose(Eigen::Vector3d(0, 0, 0));
    state_estimator_2->InitializePose(Eigen::Vector3d(0, 0.4, 0));
    
    // Simulation state
    Eigen::Vector3d true_pose_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_pose_state_est = Eigen::Vector3d(0, 0.4, 0);
    
    // Sensor delay buffers for camera
    std::deque<Eigen::Vector3d> camera_buffer_1, camera_buffer_2;
    const int delay_frames = 3;  // 50ms at 60Hz
    
    // Replanning logic
    StateEstimationReplanner replan_camera, replan_state_est;
    
    // Noise generation
    std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::normal_distribution<double> sensor_noise(0.0, 0.005);  // 5mm std dev
    std::normal_distribution<double> physics_noise(0.0, 1.0);
    
    // Statistics
    double max_error_camera = 0.0;
    double max_error_state_est = 0.0;
    double total_error_camera = 0.0;
    double total_error_state_est = 0.0;
    int error_samples = 0;
    
    // Main loop
    double last_print_time = start_time;
    int frame = 0;
    const int max_frames = 600;  // 10 seconds
    
    while (frame < max_frames && gl_sim) {
        double current_time = util::GetCurrentTime();
        double dt = 0.016;  // 60Hz
        
        // === ROBOT 1: Camera-based replanning ===
        // Simulate delayed camera measurement
        camera_buffer_1.push_back(true_pose_camera);
        if (camera_buffer_1.size() > delay_frames) {
            camera_buffer_1.pop_front();
        }
        Eigen::Vector3d delayed_camera_pose = camera_buffer_1.empty() ? true_pose_camera : camera_buffer_1.front();
        
        // Add noise to camera measurement
        Eigen::Vector3d noisy_camera_pose = delayed_camera_pose;
        noisy_camera_pose[0] += sensor_noise(rng);
        noisy_camera_pose[1] += sensor_noise(rng);
        noisy_camera_pose[2] += sensor_noise(rng) * 0.1;
        
        // Feed to robot
        robot_camera->NewCameraData(noisy_camera_pose);
        
        // Check if replanning needed (using raw camera)
        Eigen::Vector3d desired_pose_1 = robot_camera->GetPoseInWorldFrame();
        if (replan_camera.ShouldReplan(noisy_camera_pose, desired_pose_1, current_time)) {
            std::vector<Eigen::Vector3d> new_waypoints;
            new_waypoints.push_back(noisy_camera_pose);
            new_waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            
            robot_camera->SetBSplinePath(new_waypoints, current_time);
            replan_camera.RecordReplan(current_time);
            std::cout << "  Camera-based replan from delayed position: " << noisy_camera_pose.transpose() << std::endl;
        }
        
        // Get velocity command
        robot_camera->ControlLogic();
        robot_camera->SenseLogic();
        Eigen::Vector3d vel_camera = robot_camera->GetVelocityInWorldFrame();
        
        // === ROBOT 2: State estimation-based replanning ===
        // Simulate camera measurement for state estimator
        camera_buffer_2.push_back(true_pose_state_est);
        if (camera_buffer_2.size() > delay_frames) {
            camera_buffer_2.pop_front();
        }
        Eigen::Vector3d delayed_camera_pose_2 = camera_buffer_2.empty() ? true_pose_state_est : camera_buffer_2.front();
        
        // Add noise
        Eigen::Vector3d noisy_camera_pose_2 = delayed_camera_pose_2;
        noisy_camera_pose_2[0] += sensor_noise(rng);
        noisy_camera_pose_2[1] += sensor_noise(rng);
        noisy_camera_pose_2[2] += sensor_noise(rng) * 0.1;
        
        // Feed measurements to state estimator
        state_estimator_2->NewCameraData(noisy_camera_pose_2);
        
        // Also feed motor/gyro data (simulated from commanded velocity)
        Eigen::Vector4d wheel_speeds = Eigen::Vector4d::Zero();  // Simplified
        state_estimator_2->NewMotorsData(wheel_speeds);
        state_estimator_2->NewGyroData(vel_camera[2]);  // Use angular velocity
        
        // Get filtered state estimate
        Eigen::Vector3d state_estimate = state_estimator_2->GetPose();
        
        // Feed state estimate to robot (instead of raw camera)
        robot_state_est->NewCameraData(state_estimate);
        
        // Check if replanning needed (using state estimate)
        Eigen::Vector3d desired_pose_2 = robot_state_est->GetPoseInWorldFrame();
        if (replan_state_est.ShouldReplan(state_estimate, desired_pose_2, current_time)) {
            std::vector<Eigen::Vector3d> new_waypoints;
            new_waypoints.push_back(state_estimate);
            new_waypoints.push_back(Eigen::Vector3d(1.0, 0.4, 0.0));
            
            robot_state_est->SetBSplinePath(new_waypoints, current_time);
            replan_state_est.RecordReplan(current_time);
            std::cout << "  State-est replan from filtered position: " << state_estimate.transpose() << std::endl;
        }
        
        // Get velocity command
        robot_state_est->ControlLogic();
        robot_state_est->SenseLogic();
        Eigen::Vector3d vel_state_est = robot_state_est->GetVelocityInWorldFrame();
        
        // Apply physics to both robots
        Eigen::Vector3d actual_vel_camera = ApplyRealRobotPhysics(vel_camera, rng, physics_noise);
        Eigen::Vector3d actual_vel_state_est = ApplyRealRobotPhysics(vel_state_est, rng, physics_noise);
        
        // Update true positions
        true_pose_camera.head<2>() += actual_vel_camera.head<2>() * dt;
        true_pose_camera[2] += actual_vel_camera[2] * dt;
        
        true_pose_state_est.head<2>() += actual_vel_state_est.head<2>() * dt;
        true_pose_state_est[2] += actual_vel_state_est[2] * dt;
        
        // Calculate errors from straight line
        double error_camera = std::abs(true_pose_camera[1]);
        double error_state_est = std::abs(true_pose_state_est[1] - 0.4);
        
        max_error_camera = std::max(max_error_camera, error_camera);
        max_error_state_est = std::max(max_error_state_est, error_state_est);
        total_error_camera += error_camera;
        total_error_state_est += error_state_est;
        error_samples++;
        
        // Update visualization
        soccer_objects[0].position = true_pose_camera;
        soccer_objects[1].position = true_pose_state_est;
        
        // Print status
        if (current_time - last_print_time > 1.0) {
            std::cout << "\n[Time: " << std::fixed << std::setprecision(1) 
                      << current_time - start_time << "s]" << std::endl;
            std::cout << "Camera replan:    pos=" << std::fixed << std::setprecision(3) 
                      << true_pose_camera.transpose() 
                      << ", Y-error=" << error_camera << "m" << std::endl;
            std::cout << "State-est replan: pos=" << true_pose_state_est.transpose() 
                      << ", Y-error=" << error_state_est << "m" << std::endl;
            std::cout << "State estimate:   " << state_estimate.transpose() 
                      << " (vs true: " << true_pose_state_est.transpose() << ")" << std::endl;
            
            last_print_time = current_time;
        }
        
        // Check if reached goal
        if (true_pose_camera[0] > 0.9 && true_pose_state_est[0] > 0.9) {
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
    std::cout << "\n=== FINAL COMPARISON ====" << std::endl;
    std::cout << "\nCamera-based Replanning:" << std::endl;
    std::cout << "- Final position: " << true_pose_camera.transpose() << std::endl;
    std::cout << "- Max Y-error: " << max_error_camera << "m" << std::endl;
    std::cout << "- Avg Y-error: " << total_error_camera / error_samples << "m" << std::endl;
    std::cout << "- Replans: " << replan_camera.GetReplanCount() << std::endl;
    
    std::cout << "\nState Estimation-based Replanning:" << std::endl;
    std::cout << "- Final position: " << true_pose_state_est.transpose() << std::endl;
    std::cout << "- Max Y-error: " << max_error_state_est << "m" << std::endl;
    std::cout << "- Avg Y-error: " << total_error_state_est / error_samples << "m" << std::endl;
    std::cout << "- Replans: " << replan_state_est.GetReplanCount() << std::endl;
    
    std::cout << "\n=== CONCLUSION ===" << std::endl;
    double improvement = (1.0 - max_error_state_est/max_error_camera) * 100;
    if (improvement > 10) {
        std::cout << "State estimation-based replanning IMPROVED tracking by " 
                  << improvement << "%" << std::endl;
        std::cout << "The Kalman filter helps by:" << std::endl;
        std::cout << "- Filtering noisy measurements" << std::endl;
        std::cout << "- Predicting current state despite sensor delays" << std::endl;
        std::cout << "- Fusing multiple sensor inputs" << std::endl;
    } else if (improvement < -10) {
        std::cout << "State estimation-based replanning performed WORSE" << std::endl;
        std::cout << "Possible reasons:" << std::endl;
        std::cout << "- State estimator needs better tuning" << std::endl;
        std::cout << "- Model mismatch between estimator and real robot" << std::endl;
    } else {
        std::cout << "Both approaches performed similarly" << std::endl;
        std::cout << "The fundamental issue remains: high gain + delays = oscillations" << std::endl;
    }
    
    std::cout << "\n[Demo Complete]" << std::endl;
    return 0;
}