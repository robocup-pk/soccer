#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <iomanip>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "StateEstimator.h"
#include "HardwareManager.h"

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[Demo] Running Kalman Filter Analysis Demo" << std::endl;

    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Initialize State Estimator separately for analysis
    est::StateEstimator state_estimator;
    
    // Initialize another State Estimator for dead reckoning (encoder-only, no camera)
    est::StateEstimator dead_reckoning_estimator;
    
    // Initialize Hardware Manager for wheel speeds
    hw::HardwareManager hardware_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    state_estimator.InitializePose(robot_start_pose);
    dead_reckoning_estimator.InitializePose(robot_start_pose);
    
    vector<Eigen::Vector3d> waypoints;
    robot_manager.GetUniformBSplinePlanner().SetLimits(0.8, 0.5, 0.8, 0.5);
    robot_manager.GetUniformBSplinePlanner().SetFeedbackGains(0.1, 0.05);
    
    // Choose a test case based on command line argument
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    // Camera update frequency in milliseconds
    double camera_update_ms = 50.0;  // 50ms = 20Hz camera rate
    if (argc > 2) {
        camera_update_ms = std::atof(argv[2]);
    }
    
    // Open log file for Kalman filter analysis
    std::ofstream kalman_log("kalman_filter_log.txt");
    kalman_log << "# Kalman Filter Analysis Log" << std::endl;
    kalman_log << "# Format: timestamp(s) actual_x(m) actual_y(m) actual_theta(rad) " 
               << "estimated_x(m) estimated_y(m) estimated_theta(rad) "
               << "dead_reckoning_x(m) dead_reckoning_y(m) dead_reckoning_theta(rad) "
               << "camera_update(0/1) error_x(m) error_y(m) error_theta(rad) "
               << "dr_error_x(m) dr_error_y(m) dr_error_theta(rad)" << std::endl;
    kalman_log << "# Camera Update Period: " << camera_update_ms << " ms" << std::endl;
    
    // Log test case
    kalman_log << "# Test Case: " << test_case << std::endl;
    
    switch (test_case) {
        case 1: {
            // Test 1: Simple triangle path
            std::cout << "Test 1: Triangle trajectory" << std::endl;
            kalman_log << "# Trajectory: Triangle" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.25, 0.4, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
        case 2: {
            // Test 2: Square path
            std::cout << "Test 2: Square trajectory" << std::endl;
            kalman_log << "# Trajectory: Square" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        }
        case 3: {
            // Test 3: Circle
            std::cout << "Test 3: Circular trajectory" << std::endl;
            kalman_log << "# Trajectory: Circle" << std::endl;
            int N = 16;
            double radius = 0.3;
            for (int i = 0; i <= N; ++i) {
                double angle = 2.0 * M_PI * i / N;
                waypoints.push_back(Eigen::Vector3d(
                    radius * std::cos(angle),
                    radius * std::sin(angle),
                    angle
                ));
            }
            break;
        }
        case 4: {
            // Test 4: Straight line
            std::cout << "Test 4: Straight line trajectory" << std::endl;
            kalman_log << "# Trajectory: Straight Line" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.2, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.4, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.6, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.8, 0.0, 0.0));
            break;
        }
        default: {
            // Default: L-shape
            std::cout << "Default: L-shaped trajectory" << std::endl;
            kalman_log << "# Trajectory: L-shape" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            break;
        }
    }
    
    // Log waypoints
    kalman_log << "# WAYPOINTS" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        kalman_log << "# WP " << i << " " << waypoints[i][0] << " " 
                   << waypoints[i][1] << " " << waypoints[i][2] << std::endl;
    }
    
    // Print waypoints
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }
    
    // Use Uniform B-spline trajectory
    std::cout << "Using Uniform B-spline trajectory" << std::endl;
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::UniformBSpline);
    robot_manager.SetUniformBSplinePath(waypoints, util::GetCurrentTime());
    
    kalman_log << "# DATA_START" << std::endl;
    kalman_log << std::fixed << std::setprecision(6);
    
    // Timing
    auto start_time = std::chrono::steady_clock::now();
    auto last_camera_time = start_time;
    int frame_count = 0;
    const int max_frames = 500; // Limit simulation length
    
    // Camera noise parameters - MORE REALISTIC NOISE
    double camera_noise_std_m = 0.02;  // 20mm standard deviation for position (more realistic)
    double camera_noise_std_rad = 0.05;  // ~3 degrees standard deviation for orientation
    
    // Add some bias to simulate systematic camera errors
    double camera_bias_x = 0.005;  // 5mm bias in x
    double camera_bias_y = -0.003; // -3mm bias in y
    double camera_bias_theta = 0.01; // Small orientation bias
    
    std::cout << "Starting simulation with camera updates every " << camera_update_ms << " ms" << std::endl;
    std::cout << "Camera noise: position=" << camera_noise_std_m*1000 << "mm, "
              << "orientation=" << camera_noise_std_rad*180/M_PI << " degrees" << std::endl;
    std::cout << "Camera bias: x=" << camera_bias_x*1000 << "mm, y=" << camera_bias_y*1000 
              << "mm, theta=" << camera_bias_theta*180/M_PI << " deg" << std::endl;
    
    while (frame_count < max_frames) {
        // Run simulation step
        if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
            std::cout << "[Demo] Simulation finished" << std::endl;
            break;
        }
        
        // Process input and update robot state
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Control logic for RobotManager
        robot_manager.ControlLogic();
        
        // Sense logic for RobotManager
        robot_manager.SenseLogic();
        
        // Get current ACTUAL robot state
        Eigen::Vector3d actual_pose = robot_manager.GetPoseInWorldFrame();
        
        // Calculate velocity from position change (finite difference)
        static Eigen::Vector3d prev_pose = actual_pose;
        static double prev_time = util::GetCurrentTime();
        double current_time_vel = util::GetCurrentTime();
        double dt_vel = current_time_vel - prev_time;
        
        Eigen::Vector3d robot_velocity_world;
        if (dt_vel > 0.001) {  // Avoid division by very small dt
            robot_velocity_world[0] = (actual_pose[0] - prev_pose[0]) / dt_vel;
            robot_velocity_world[1] = (actual_pose[1] - prev_pose[1]) / dt_vel;
            robot_velocity_world[2] = util::WrapAngle(actual_pose[2] - prev_pose[2]) / dt_vel;
            prev_pose = actual_pose;
            prev_time = current_time_vel;
        } else {
            robot_velocity_world = Eigen::Vector3d::Zero();
        }
        
        // Convert world frame velocity to body frame
        double theta = actual_pose[2];
        Eigen::Vector3d robot_velocity_body;
        robot_velocity_body[0] = robot_velocity_world[0] * cos(theta) + robot_velocity_world[1] * sin(theta);
        robot_velocity_body[1] = -robot_velocity_world[0] * sin(theta) + robot_velocity_world[1] * cos(theta);
        robot_velocity_body[2] = robot_velocity_world[2];  // Angular velocity is the same
        
        // Debug velocity - commented out for cleaner output
        // static int vel_debug_count = 0;
        // vel_debug_count++;
        // if (vel_debug_count % 20 == 0) {
        //     std::cout << "[VelDebug] World vel: " << robot_velocity_world.transpose() 
        //               << ", Body vel: " << robot_velocity_body.transpose() 
        //               << ", Theta: " << theta << std::endl;
        // }
        
        hardware_manager.SetBodyVelocity(robot_velocity_body);
        
        // Get encoder data
        std::optional<Eigen::Vector4d> motors_rpms = hardware_manager.NewMotorsRpms();
        static int total_reads = 0;
        static int has_value_count = 0;
        total_reads++;
        if (motors_rpms.has_value()) {
            has_value_count++;
            // Feed encoder data to both state estimators
            state_estimator.NewMotorsData(motors_rpms.value());
            dead_reckoning_estimator.NewMotorsData(motors_rpms.value());
            
            // Print prediction step details (after encoder update)
            static int prediction_count = 0;
            prediction_count++;
            if (prediction_count % 50 == 0) {  // Print every 50 predictions
                std::cout << "\n===== PREDICTION STEP (Encoder Update) #" << prediction_count << " =====" << std::endl;
                std::cout << "Position after prediction: " << state_estimator.GetPose().transpose() << std::endl;
                std::cout << "P-Matrix (State Covariance) after prediction:" << std::endl;
                std::cout << state_estimator.GetStateCovariance() << std::endl;
            }
            
            // Debug: Check if dead reckoning is actually updating
            static int encoder_count = 0;
            static int non_zero_count = 0;
            encoder_count++;
            
            // Count non-zero RPMs
            if (motors_rpms.value().norm() > 0.01) {
                non_zero_count++;
            }
            
            // Reduced frequency debug output
            if (encoder_count % 100 == 0) {  // Changed from 20 to 100
                Eigen::Vector3d dr_pose = dead_reckoning_estimator.GetPose();
                Eigen::Vector3d kalman_pose = state_estimator.GetPose();
                std::cout << "\n[Summary] After " << encoder_count << " encoder updates:"
                          << "\n  Non-zero RPMs: " << non_zero_count << "/" << encoder_count
                          << "\n  Kalman pose: " << kalman_pose.transpose()
                          << "\n  Dead Reck pose: " << dr_pose.transpose()
                          << "\n  Actual pose: " << actual_pose.transpose() << std::endl;
            }
        }
        
        // Camera update flag
        int camera_update = 0;
        
        // Calculate current time
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> time_since_last_camera = current_time - last_camera_time;
        
        // Simulate camera measurements at specified frequency (time-based, not frame-based)
        if (time_since_last_camera.count() >= camera_update_ms) {
            // Generate Gaussian noise using Box-Muller transform for better distribution
            double u1 = rand() / (double)RAND_MAX;
            double u2 = rand() / (double)RAND_MAX;
            double gaussian_x = sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
            double gaussian_y = sqrt(-2 * log(u1)) * sin(2 * M_PI * u2);
            
            u1 = rand() / (double)RAND_MAX;
            u2 = rand() / (double)RAND_MAX;
            double gaussian_theta = sqrt(-2 * log(u1)) * cos(2 * M_PI * u2);
            
            // Add noise and bias to simulate real camera measurements
            Eigen::Vector3d camera_measurement = actual_pose;
            camera_measurement[0] += camera_bias_x + camera_noise_std_m * gaussian_x;
            camera_measurement[1] += camera_bias_y + camera_noise_std_m * gaussian_y;
            camera_measurement[2] += camera_bias_theta + camera_noise_std_rad * gaussian_theta;
            
            // Store pose before camera update
            Eigen::Vector3d pose_before_update = state_estimator.GetPose();
            Eigen::Matrix3d P_before = state_estimator.GetStateCovariance();
            
            // Feed camera data ONLY to Kalman filter state estimator (not dead reckoning)
            state_estimator.NewCameraData(camera_measurement);
            
            // Print update step details (after camera measurement)
            static int update_count = 0;
            update_count++;
            std::cout << "\n===== UPDATE STEP (Camera Measurement) #" << update_count << " =====" << std::endl;
            std::cout << "Camera Input: " << camera_measurement.transpose() << std::endl;
            std::cout << "Position before update: " << pose_before_update.transpose() << std::endl;
            std::cout << "Innovation (measurement - prediction): " << state_estimator.GetLastInnovation().transpose() << std::endl;
            std::cout << "Kalman Gain K:" << std::endl;
            std::cout << state_estimator.GetLastKalmanGain() << std::endl;
            std::cout << "Corrected Position: " << state_estimator.GetPose().transpose() << std::endl;
            std::cout << "Innovation Covariance S:" << std::endl;
            std::cout << state_estimator.GetLastInnovationCovariance() << std::endl;
            std::cout << "P-Matrix before update:" << std::endl;
            std::cout << P_before << std::endl;
            std::cout << "P-Matrix after update:" << std::endl;
            std::cout << state_estimator.GetStateCovariance() << std::endl;
            
            // Initialize dead reckoning ONLY on the first camera update
            static bool dead_reckoning_initialized = false;
            if (!dead_reckoning_initialized) {
                // Initialize dead reckoning with the ACTUAL pose (not noisy camera measurement)
                // This simulates perfect initial calibration
                dead_reckoning_estimator.InitializePose(actual_pose);
                // IMPORTANT: Pre-initialize the encoder timing by sending a dummy encoder update
                // This prevents the first real encoder update from being ignored
                dead_reckoning_estimator.NewMotorsData(Eigen::Vector4d::Zero());
                dead_reckoning_initialized = true;
                std::cout << "[Demo] Dead reckoning initialized at pose: " 
                          << actual_pose[0] << ", " << actual_pose[1] << ", " << actual_pose[2] << std::endl;
            }
            
            camera_update = 1;
            last_camera_time = current_time;
        }
        
        // Get ESTIMATED pose from Kalman filter
        Eigen::Vector3d estimated_pose = state_estimator.GetPose();
        
        // Get DEAD RECKONING pose (encoder-only, no camera)
        Eigen::Vector3d dead_reckoning_pose = dead_reckoning_estimator.GetPose();
        
        // Calculate Kalman filter error
        Eigen::Vector3d error = actual_pose - estimated_pose;
        // Normalize angle error
        error[2] = util::WrapAngle(error[2]);
        
        // Calculate dead reckoning error
        Eigen::Vector3d dr_error = actual_pose - dead_reckoning_pose;
        // Normalize angle error
        dr_error[2] = util::WrapAngle(dr_error[2]);
        
        // Calculate timestamp
        std::chrono::duration<double> elapsed = current_time - start_time;
        double timestamp = elapsed.count();
        
        // Log data with dead reckoning included
        kalman_log << timestamp << " " 
                   << actual_pose[0] << " " << actual_pose[1] << " " << actual_pose[2] << " "
                   << estimated_pose[0] << " " << estimated_pose[1] << " " << estimated_pose[2] << " "
                   << dead_reckoning_pose[0] << " " << dead_reckoning_pose[1] << " " << dead_reckoning_pose[2] << " "
                   << camera_update << " "
                   << error[0] << " " << error[1] << " " << error[2] << " "
                   << dr_error[0] << " " << dr_error[1] << " " << dr_error[2]
                   << std::endl;
        
        // Update soccer objects with current robot pose
        soccer_objects[0].position = actual_pose;
        
        frame_count++;
        
        // Print progress every 100 frames
        if (frame_count % 100 == 0) {
            std::cout << "Frame " << frame_count << ":" << std::endl;
            std::cout << "  Kalman Error (mm): [" << error[0]*1000 << ", " << error[1]*1000 
                      << "], (deg): " << error[2]*180/M_PI << std::endl;
            std::cout << "  Dead Reckoning Error (mm): [" << dr_error[0]*1000 << ", " << dr_error[1]*1000 
                      << "], (deg): " << dr_error[2]*180/M_PI << std::endl;
        }
        
        // Check if we should exit (press ESC in the window)
        if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }
    }
    
    kalman_log.close();
    std::cout << "[Demo] Kalman filter data saved to kalman_filter_log.txt" << std::endl;
    std::cout << "[Demo] Recorded " << frame_count << " frames" << std::endl;
    
    // Calculate and print summary statistics
    std::cout << "\nRun 'python3 plot_kalman_analysis.py' to visualize the results" << std::endl;
    
    return 0;
}