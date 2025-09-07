#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"
#include "Kick.h"
// Pure Sumatra system includes
#include "CircularObstacle.h"
#include "AdvancedMotionPlanner.h"
#include "MoveConstraints.h"

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[Demo] Running RobotManager demo with trajectory logging" << std::endl;
    
    // Initialize random seed for drift simulation
    srand(static_cast<unsigned int>(time(nullptr)));

    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    vector<Eigen::Vector3d> waypoints;
    
    std::cout << "[Demo] Using TIGERs trajectory planning system" << std::endl;
    
    // Choose a test case based on command line argument
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    // Open log file for trajectory data
    std::ofstream trajectory_log("trajectory_log.txt");
    trajectory_log << "# Trajectory Log File" << std::endl;
    trajectory_log << "# Format: timestamp(s) x(m) y(m) theta(rad) vx(m/s) vy(m/s) omega(rad/s)" << std::endl;
    
    // Log waypoints
    trajectory_log << "# WAYPOINTS" << std::endl;
    
    switch (test_case) {
        case 1: {
            // Test 1: Straight line trajectory
            std::cout << "Test 1: Straight line trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.8, -0.6, 0));
            waypoints.push_back(Eigen::Vector3d(1.0, -0.4, 0.785));
            waypoints.push_back(Eigen::Vector3d(1.2, -0.6, 0));
            waypoints.push_back(Eigen::Vector3d(1.0, -0.8, -0.785));
            waypoints.push_back(Eigen::Vector3d(0.8, -0.6, 0));
            break;
        }
        case 2: {
            // Test 2: L-shaped path (90-degree turn)
            std::cout << "Test 2: L-shaped path (90-degree turn)" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            break;
        }
        case 3: {
            // Test 3: Square path
            std::cout << "Test 3: Square path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        }
        case 4: {
            // Test 4: Circular path
            std::cout << "Test 4: Circular path" << std::endl;
            int N = 16;
            double radius = 0.5;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
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
        case 5: {
            // Test 5: S-curve trajectory
            std::cout << "Test 5: S-curve trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.8, 0.2, 0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.4, 0.785));
            waypoints.push_back(Eigen::Vector3d(1.2, 0.2, -0.785));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, -2.356));
            waypoints.push_back(Eigen::Vector3d(0.8, 0.2, 0.785));
            break;
        }
        case 6: {
            // Test 6: Sharp zigzag (stress test for corners)
            std::cout << "Test 6: Sharp zigzag trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.3, 0.3, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.6, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.9, 0.3, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.2, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.5, 0.3, M_PI/4));
            break;
        }
        case 7: {
            // Test 7: Lane change maneuver
            std::cout << "Test 7: Lane change maneuver" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.3, M_PI/8));
            waypoints.push_back(Eigen::Vector3d(1.5, 0.5, 0.0));
            waypoints.push_back(Eigen::Vector3d(2.0, 0.5, 0.0));
            waypoints.push_back(Eigen::Vector3d(2.5, 0.5, 0.0));
            break;
        }
        case 8: {
            // Test 8: Figure-8 pattern
            std::cout << "Test 8: Figure-8 pattern" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.25, 0.25, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(-0.25, 0.25, 3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.25, -0.25, -3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, -0.5, -M_PI/2));
            waypoints.push_back(Eigen::Vector3d(-0.25, -0.25, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
        case 9: {
            // Test 9: Star pattern (multiple sharp turns)
            std::cout << "Test 9: Star pattern" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            for (int i = 0; i < 5; ++i) {
                double angle = 2.0 * M_PI * i / 5;
                waypoints.push_back(Eigen::Vector3d(
                    0.5 * std::cos(angle),
                    0.5 * std::sin(angle),
                    angle
                ));
                // Inner point
                double inner_angle = angle + 2.0 * M_PI / 10;
                waypoints.push_back(Eigen::Vector3d(
                    0.2 * std::cos(inner_angle),
                    0.2 * std::sin(inner_angle),
                    inner_angle
                ));
            }
            break;
        }
        case 10: {
            // Test 10: Improved spiral trajectory with smooth velocity-based orientations
            std::cout << "Test 10: Smooth spiral trajectory" << std::endl;
            int N = 16;  // Fewer waypoints for smoother motion
            std::vector<Eigen::Vector3d> spiral_waypoints;
            //spiral_waypoints.push_back(Eigen::Vector3d(0,0,0));
            //spiral_waypoints.push_back(Eigen::Vector3d(0.5,0,0));
            for (int i = 0; i <= N; ++i) {
                double angle = 1.5 * M_PI * i / N;  // Reduced to 0.75 rotations (270°)
                double radius = 0.1 + 0.3 * i / N;  // Radius from 0.1 to 0.4 (smaller for safety)
                
                // Position
                double x = radius * std::cos(angle);
                double y = radius * std::sin(angle);
                
                // Calculate orientation based on velocity direction (tangent to spiral)
                double theta;
                if (i == 0) {
                    theta = 0.0;  // Start facing forward
                } else {
                    // Compute tangent vector (derivative of spiral)
                    double dr_dangle = 0.3 / (1.5 * M_PI);  // radius change rate
                    double dx_dangle = dr_dangle * std::cos(angle) - radius * std::sin(angle);
                    double dy_dangle = dr_dangle * std::sin(angle) + radius * std::cos(angle);
                    theta = std::atan2(dy_dangle, dx_dangle);
                }
                
                spiral_waypoints.push_back(Eigen::Vector3d(x, y, theta));
            }
            
            waypoints = spiral_waypoints;
            break;
        }
        case 11: {
            // Test 11: PURE Sumatra PathFinder - Direct path (no obstacles)
            std::cout << "========================================" << std::endl;
            std::cout << "Test 11: PURE Sumatra PathFinder System" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "[SUMATRA] Direct path with no obstacles" << std::endl;
            
            // Don't use waypoints! Use pure Sumatra planTrajectory approach
            waypoints.clear(); // Clear waypoints - we don't use them in pure Sumatra!
            
            // Instead, we'll directly call planTrajectory in the execution loop
            std::cout << "[SUMATRA] Will call: planTrajectory(start, vel, destination, obstacles, constraints)" << std::endl;
            std::cout << "[SUMATRA] Start: (0,0,0) -> Destination: (1.2, 0.8, π/4)" << std::endl;
            std::cout << "[SUMATRA] Obstacles: None (direct path)" << std::endl;
            break;
        }
        case 12: {
            // Test 12: PURE Sumatra PathFinder - With circular obstacles
            std::cout << "========================================" << std::endl;
            std::cout << "Test 12: Sumatra PathFinder + Obstacles" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "[SUMATRA] Path with circular obstacles blocking direct route" << std::endl;
            
            waypoints.clear(); // No waypoints in pure Sumatra!
            
            std::cout << "[SUMATRA] Will create circular obstacles at strategic positions" << std::endl;
            std::cout << "[SUMATRA] Start: (0,0,0) -> Destination: (2.0, 1.0, π/2)" << std::endl;
            std::cout << "[SUMATRA] PathFinder will automatically generate waypoints around obstacles!" << std::endl;
            break;
        }
        case 13: {
            // Test 13: PURE Sumatra PathFinder - Complex obstacle field
            std::cout << "========================================" << std::endl;
            std::cout << "Test 13: Complex Obstacle Navigation" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "[SUMATRA] Complex obstacle field with multiple barriers" << std::endl;
            
            waypoints.clear(); // Pure Sumatra approach!
            
            std::cout << "[SUMATRA] Will create multiple obstacles forming a complex field" << std::endl;
            std::cout << "[SUMATRA] Start: (0,0,0) -> Destination: (1.5, 1.5, π)" << std::endl;
            std::cout << "[SUMATRA] Testing advanced obstacle avoidance capabilities!" << std::endl;
            break;
        }
        case 14: {
            // Test 14: SIMPLE obstacle avoidance test
            std::cout << "========================================" << std::endl;
            std::cout << "Test 14: Simple Obstacle Avoidance Test" << std::endl;  
            std::cout << "========================================" << std::endl;
            std::cout << "[SUMATRA] SIMPLE TEST: Two obstacles blocking direct path" << std::endl;
            
            waypoints.clear(); // Pure Sumatra approach!
            
            std::cout << "[SUMATRA] Start: (0, 0, 0) -> Destination: (1.0, 0, 0)" << std::endl;
            std::cout << "[SUMATRA] Obstacle 1: (0.5, 0, 0) - DIRECTLY in path!" << std::endl;
            std::cout << "[SUMATRA] Obstacle 2: (0.5, 0.5, 0) - offset from path" << std::endl;
            std::cout << "[SUMATRA] Robot should go around both obstacles!" << std::endl;
            break;
        }
        default: {
            // Default: Forward and back trajectory
            std::cout << "Default: Forward and back trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
    }
    
    // Log waypoints to file
    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_log << "# WP " << i << " " << waypoints[i][0] << " " 
                      << waypoints[i][1] << " " << waypoints[i][2] << std::endl;
    }
    
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }
    
    // Use TIGERs trajectory system (type 4 = TIGERs BangBang)
    trajectory_log << "# TRAJECTORY_TYPE 4" << std::endl;
    
    std::cout << "Using TIGERs-style Advanced Motion Planning + Trajectory Tracking" << std::endl;
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::TIGERsTrajectory);
    
    // PURE SUMATRA TEST: Handle empty waypoints (cases 11-13)
    if (waypoints.empty()) {
        std::cout << "\n=== PURE SUMATRA PATHFINDER TEST ===" << std::endl;
        std::cout << "No waypoints provided - using direct planTrajectory() call!" << std::endl;
        
        // We'll call planTrajectory directly in the simulation loop based on test case
        std::cout << "Will call planTrajectory with obstacles and constraints in simulation loop" << std::endl;
    } else {
        // Traditional waypoint-based approach
        robot_manager.SetSmoothPathTrackerPath(waypoints, util::GetCurrentTime());
    }
    
    trajectory_log << "# DATA_START" << std::endl;
    
    // Timing
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    const int max_frames = 1000; // Limit to prevent infinite logging
    
    // For DB-RRT tracking
    int current_waypoint_idx = 1;
    bool db_rrt_finished = true;
    
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
        
        // PURE SUMATRA PATHFINDER TEST: Call planTrajectory directly for cases 11-13
        if (waypoints.empty() && frame_count == 10) { // Call once after initial settling
            std::cout << "\n=== CALLING PURE SUMATRA PATHFINDER ===" << std::endl;
            
            // Create AdvancedMotionPlanner directly (bypass RobotManager waypoint system)
            ctrl::AdvancedMotionPlanner pure_sumatra_planner;
            
            // Create obstacles and constraints based on test case
            std::vector<std::shared_ptr<ctrl::IObstacle>> obstacles;
            ctrl::MoveConstraints constraints;
            
            // Configure constraints (EXACT Sumatra defaults)
            constraints.setVelMax(1.2)      // m/s
                      .setAccMax(2.0)      // m/s²  
                      .setVelMaxW(6.0)     // rad/s
                      .setAccMaxW(15.0);   // rad/s²
            
            Eigen::Vector3d start_pos(0.0, 0.0, 0.0);
            Eigen::Vector3d start_vel = robot_manager.GetVelocityInWorldFrame();
            Eigen::Vector3d destination;
            
            if (test_case == 11) {
                // Test 11: Direct path (no obstacles)
                destination = Eigen::Vector3d(1.2, 0.8, M_PI/4);
                std::cout << "[SUMATRA] Test 11: Direct path to (1.2, 0.8, π/4)" << std::endl;
                
            } else if (test_case == 12) {
                // Test 12: Single obstacle blocking direct path
                destination = Eigen::Vector3d(2.0, 1.0, M_PI/2);
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(1.0, 0.5), 0.3, "Obstacle1"));
                std::cout << "[SUMATRA] Test 12: Path to (2.0, 1.0, π/2) with 1 circular obstacle" << std::endl;
                
            } else if (test_case == 13) {
                // Test 13: Complex obstacle field
                destination = Eigen::Vector3d(1.5, 1.5, M_PI);
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(0.5, 0.5), 0.2, "Wall1"));
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(1.0, 0.8), 0.15, "Wall2"));
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(0.8, 1.2), 0.25, "Wall3"));
                std::cout << "[SUMATRA] Test 13: Path to (1.5, 1.5, π) with 3 obstacles" << std::endl;
                
            } else if (test_case == 14) {
                // Test 14: SIMPLE obstacle avoidance test
                destination = Eigen::Vector3d(1.0, 0.0, 0.0);
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(0.5, 0.0), 0.1, "DirectBlock"));
                obstacles.push_back(std::make_shared<ctrl::CircularObstacle>(Eigen::Vector2d(0.5, 0.5), 0.1, "SideBlock"));
                std::cout << "[SUMATRA] Test 14: SIMPLE path from (0,0,0) to (1,0,0)" << std::endl;
                std::cout << "[SUMATRA] Obstacle 1 'DirectBlock': (0.5, 0.0) radius=0.1 - BLOCKS direct path" << std::endl;
                std::cout << "[SUMATRA] Obstacle 2 'SideBlock': (0.5, 0.5) radius=0.1 - Forces detour" << std::endl;
            }
            
            std::cout << "[SUMATRA] Calling: planTrajectory(pos, vel, dest, " << obstacles.size() << " obstacles, constraints)" << std::endl;
            
            // THIS IS THE PURE SUMATRA CALL!
            pure_sumatra_planner.planTrajectory(
                start_pos,      // Current robot position
                start_vel,      // Current robot velocity  
                destination,    // Final destination
                obstacles,      // Obstacles for avoidance
                constraints     // Sumatra-style constraints
            );
            
            if (pure_sumatra_planner.isValid()) {
                std::cout << "[SUMATRA] SUCCESS! Generated trajectory with duration: " 
                          << pure_sumatra_planner.getTotalTime() << "s" << std::endl;
                
                // CLEAN SOLUTION: Use RobotManager's direct Sumatra trajectory method
                std::cout << "[SUMATRA] Setting Sumatra trajectory directly via RobotManager..." << std::endl;
                robot_manager.SetSumatraTrajectory(pure_sumatra_planner);
                std::cout << "[SUMATRA] Robot should now follow obstacle-aware trajectory!" << std::endl;
                
            } else {
                std::cout << "[SUMATRA] FAILED! Could not generate valid trajectory" << std::endl;
            }
        }
        
        // Get current robot state
        Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();
        Eigen::Vector3d current_velocity = robot_manager.GetVelocityInWorldFrame();
        
        // Realistic RoboCup SSL noise simulation
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<double> vision_noise(0.0, 0.005);  // 5mm std dev for SSL vision
        static std::normal_distribution<double> orientation_noise(0.0, 0.02);  // 0.02 rad (~1.1°) for orientation
        static std::uniform_real_distribution<double> dropout_prob(0.0, 1.0);
        static int frames_since_last_vision = 0;
        static Eigen::Vector3d last_vision_pose = current_pose;
        static double accumulated_drift = 0.0;
        
        // Simulate realistic SSL conditions
        frames_since_last_vision++;
        
        // Vision system updates (SSL camera runs at ~60Hz, we simulate ~50Hz with occasional dropouts)
        bool vision_available = (frames_since_last_vision >= 1) && (dropout_prob(gen) > 0.05); // 5% dropout rate
        
        if (vision_available) {
            frames_since_last_vision = 0;
            
            // Add realistic vision noise
            Eigen::Vector3d noisy_vision_pose = current_pose;
            noisy_vision_pose[0] += vision_noise(gen);  // X position noise
            noisy_vision_pose[1] += vision_noise(gen);  // Y position noise  
            noisy_vision_pose[2] += orientation_noise(gen);  // Orientation noise
            
            // Simulate state estimation drift between vision updates (IMU drift, wheel slip, etc.)
            accumulated_drift += 0.001 * frames_since_last_vision;  // 1mm drift per frame without vision
            noisy_vision_pose[0] += accumulated_drift * (gen() % 3 - 1);  // Random drift direction
            noisy_vision_pose[1] += accumulated_drift * (gen() % 3 - 1);
            
            last_vision_pose = noisy_vision_pose;
        }
        
        // The TIGERs system uses robust PID feedback control that automatically 
        // handles tracking errors without needing explicit replanning
        
        // Calculate timestamp
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        double timestamp = elapsed.count();
        
        // Log trajectory data
        trajectory_log << timestamp << " " 
                      << current_pose[0] << " " << current_pose[1] << " " << current_pose[2] << " "
                      << current_velocity[0] << " " << current_velocity[1] << " " << current_velocity[2] 
                      << std::endl;
        
        // Update soccer objects with current robot pose
        soccer_objects[0].position = current_pose;
        
        frame_count++;
        
        // Check if we should exit (press ESC in the window)
        if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }
    }
    
    trajectory_log.close();
    std::cout << "[Demo] Trajectory data saved to trajectory_log.txt" << std::endl;
    std::cout << "[Demo] Recorded " << frame_count << " frames" << std::endl;
    
    std::cout << "[Demo] TIGERs trajectory execution completed" << std::endl;
    
    return 0;
}
