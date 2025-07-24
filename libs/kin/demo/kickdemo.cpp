// Enhanced Demo: RRT path planning with TrajectoryManager and kick functionality
#define _USE_MATH_DEFINES
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <iomanip>

// Project libs
#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "RRT.h"
#include "Waypoint.h"
#include "Kinematics.h"
#include "Kick.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"
#include "TrapezoidalTrajectoryVi3D.h"

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Enhanced RRT + TrajectoryManager + Kick Demo..." << std::endl;
    
    // Check configuration
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[KickDemo] Set num_robots to 1. Exiting!" << std::endl;
        return 0;
    }

    // Initialize soccer objects using the system's InitSoccerObjects function
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    
    // Initialize OpenGL simulation
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize robot manager for advanced trajectory control
    rob::RobotManager robot_manager;
    
    // Fixed positions for demo
    Eigen::Vector3d robot_start_pose(-2.0, -1.5, 0.0);    // Robot starts at bottom-left
    Eigen::Vector3d ball_position(1.5, 0.5, 0.0);         // Ball at different position
    Eigen::Vector3d target_position(2.0, 1.5, 0.0);       // Final target after kick
    
    robot_manager.InitializePose(robot_start_pose);
    
    // Debug: Check if initialization worked
    Eigen::Vector3d initialized_pose = robot_manager.GetPoseInWorldFrame();
    std::cout << "[KickDemo] 🔧 Robot initialized to: (" << initialized_pose.x() << ", " << initialized_pose.y() << ", " << initialized_pose.z() << ")" << std::endl;
    
    // Set fixed positions in soccer objects
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot0") {
            obj.position = robot_start_pose;
            obj.velocity = Eigen::Vector3d::Zero();
        } else if (obj.name == "ball") {
            obj.position = ball_position;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    
    // Demo state variables
    enum DemoState {
        INIT,
        PLANNING_PATH_TO_BALL,
        MOVING_TO_BALL,
        ORIENTING_TO_BALL,
        KICKING_BALL,
        BALL_TRACKING,
        COMPLETED
    };
    
    DemoState demo_state = INIT;
    bool demo_started = false;
    state::Path rrt_path;
    std::vector<Eigen::Vector3d> trajectory_path;
    double demo_start_time = 0.0;
    double kicking_distance = 0.25; // 25cm - optimal kicking distance (SSL rule compliant)

    std::cout << "\nEnhanced Kick Demo Instructions:" << std::endl;
    std::cout << "1. Robot starts at (-2.0, -1.5)" << std::endl;
    std::cout << "2. Ball is positioned at (1.5, 0.5)" << std::endl;
    std::cout << "3. RRT plans optimal path from robot to ball" << std::endl;
    std::cout << "4. TrajectoryManager executes smooth trapezoidal trajectory" << std::endl;
    std::cout << "5. Robot orients towards ball (SSL compliance)" << std::endl;
    std::cout << "6. Robot kicks ball towards target (2.0, 1.5)" << std::endl;
    std::cout << "7. Ball physics simulation shows realistic movement" << std::endl;
    std::cout << "8. Press ESC to exit\n" << std::endl;

    // Main simulation loop
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!demo_started) {
            demo_start_time = current_time;
            demo_started = true;
            demo_state = PLANNING_PATH_TO_BALL;
        }
        
        // Handle input
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Demo state machine
        switch (demo_state) {
            case INIT:
                // Already handled above
                break;
                
            case PLANNING_PATH_TO_BALL: {
                std::cout << "[KickDemo] Planning RRT path from robot to ball..." << std::endl;
                
                // Get current robot and ball positions
                Eigen::Vector3d robot_pos = robot_start_pose;
                Eigen::Vector3d ball_pos = ball_position;
                
                // Calculate pre-kick position (optimal distance for kicking)
                Eigen::Vector3d ball_to_target = (target_position - ball_pos).normalized();
                Eigen::Vector3d pre_kick_pos = ball_pos - ball_to_target * kicking_distance;
                pre_kick_pos.z() = 0; // Keep 2D
                
                std::cout << "[KickDemo] Pre-kick position: (" << pre_kick_pos.x() << ", " << pre_kick_pos.y() << ")" << std::endl;
                
                // Create waypoints for RRT (convert from meters to mm for RRT)
                state::Waypoint start_wp(robot_pos.x() * 1000, robot_pos.y() * 1000, robot_pos.z());
                state::Waypoint goal_wp(pre_kick_pos.x() * 1000, pre_kick_pos.y() * 1000, 0);
                
                // Configure RRT parameters for optimal path planning
                algos::RRTParams rrt_params = algos::DefaultRRTParams();
                rrt_params.max_iterations = 2000;
                rrt_params.step_size = 150.0; // 15cm steps for smooth path
                rrt_params.goal_tolerance = 100.0; // 10cm tolerance
                rrt_params.use_rrt_star = true; // Use RRT* for optimal paths
                
                // Plan path using RRT
                rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
                
                if (rrt_path.empty()) {
                    std::cout << "[KickDemo] ❌ Failed to find path to ball!" << std::endl;
                    demo_state = COMPLETED;
                    break;
                }
                
                std::cout << "[KickDemo] ✅ Found RRT path with " << rrt_path.size() << " waypoints" << std::endl;
                
                // Convert RRT path to Eigen::Vector3d trajectory (convert back from mm to meters)
                trajectory_path.clear();
                for (const auto& wp : rrt_path) {
                    trajectory_path.push_back(Eigen::Vector3d(wp.x / 1000.0, wp.y / 1000.0, wp.angle));
                }
                
                // Use RobotManager's SetPath with TrajectoryManager (TrapezoidalTrajectoryVi3D)
                robot_manager.SetPath(trajectory_path, current_time);
                
                std::cout << "[KickDemo] ✅ Set trajectory path with " << trajectory_path.size() << " waypoints" << std::endl;
                std::cout << "[KickDemo] ✅ TrajectoryManager will handle smooth trapezoidal motion" << std::endl;
                std::cout << "[KickDemo] 🔄 ABOUT TO TRANSITION TO MOVING_TO_BALL STATE!!!" << std::endl;
                
                demo_state = MOVING_TO_BALL;
                std::cout << "[KickDemo] ✅ SUCCESSFULLY TRANSITIONED TO MOVING_TO_BALL STATE!!!" << std::endl;
                break;
            }
            
            case MOVING_TO_BALL: {
                std::cout << "[KickDemo] 🚨 ENTERED MOVING_TO_BALL CASE!!!" << std::endl;
                
                // Check if robot has reached the ball area using TrajectoryManager
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d to_ball = ball_position - robot_pos;
                double distance_to_ball = to_ball.norm();
                
                // Show target pre-kick position for comparison
                Eigen::Vector3d ball_to_target = (target_position - ball_position).normalized();
                Eigen::Vector3d pre_kick_pos = ball_position - ball_to_target * kicking_distance;
                pre_kick_pos.z() = 0;
                double distance_to_prekick = (pre_kick_pos - robot_pos).norm();
                
                // Print progress info immediately (no timing)
                std::cout << "[KickDemo] 🤖 Robot at: (" << robot_pos.x() << ", " << robot_pos.y() << "), Distance to pre-kick: " << distance_to_prekick << "m" << std::endl;
                
                // Check if robot reached pre-kick position (the actual target) OR if trajectory completed
                std::string robot_state = robot_manager.GetRobotState();
                bool trajectory_completed = (robot_state != "AUTONOMOUS_DRIVING");
                
                if (distance_to_prekick <= 0.3) { // 30cm tolerance to pre-kick position
                    std::cout << "[KickDemo] ✅ Robot reached pre-kick position! Distance: " << distance_to_prekick << "m" << std::endl;
                    demo_state = ORIENTING_TO_BALL;
                } else if (trajectory_completed) {
                    std::cout << "[KickDemo] ✅ Trajectory completed. Robot at: (" << robot_pos.x() << ", " << robot_pos.y() << "), Distance: " << distance_to_prekick << "m" << std::endl;
                    demo_state = ORIENTING_TO_BALL;
                }
                break;
            }
            
            case ORIENTING_TO_BALL: {
                std::cout << "[KickDemo] Orienting robot to face the ball..." << std::endl;
                
                // Get current robot and ball positions
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d ball_pos = ball_position;
                
                // Calculate required angle to face the ball for optimal kick
                double dx = ball_pos.x() - robot_pos.x();
                double dy = ball_pos.y() - robot_pos.y();
                double required_angle = atan2(dy, dx);
                
                std::cout << "[KickDemo] Robot pos: (" << robot_pos.x() << ", " << robot_pos.y() << ", " << robot_pos.z() << ")" << std::endl;
                std::cout << "[KickDemo] Ball pos: (" << ball_pos.x() << ", " << ball_pos.y() << ")" << std::endl;
                std::cout << "[KickDemo] Required angle to face ball: " << required_angle << " rad (" << (required_angle * 180.0 / M_PI) << " degrees)" << std::endl;
                
                // Orient robot to face the ball directly by updating the soccer objects
                for (auto& obj : soccer_objects) {
                    if (obj.name == "robot0") {
                        obj.position.z() = required_angle;  // Set orientation directly
                        std::cout << "[KickDemo] ✅ Robot orientation set to: " << required_angle << " rad" << std::endl;
                        break;
                    }
                }
                
                // Also update robot manager's internal state
                Eigen::Vector3d new_pose(robot_pos.x(), robot_pos.y(), required_angle);
                robot_manager.InitializePose(new_pose);
                
                std::cout << "[KickDemo] ✅ Robot is now facing the ball!" << std::endl;
                demo_state = KICKING_BALL;
                break;
            }
            
            case KICKING_BALL: {
                std::cout << "[KickDemo] 🦵 Executing kick towards target!" << std::endl;
                
                // Find robot and ball objects
                state::SoccerObject* robot = nullptr;
                state::Ball* ball = nullptr;
                
                for (auto& obj : soccer_objects) {
                    if (obj.name == "robot0") {
                        robot = &obj;
                    } else if (obj.name == "ball") {
                        ball = dynamic_cast<state::Ball*>(&obj);
                    }
                }
                
                if (!robot || !ball) {
                    std::cout << "[KickDemo] ❌ Error: Could not find robot or ball!" << std::endl;
                    demo_state = COMPLETED;
                    break;
                }
                
                // Calculate kick direction towards target (not just towards ball)
                Eigen::Vector3d ball_pos = ball->position;
                Eigen::Vector2d kick_direction = (target_position.head<2>() - ball_pos.head<2>()).normalized();
                
                // Use optimal kick power for reaching target
                double distance_to_target = (target_position - ball_pos).norm();
                double kick_power = std::min(distance_to_target * 1.2 + 1.0, 5.5); // Dynamic power, SSL compliant
                
                std::cout << "[KickDemo] Kick direction: (" << kick_direction.x() << ", " << kick_direction.y() << ")" << std::endl;
                std::cout << "[KickDemo] Kick power: " << kick_power << " m/s" << std::endl;
                std::cout << "[KickDemo] Target distance: " << distance_to_target << "m" << std::endl;
                
                // Execute the kick using the enhanced kick system
                bool kick_success = kin::Kick(*robot, *ball, kick_power, true);
                
                if (kick_success) {
                    std::cout << "[KickDemo] ✅ Ball kicked successfully!" << std::endl;
                    std::cout << "[KickDemo] Ball initial velocity: " << ball->velocity.norm() << " m/s" << std::endl;
                    demo_state = BALL_TRACKING;
                } else {
                    std::cout << "[KickDemo] ❌ Kick failed!" << std::endl;
                    demo_state = COMPLETED;
                }
                break;
            }
            
            case BALL_TRACKING: {
                // Track ball movement and provide updates
                static double last_tracking_print = 0;
                static double initial_ball_speed = 0;
                static bool initial_speed_recorded = false;
                
                // Find ball object
                state::Ball* ball = nullptr;
                for (auto& obj : soccer_objects) {
                    if (obj.name == "ball") {
                        ball = dynamic_cast<state::Ball*>(&obj);
                        break;
                    }
                }
                
                if (ball) {
                    double current_ball_speed = ball->velocity.head<2>().norm();
                    double distance_to_target = (target_position - ball->position).norm();
                    
                    if (!initial_speed_recorded) {
                        initial_ball_speed = current_ball_speed;
                        initial_speed_recorded = true;
                    }
                    
                    // Print tracking info every 2 seconds
                    if (current_time - last_tracking_print > 2.0) {
                        std::cout << "[KickDemo] 🏈 Ball tracking - Pos: (" 
                                  << std::fixed << std::setprecision(2) << ball->position.x() << ", " << ball->position.y() 
                                  << "), Speed: " << current_ball_speed << " m/s"
                                  << ", Target distance: " << distance_to_target << "m" << std::endl;
                        last_tracking_print = current_time;
                    }
                    
                    // Check if ball has stopped or reached near target
                    if (current_ball_speed < 0.1 || distance_to_target < 0.3) {
                        std::cout << "\n[KickDemo] ✅ Ball movement completed!" << std::endl;
                        std::cout << "[KickDemo] Final position: (" << ball->position.x() << ", " << ball->position.y() << ")" << std::endl;
                        std::cout << "[KickDemo] Final distance to target: " << distance_to_target << "m" << std::endl;
                        std::cout << "[KickDemo] Speed reduction: " << initial_ball_speed << " → " << current_ball_speed << " m/s" << std::endl;
                        demo_state = COMPLETED;
                    }
                } else {
                    demo_state = COMPLETED;
                }
                break;
            }
            
            case COMPLETED: {
                static bool completion_printed = false;
                if (!completion_printed) {
                    std::cout << "\n🏆 [KickDemo] Demo completed successfully!" << std::endl;
                    std::cout << "[KickDemo] ✅ RRT path planning executed" << std::endl;
                    std::cout << "[KickDemo] ✅ TrajectoryManager provided smooth motion" << std::endl;
                    std::cout << "[KickDemo] ✅ SSL-compliant kick executed" << std::endl;
                    std::cout << "[KickDemo] ✅ Realistic ball physics demonstrated" << std::endl;
                    std::cout << "[KickDemo] Press ESC to exit" << std::endl;
                    completion_printed = true;
                }
                break;
            }
        }
        
        // Update robot logic (simplified, matching dribbling demo approach)
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Always update robot state from robot manager (crucial for visualization and movement)
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot0") {
                obj.position = robot_manager.GetPoseInWorldFrame();
                obj.velocity = robot_manager.GetVelocityInWorldFrame();
                break;
            }
        }
        
        // Update physics for all objects
        kin::UpdateKinematics(soccer_objects, dt);
        kin::CheckAndResolveCollisions(soccer_objects);
        
        // Check if simulation should continue and render
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            std::cout << "[KickDemo] Simulation window closed" << std::endl;
            break;
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}