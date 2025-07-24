// Demo application to test RRT path planning with dribbling functionality
#define _USE_MATH_DEFINES
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>

// Project libs
#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "RRT.h"
#include "Waypoint.h"
#include "Kinematics.h"
#include "Dribble.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"
#include "TrapezoidalTrajectoryVi3D.h"

int main(int argc, char* argv[]) {
    std::cout << "[DribblingDemo] Starting RRT + Trapezoidal Trajectory Dribbling Demo..." << std::endl;
    
    // Check configuration
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[DribblingDemo] Set num_robots to 1. Exiting!" << std::endl;
        return 0;
    }

    // Initialize soccer objects (robot and ball)
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    
    // Initialize OpenGL simulation
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize robot manager
    rob::RobotManager robot_manager;
    
    // Fixed positions for demo
    Eigen::Vector3d robot_start_pose(-2.0, -1.5, 0);    // Robot starts at bottom-left
    Eigen::Vector3d ball_position(0.0, 0.0, 0);         // Ball at center
    Eigen::Vector3d target_position(2.0, 1.5, 0);       // Target at top-right
    
    robot_manager.InitializePose(robot_start_pose);
    
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
        STARTING_DRIBBLE,
        PLANNING_DRIBBLE_PATH,
        DRIBBLING_TO_TARGET,
        COMPLETED
    };
    
    DemoState demo_state = INIT;
    bool demo_started = false;
    state::Path rrt_path;
    std::vector<Eigen::Vector3d> trajectory_path;
    double demo_start_time = 0.0;
    double approach_distance = 0.3; // 30cm - distance to start dribbling
    bool is_dribbling = false;
    double orientation_start_time = 0.0;
    
    std::cout << "\nDemo Instructions:" << std::endl;
    std::cout << "1. Robot starts at (-2.0, -1.5)" << std::endl;
    std::cout << "2. Ball is positioned at (0.0, 0.0)" << std::endl;
    std::cout << "3. Target is at (2.0, 1.5)" << std::endl;
    std::cout << "4. Robot uses RRT to plan path to ball" << std::endl;
    std::cout << "5. Robot approaches ball" << std::endl;
    std::cout << "6. Robot orients to face the ball (SSL rule compliance)" << std::endl;
    std::cout << "7. Robot starts dribbling the ball" << std::endl;
    std::cout << "8. Robot uses RRT to plan path to target while dribbling" << std::endl;
    std::cout << "9. Robot dribbles ball to target" << std::endl;
    std::cout << "10. Press ESC to exit\n" << std::endl;

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
                std::cout << "[DribblingDemo] Planning RRT path from robot to ball..." << std::endl;
                
                // Get current robot and ball positions
                Eigen::Vector3d robot_pos = robot_start_pose;
                Eigen::Vector3d ball_pos = ball_position;
                
                // Create waypoints for RRT (convert from meters to mm for RRT)
                state::Waypoint start_wp(robot_pos.x() * 1000, robot_pos.y() * 1000, robot_pos.z());
                state::Waypoint goal_wp(ball_pos.x() * 1000, ball_pos.y() * 1000, 0);
                
                // Configure RRT parameters
                algos::RRTParams rrt_params = algos::DefaultRRTParams();
                rrt_params.max_iterations = 1000;
                rrt_params.step_size = 100.0; // 10cm steps
                rrt_params.goal_tolerance = 300.0; // 30cm tolerance (same as approach distance)
                rrt_params.use_rrt_star = true; // Use RRT* for smoother paths
                
                // Plan path using RRT
                rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
                
                if (rrt_path.empty()) {
                    std::cout << "[DribblingDemo] Failed to find path to ball!" << std::endl;
                    demo_state = COMPLETED;
                    break;
                }
                
                std::cout << "[DribblingDemo] Found RRT path with " << rrt_path.size() << " waypoints" << std::endl;
                
                // Convert RRT path to Eigen::Vector3d trajectory (convert back from mm to meters)
                trajectory_path.clear();
                
                for (const auto& wp : rrt_path) {
                    trajectory_path.push_back(Eigen::Vector3d(wp.x / 1000.0, wp.y / 1000.0, wp.angle));
                }
                
                // Smooth the path by ensuring the last point is at approach distance from ball
                Eigen::Vector3d final_pos = trajectory_path.back();
                Eigen::Vector3d to_ball = ball_pos - final_pos;
                to_ball.z() = 0; // Keep 2D
                double distance = to_ball.norm();
                
                if (distance > approach_distance) {
                    // Move final position closer to ball
                    Eigen::Vector3d approach_position = ball_pos - to_ball.normalized() * approach_distance;
                    approach_position.z() = 0;
                    trajectory_path.back() = approach_position;
                    std::cout << "[DribblingDemo] Adjusted final position to " << approach_distance << "m from ball" << std::endl;
                }
                
                // Use RobotManager's SetPath with TrapezoidalTrajectoryVi3D (handled automatically)
                robot_manager.SetPath(trajectory_path, current_time);
                
                std::cout << "[DribblingDemo] Set trajectory path with " << trajectory_path.size() << " waypoints" << std::endl;
                demo_state = MOVING_TO_BALL;
                break;
            }
            
            case MOVING_TO_BALL: {
                // Check if robot has reached the ball area
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d to_ball = ball_position - robot_pos;
                double distance_to_ball = to_ball.norm();
                
                // Print progress occasionally
                static double last_print = 0;
                if (current_time - last_print > 1.0) {
                    std::cout << "[DribblingDemo] Moving to ball... Robot at (" 
                              << robot_pos.x() << ", " << robot_pos.y() << "), Distance to ball: " 
                              << distance_to_ball << "m" << std::endl;
                    last_print = current_time;
                }
                
                // Check if robot reached ball area
                if (distance_to_ball <= approach_distance) {
                    std::cout << "[DribblingDemo] Robot reached ball area! Distance: " << distance_to_ball << "m" << std::endl;
                    demo_state = ORIENTING_TO_BALL;
                }
                break;
            }
            
            case ORIENTING_TO_BALL: {
                std::cout << "[DribblingDemo] Orienting robot to face the ball..." << std::endl;
                
                // Get current robot and ball positions
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d ball_pos = ball_position;
                
                // Calculate required angle to face the ball
                double dx = ball_pos.x() - robot_pos.x();
                double dy = ball_pos.y() - robot_pos.y();
                double required_angle = atan2(dy, dx);
                
                std::cout << "[DribblingDemo] Robot pos: (" << robot_pos.x() << ", " << robot_pos.y() << ", " << robot_pos.z() << ")" << std::endl;
                std::cout << "[DribblingDemo] Ball pos: (" << ball_pos.x() << ", " << ball_pos.y() << ")" << std::endl;
                std::cout << "[DribblingDemo] Required angle to face ball: " << required_angle << " rad (" << (required_angle * 180.0 / M_PI) << " degrees)" << std::endl;
                
                // FORCE robot to face the ball directly by updating the soccer objects
                for (auto& obj : soccer_objects) {
                    if (obj.name == "robot0") {
                        obj.position.z() = required_angle;  // Set orientation directly
                        std::cout << "[DribblingDemo] Robot orientation set to: " << required_angle << " rad" << std::endl;
                        break;
                    }
                }
                
                // Also update robot manager's internal state
                Eigen::Vector3d new_pose(robot_pos.x(), robot_pos.y(), required_angle);
                robot_manager.InitializePose(new_pose);
                
                std::cout << "[DribblingDemo] Robot is now facing the ball!" << std::endl;
                demo_state = STARTING_DRIBBLE;
                break;
            }
            
            case STARTING_DRIBBLE: {
                std::cout << "[DribblingDemo] Starting dribble..." << std::endl;
                
                // Start dribbling
                robot_manager.DribbleBall();
                is_dribbling = true;
                
                // Small delay to establish dribbling
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                demo_state = PLANNING_DRIBBLE_PATH;
                break;
            }
            
            case PLANNING_DRIBBLE_PATH: {
                std::cout << "[DribblingDemo] Planning RRT path from current robot position to target while dribbling..." << std::endl;
                
                // Get current robot position (since robot is controlling the ball)
                Eigen::Vector3d current_robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d target_pos = target_position;
                
                // Create waypoints for RRT (convert from meters to mm for RRT)
                state::Waypoint start_wp(current_robot_pos.x() * 1000, current_robot_pos.y() * 1000, 0);
                state::Waypoint goal_wp(target_pos.x() * 1000, target_pos.y() * 1000, 0);
                
                // Configure RRT parameters for dribbling (need smoother path)
                algos::RRTParams rrt_params = algos::DefaultRRTParams();
                rrt_params.max_iterations = 1000;
                rrt_params.step_size = 150.0; // Smaller steps for better control while dribbling
                rrt_params.goal_tolerance = 200.0; // 20cm tolerance to target
                rrt_params.use_rrt_star = true; // Use RRT* for optimal dribbling path
                
                // Plan path using RRT
                rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
                
                if (rrt_path.empty()) {
                    std::cout << "[DribblingDemo] Failed to find dribbling path to target!" << std::endl;
                    demo_state = COMPLETED;
                    break;
                }
                
                std::cout << "[DribblingDemo] Found RRT dribbling path with " << rrt_path.size() << " waypoints" << std::endl;
                
                // Convert RRT path to Eigen::Vector3d trajectory (convert back from mm to meters)
                trajectory_path.clear();
                
                for (const auto& wp : rrt_path) {
                    trajectory_path.push_back(Eigen::Vector3d(wp.x / 1000.0, wp.y / 1000.0, wp.angle));
                }
                
                // Use RobotManager's SetPath for dribbling trajectory
                robot_manager.SetPath(trajectory_path, current_time);
                
                std::cout << "[DribblingDemo] Set dribbling trajectory with " << trajectory_path.size() << " waypoints" << std::endl;
                demo_state = DRIBBLING_TO_TARGET;
                break;
            }
            
            case DRIBBLING_TO_TARGET: {
                // Check if robot has reached the target area while dribbling
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d to_target = target_position - robot_pos;
                double distance_to_target = to_target.norm();
                
                // Print progress occasionally
                static double last_print = 0;
                if (current_time - last_print > 1.0) {
                    std::cout << "[DribblingDemo] Dribbling to target... Robot at (" 
                              << robot_pos.x() << ", " << robot_pos.y() << "), Distance to target: " 
                              << distance_to_target << "m" << std::endl;
                    last_print = current_time;
                }
                
                // Check if robot reached target area
                if (distance_to_target <= 0.5) { // 50cm tolerance to target
                    std::cout << "[DribblingDemo] Robot reached target area while dribbling! Distance: " << distance_to_target << "m" << std::endl;
                    
                    // Stop dribbling
                    robot_manager.SetRobotAction(rob::RobotAction::MOVE);
                    is_dribbling = false;
                    
                    demo_state = COMPLETED;
                }
                break;
            }
            
            case COMPLETED: {
                static bool completion_printed = false;
                if (!completion_printed) {
                    std::cout << "\n[DribblingDemo] Demo completed successfully!" << std::endl;
                    std::cout << "[DribblingDemo] Robot successfully dribbled ball from center to target!" << std::endl;
                    std::cout << "[DribblingDemo] Press ESC to exit" << std::endl;
                    completion_printed = true;
                }
                break;
            }
        }
        
        // Execute robot actions (including dribbling)
        if (robot_manager.GetRobotAction() == rob::RobotAction::DRIBBLE_BALL) {
            robot_manager.ExecuteDribbleAction(soccer_objects);
        }
        
        // Update robot logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Update physics
        kin::UpdateKinematics(soccer_objects, dt);
        kin::CheckAndResolveCollisions(soccer_objects);
        
        // Update robot state from robot manager (crucial for visualization and movement)
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot0") {
                obj.position = robot_manager.GetPoseInWorldFrame();
                break;
            }
        }
        
        // Check if simulation should continue and render
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            std::cout << "[DribblingDemo] Simulation window closed" << std::endl;
            break;
        }
        
        // Print ball and dribbling status periodically
        static double last_ball_print = 0;
        if (current_time - last_ball_print > 2.0) {
            for (const auto& obj : soccer_objects) {
                if (obj.name == "ball") {
                    double speed = std::sqrt(obj.velocity.x()*obj.velocity.x() + obj.velocity.y()*obj.velocity.y());
                    std::cout << "[DribblingDemo] Ball: pos(" << obj.position.x() << ", " << obj.position.y() 
                              << "), speed=" << speed << " m/s, attached=" << (obj.is_attached ? "YES" : "NO") << std::endl;
                    break;
                }
            }
            last_ball_print = current_time;
        }
    }

    std::cout << "[DribblingDemo] Demo finished!" << std::endl;
    return 0;
}
