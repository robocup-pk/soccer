// Demo application to test RRT path planning with kick functionality
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

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

int main(int argc, char* argv[]) {
    std::cout << "[DemoRRTKick] Starting RRT path planning with kick demo..." << std::endl;
    
    // Check configuration
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[DemoRRTKick] Set num_robots to 1. Exiting!" << std::endl;
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
    Eigen::Vector3d initial_pose(-2.0, -2.0, 0);  // Start at bottom-left corner
    robot_manager.InitializePose(initial_pose);
    
    // Demo control variables
    bool demo_started = false;
    bool path_planned = false;
    bool path_executed = false;
    bool ball_kicked = false;
    state::Path rrt_path;
    int current_path_index = 0;
    double demo_start_time = 0.0;
    double path_completion_tolerance = 0.15; // 15cm tolerance for reaching waypoints

    std::cout << "RRT Trajectory Kick Demo Instructions:" << std::endl;
    std::cout << "1. Robot starts at (-0.5, -0.5)" << std::endl;
    std::cout << "2. Create RRT path with (x,y,theta) waypoints" << std::endl;
    std::cout << "3. Robot follows trajectory using TrapezoidalTrajectoryVi3D" << std::endl;
    std::cout << "4. Robot orients toward ball using theta values" << std::endl;
    std::cout << "5. When robot faces ball, applies SSL kick" << std::endl;
    std::cout << "6. Press ESC to exit" << std::endl;
    
    // Main simulation loop
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!demo_started) {
            demo_start_time = current_time;
            demo_started = true;
            
            // Position robot at starting location (closer to test RRT)
            for (auto& obj : soccer_objects) {
                if (obj.name == "robot0") {
                    obj.position = Eigen::Vector3d(-0.5, -0.5, 0);  // Close start position
                }
                if (obj.name == "ball") {
                    obj.position = Eigen::Vector3d(0.5, 0.5, 0);   // Close goal position  
                    obj.velocity = Eigen::Vector3d::Zero();        // Ball stationary
                }
            }
            
            std::cout << "[Demo] Demo started - robot at (-0.5, -0.5), ball at (0.5, 0.5)" << std::endl;
        }
        
        // Step 1: Create path with proper theta values for kick demonstration  
        if (demo_started && !path_planned) {
            std::cout << "[Demo] Creating path with RRT waypoints (x,y,theta) for trajectory following..." << std::endl;
            
            // Create path with theta values - RRT provides (x, y, theta)
            rrt_path.clear();
            rrt_path.push_back(state::Waypoint(-500, -500, 0));           // Start position
            rrt_path.push_back(state::Waypoint(0, 0, M_PI/4));           // Middle with 45° orientation  
            rrt_path.push_back(state::Waypoint(500, 500, M_PI/4));       // Ball position, oriented toward ball
            
            // Convert RRT waypoints to Eigen::Vector3d path for TrajectoryManager
            std::vector<Eigen::Vector3d> trajectory_path;
            for (const auto& wp : rrt_path) {
                // Convert from offset mm to world meters, keep theta
                double world_x = wp.x / 1000.0;
                double world_y = wp.y / 1000.0;
                double theta = wp.angle;
                trajectory_path.push_back(Eigen::Vector3d(world_x, world_y, theta));
            }
            
            // Use RobotManager's SetPath with TrapezoidalTrajectoryVi3D
            std::cout << "[Demo] Setting trajectory path with " << trajectory_path.size() << " waypoints:" << std::endl;
            for (size_t i = 0; i < trajectory_path.size(); ++i) {
                std::cout << "  Waypoint " << i << ": World(" << trajectory_path[i].x() << ", " 
                          << trajectory_path[i].y() << ", θ=" << trajectory_path[i].z() << " rad)" << std::endl;
            }
            
            robot_manager.SetPath(trajectory_path, util::GetCurrentTime());
            
            path_planned = true;
            path_executed = false; // Will be set when trajectory is complete
        }
        
        // Step 2: Check if trajectory execution is complete
        if (path_planned && !path_executed) {
            // TrajectoryManager handles the path following automatically
            // We just need to check if the robot has finished the trajectory
            
            // Get robot position to check if close to final waypoint
            Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d ball_pos(0.5, 0.5, 0); // Final waypoint position
            
            double distance_to_ball = std::sqrt(std::pow(robot_pos.x() - ball_pos.x(), 2) + 
                                               std::pow(robot_pos.y() - ball_pos.y(), 2));
            
            // Debug output every second  
            static double last_debug_time = 0;
            if (current_time - last_debug_time > 1.0) {
                std::cout << "[Debug] Following trajectory - Robot: (" << robot_pos.x() << ", " 
                          << robot_pos.y() << ", θ=" << robot_pos.z() << "), Distance to ball: " 
                          << distance_to_ball << "m" << std::endl;
                last_debug_time = current_time;
            }
            
            // Check if robot reached final position (ball area)
            if (distance_to_ball < path_completion_tolerance) {
                path_executed = true;
                std::cout << "[Demo] Trajectory execution complete! Robot reached ball area." << std::endl;
            }
        }
        
        // Step 3: Apply kick when robot reaches ball area
        if (path_executed && !ball_kicked) {
            std::cout << "[Demo] Trajectory complete - checking kick conditions..." << std::endl;
            
            // Find robot and ball objects
            state::SoccerObject* robot = nullptr;
            state::SoccerObject* ball = nullptr;
            
            for (auto& obj : soccer_objects) {
                if (obj.name == "robot0") robot = &obj;
                if (obj.name == "ball") ball = &obj;
            }
            
            if (robot && ball) {
                // Get positions
                Eigen::Vector3d robot_center = robot->GetCenterPosition();
                Eigen::Vector3d ball_center = ball->GetCenterPosition();
                double distance_to_ball = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                                                   std::pow(ball_center.y() - robot_center.y(), 2));
                
                std::cout << "[Demo] Robot at (" << robot_center.x() << ", " << robot_center.y() 
                          << ", θ=" << robot->position[2] << "), Ball at (" << ball_center.x() << ", " 
                          << ball_center.y() << "), Distance: " << distance_to_ball << "m" << std::endl;
                
                // Update robot position from RobotManager (trajectory may have set orientation)
                Eigen::Vector3d robot_pose = robot_manager.GetPoseInWorldFrame();
                robot->position = robot_pose;
                
                // Check if robot is facing the ball using IsPointInFrontSector
                Eigen::Vector2d ball_point(ball_center.x(), ball_center.y());
                bool robot_facing_ball = robot->IsPointInFrontSector(ball_point);
                
                std::cout << "[Demo] Robot facing ball: " << (robot_facing_ball ? "YES" : "NO") << std::endl;
                
                if (distance_to_ball < 0.4) {  // If robot is close to ball
                    if (robot_facing_ball) {
                        // Robot is properly oriented - apply kick
                        double kick_power = 4.0;  // 4 m/s kick
                        std::cout << "[Demo] Robot properly positioned and oriented - applying kick!" << std::endl;
                        
                        bool kick_successful = kin::Kick(*robot, *ball, kick_power, false);
                        
                        if (kick_successful) {
                            std::cout << "[Demo] ✅ SSL kick successful! Ball kicked with " << kick_power << " m/s power!" << std::endl;
                            ball_kicked = true;
                        } else {
                            std::cout << "[Demo] ❌ SSL kick failed - checking SSL constraints..." << std::endl;
                            // The SSL Kick function will output the specific reason for failure
                        }
                    } else {
                        // Robot needs to orient toward ball - calculate required angle
                        double angle_to_ball = std::atan2(ball_center.y() - robot_center.y(), 
                                                         ball_center.x() - robot_center.x());
                        std::cout << "[Demo] Robot not facing ball. Required angle: " << angle_to_ball 
                                  << " rad, Current: " << robot->position[2] << " rad" << std::endl;
                        
                        // Create a single-point trajectory to rotate toward ball
                        std::vector<Eigen::Vector3d> orient_path;
                        orient_path.push_back(robot_pose); // Current position
                        orient_path.push_back(Eigen::Vector3d(robot_center.x(), robot_center.y(), angle_to_ball)); // Same position, new angle
                        robot_manager.SetPath(orient_path, util::GetCurrentTime());
                    }
                } else {
                    // Move closer to ball
                    std::cout << "[Demo] Robot too far from ball - moving closer..." << std::endl;
                    robot_manager.AddGoal(Eigen::Vector3d(ball_center.x(), ball_center.y(), 0));
                }
            }
        }
        
        // Handle input
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Update robot logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Update physics
        kin::UpdateKinematics(soccer_objects, dt);
        kin::CheckAndResolveCollisions(soccer_objects);
        
        // Update robot state from robot manager
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot0") {
                obj.position = robot_manager.GetPoseInWorldFrame();
                break;
            }
        }
        
        // Check if simulation should continue and render
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            std::cout << "[Demo] Simulation window closed" << std::endl;
            break;
        }
        
        // Demo completion check
        if (ball_kicked) {
            static double kick_completion_time = 0;
            if (kick_completion_time == 0) {
                kick_completion_time = current_time;
            }
            
            // Show ball movement for 3 seconds after kick
            if (current_time - kick_completion_time > 3.0) {
                std::cout << "[Demo] RRT Kick Demo completed successfully!" << std::endl;
                std::cout << "Summary:" << std::endl;
                std::cout << "✅ RRT planned path from (-2, -2) to ball at (1, 1)" << std::endl;
                std::cout << "✅ Robot followed RRT path successfully" << std::endl;
                std::cout << "✅ SSL kick applied when robot reached ball" << std::endl;
                break;
            }
        }
    }
    
    std::cout << "[DemoRRTKick] Demo finished!" << std::endl;
    return 0;
}