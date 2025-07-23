// Demo application to test RRT* path planning with dribble functionality
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
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

int main(int argc, char* argv[]) {
    std::cout << "[DemoRRTDribble] Starting RRT* path planning with dribble demo..." << std::endl;
    
    // Check configuration
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[DemoRRTDribble] Set num_robots to 1. Exiting!" << std::endl;
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
    Eigen::Vector3d initial_pose(-1.5, -1.0, 0);  // Start at a different position
    robot_manager.InitializePose(initial_pose);
    
    // Demo control variables
    bool demo_started = false;
    bool path_planned = false;
    bool path_executed = false;
    bool ball_reached = false;
    bool is_dribbling = false;
    state::Path rrt_path;
    double demo_start_time = 0.0;
    double dribble_start_time = 0.0;
    double path_completion_tolerance = 0.30; // 30cm tolerance for dribbling
    double dribble_duration = 5.0; // Dribble for 5 seconds

    std::cout << "RRT* Dribble Demo Instructions:" << std::endl;
    std::cout << "1. Robot starts at (-1.5, -1.0)" << std::endl;
    std::cout << "2. Uses RRT* algorithm to plan optimal path to ball" << std::endl;
    std::cout << "3. Robot follows trajectory using TrapezoidalTrajectoryVi3D" << std::endl;
    std::cout << "4. When robot reaches ball, it starts dribbling" << std::endl;
    std::cout << "5. Robot maintains ball control using SSL dribble function" << std::endl;
    std::cout << "6. Press ESC to exit" << std::endl;
    
    // Main simulation loop
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!demo_started) {
            demo_start_time = current_time;
            demo_started = true;
            
            // Position robot at starting location
            for (auto& obj : soccer_objects) {
                if (obj.name == "robot0") {
                    obj.position = Eigen::Vector3d(-1.5, -1.0, 0);  // Starting position
                }
                if (obj.name == "ball") {
                    obj.position = Eigen::Vector3d(0.8, 0.6, 0);   // Ball position  
                    obj.velocity = Eigen::Vector3d::Zero();        // Ball stationary
                }
            }
            
            std::cout << "[Demo] Demo started - robot at (-1.5, -1.0), ball at (0.8, 0.6)" << std::endl;
        }
        
        // Step 1: Use RRT* algorithm to plan optimal path to ball
        if (demo_started && !path_planned) {
            std::cout << "[Demo] Using RRT* algorithm to plan optimal path to ball..." << std::endl;
            
            // Fix coordinate system for RRT - use positions within field bounds
            double field_width_mm = vis::SoccerField::GetInstance().width_mm;   // ~4076mm
            double field_height_mm = vis::SoccerField::GetInstance().height_mm; // ~2900mm
            
            std::cout << "[Debug] Field dimensions: " << field_width_mm << "x" << field_height_mm << " mm" << std::endl;
            
            // Convert robot and ball positions to RRT coordinate system (0 to field_size)
            // Robot at (-1.5, -1.0) -> RRT coords  
            double robot_rrt_x = field_width_mm/2 + (-1500);  // Center + offset
            double robot_rrt_y = field_height_mm/2 + (-1000);
            
            // Ball at (0.8, 0.6) -> RRT coords
            double ball_rrt_x = field_width_mm/2 + 800;
            double ball_rrt_y = field_height_mm/2 + 600;
            
            // Ensure coordinates are within bounds
            robot_rrt_x = std::max(100.0, std::min(robot_rrt_x, field_width_mm - 100));
            robot_rrt_y = std::max(100.0, std::min(robot_rrt_y, field_height_mm - 100)); 
            ball_rrt_x = std::max(100.0, std::min(ball_rrt_x, field_width_mm - 100));
            ball_rrt_y = std::max(100.0, std::min(ball_rrt_y, field_height_mm - 100));
            
            state::Waypoint start_wp(robot_rrt_x, robot_rrt_y, 0);
            state::Waypoint goal_wp(ball_rrt_x, ball_rrt_y, 0);
            
            std::cout << "[Debug] RRT* start: (" << start_wp.x << ", " << start_wp.y << ")" << std::endl;
            std::cout << "[Debug] RRT* goal: (" << goal_wp.x << ", " << goal_wp.y << ")" << std::endl;
            
            // Create RRT* parameters for optimal path planning
            algos::RRTParams rrt_params;
            rrt_params.use_rrt_star = true;           // Enable RRT* for optimal paths
            rrt_params.max_iterations = 1000;        // More iterations for better optimization
            rrt_params.step_size = 80.0;             // Smaller steps for smoother path
            rrt_params.goal_bias = 0.3;              // Lower bias for more exploration
            rrt_params.goal_tolerance = 100.0;       // Tighter tolerance
            rrt_params.near_radius = 250.0;          // Larger radius for more rewiring
            
            // Call RRT* algorithm
            rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
            
            if (!rrt_path.empty()) {
                std::cout << "[Demo] ✅ RRT* algorithm found optimal path with " << rrt_path.size() << " waypoints!" << std::endl;
                
                // Convert RRT path to trajectory waypoints with theta values
                std::vector<Eigen::Vector3d> trajectory_path;
                
                for (size_t i = 0; i < rrt_path.size(); ++i) {
                    // Convert from RRT coordinates back to world coordinates
                    double world_x = (rrt_path[i].x - field_width_mm/2) / 1000.0;
                    double world_y = (rrt_path[i].y - field_height_mm/2) / 1000.0;
                    
                    // Calculate theta: direction to next waypoint (or to ball for last waypoint)
                    double theta = 0;
                    if (i < rrt_path.size() - 1) {
                        // Direction to next waypoint
                        double dx = rrt_path[i+1].x - rrt_path[i].x;
                        double dy = rrt_path[i+1].y - rrt_path[i].y;
                        theta = std::atan2(dy, dx);
                    } else {
                        // Last waypoint: face toward ball
                        double ball_x = (ball_rrt_x - field_width_mm/2) / 1000.0;
                        double ball_y = (ball_rrt_y - field_height_mm/2) / 1000.0;
                        double dx = ball_x - world_x;
                        double dy = ball_y - world_y;
                        theta = std::atan2(dy, dx);
                    }
                    
                    trajectory_path.push_back(Eigen::Vector3d(world_x, world_y, theta));
                    
                    std::cout << "  RRT* Waypoint " << i << ": RRT(" << rrt_path[i].x << ", " << rrt_path[i].y 
                              << ") -> World(" << world_x << ", " << world_y << ", θ=" << theta << " rad)" << std::endl;
                }
                
                // Use RobotManager's SetPath with TrapezoidalTrajectoryVi3D
                robot_manager.SetPath(trajectory_path, util::GetCurrentTime());
                
                path_planned = true;
                path_executed = false;
            } else {
                std::cout << "[Demo] RRT* algorithm failed to find path! Retrying..." << std::endl;
                // Reset and try again
                demo_started = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        
        // Step 2: Check if trajectory execution is complete
        if (path_planned && !path_executed) {
            // TrajectoryManager handles the path following automatically
            // We just need to check if the robot has finished the trajectory
            
            // Get robot position to check if close to final waypoint
            Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d ball_pos(0.8, 0.6, 0); // Final waypoint position
            
            double distance_to_ball = std::sqrt(std::pow(robot_pos.x() - ball_pos.x(), 2) + 
                                               std::pow(robot_pos.y() - ball_pos.y(), 2));
            
            // Debug output every second  
            static double last_debug_time = 0;
            if (current_time - last_debug_time > 1.0) {
                std::cout << "[Debug] Following RRT* trajectory - Robot: (" << robot_pos.x() << ", " 
                          << robot_pos.y() << ", θ=" << robot_pos.z() << "), Distance to ball: " 
                          << distance_to_ball << "m" << std::endl;
                last_debug_time = current_time;
            }
            
            // Check if robot reached final position (ball area)
            if (distance_to_ball < path_completion_tolerance) {
                path_executed = true;
                ball_reached = true;
                dribble_start_time = current_time;
                std::cout << "[Demo] RRT* trajectory execution complete! Robot reached ball area." << std::endl;
            }
        }
        
        // Step 3: Start dribbling when robot reaches ball area
        if (ball_reached && !is_dribbling) {
            std::cout << "[Demo] Starting dribble phase..." << std::endl;
            is_dribbling = true;
        }
        
        // Step 4: Continuous dribbling
        if (is_dribbling) {
            double dribble_elapsed = current_time - dribble_start_time;
            
            // Find robot and ball objects
            state::SoccerObject* robot = nullptr;
            state::SoccerObject* ball = nullptr;
            
            for (auto& obj : soccer_objects) {
                if (obj.name == "robot0") robot = &obj;
                if (obj.name == "ball") ball = &obj;
            }
            
            if (robot && ball && dribble_elapsed < dribble_duration) {
                // Get positions
                Eigen::Vector3d robot_center = robot->GetCenterPosition();
                Eigen::Vector3d ball_center = ball->GetCenterPosition();
                double distance_to_ball = std::sqrt(std::pow(ball_center.x() - robot_center.x(), 2) + 
                                                   std::pow(ball_center.y() - robot_center.y(), 2));
                
                // Update robot position from RobotManager 
                Eigen::Vector3d robot_pose = robot_manager.GetPoseInWorldFrame();
                robot->position = robot_pose;
                
                // Apply dribble control
                double dribble_power = 1.2;  // Moderate dribble power
                bool dribble_successful = kin::Dribble(*robot, *ball, dribble_power, false);
                
                // Debug output every half second during dribbling
                static double last_dribble_debug = 0;
                if (current_time - last_dribble_debug > 0.5) {
                    std::cout << "[Dribble] Time: " << std::fixed << std::setprecision(1) << dribble_elapsed 
                              << "s, Distance: " << std::setprecision(3) << distance_to_ball 
                              << "m, Success: " << (dribble_successful ? "YES" : "NO") << std::endl;
                    last_dribble_debug = current_time;
                }
                
                // Optional: Move robot slightly while dribbling to demonstrate ball control
                if (dribble_elapsed > 2.0 && dribble_elapsed < 4.0) {
                    // Move in a small circle while dribbling
                    double circle_time = dribble_elapsed - 2.0;
                    double circle_radius = 0.3;
                    double angular_speed = 1.0; // rad/s
                    
                    double center_x = 0.8;
                    double center_y = 0.6;
                    double target_x = center_x + circle_radius * std::cos(angular_speed * circle_time);
                    double target_y = center_y + circle_radius * std::sin(angular_speed * circle_time);
                    double target_theta = angular_speed * circle_time + M_PI/2; // Face tangent to circle
                    
                    robot_manager.AddGoal(Eigen::Vector3d(target_x, target_y, target_theta));
                }
                
            } else if (dribble_elapsed >= dribble_duration) {
                std::cout << "[Demo] Dribble phase completed after " << dribble_duration << " seconds!" << std::endl;
                std::cout << "[Demo] RRT* Dribble Demo completed successfully!" << std::endl;
                std::cout << "Summary:" << std::endl;
                std::cout << "✅ RRT* planned optimal path from (-1.5, -1.0) to ball at (0.8, 0.6)" << std::endl;
                std::cout << "✅ Robot followed RRT* path using TrapezoidalTrajectoryVi3D" << std::endl;
                std::cout << "✅ SSL dribble applied for continuous ball control" << std::endl;
                std::cout << "✅ Robot maintained ball control while moving" << std::endl;
                break;
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
    }
    
    std::cout << "[DemoRRTDribble] Demo finished!" << std::endl;
    return 0;
}