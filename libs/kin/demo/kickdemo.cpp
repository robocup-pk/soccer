// Demo application to test RRT path planning with trapezoidal trajectory and kick functionality
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
#include "TrapezoidalTrajectoryVi3D.h"

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Starting RRT + Trapezoidal Trajectory Kick Demo..." << std::endl;
    
    // Check configuration
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[KickDemo] Set num_robots to 1. Exiting!" << std::endl;
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
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0);  // Robot starts at bottom-left
    Eigen::Vector3d ball_position(-1.5, 0.0, 0);       // Ball at top-right
    
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
        PLANNING_PATH,
        EXECUTING_TRAJECTORY,
        KICKING_BALL,
        COMPLETED
    };
    
    DemoState demo_state = INIT;
    bool demo_started = false;
    state::Path rrt_path;
    std::vector<Eigen::Vector3d> trajectory_path;
    double demo_start_time = 0.0;
    double kicking_distance = 0.2; // 20cm - distance to start orienting towards ball

    std::cout << "\nDemo Instructions:" << std::endl;
    std::cout << "1. Robot starts at (-2.0, -1.5)" << std::endl;
    std::cout << "2. Ball is positioned at (1.5, 1.0)" << std::endl;
    std::cout << "3. RRT will plan a path from robot to ball" << std::endl;
    std::cout << "4. Robot follows trapezoidal trajectory to reach ball" << std::endl;
    std::cout << "5. Robot orients towards ball and kicks it" << std::endl;
    std::cout << "6. Press ESC to exit\n" << std::endl;

    // Main simulation loop
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!demo_started) {
            demo_start_time = current_time;
            demo_started = true;
            demo_state = PLANNING_PATH;
        }
        
        // Handle input
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Demo state machine
        switch (demo_state) {
            case INIT:
                // Already handled above
                break;
                
            case PLANNING_PATH: {
                std::cout << "[KickDemo] Planning RRT path from robot to ball..." << std::endl;
                
                Eigen::Vector3d robot_pos = robot_start_pose;
                Eigen::Vector3d ball_pos = ball_position;
                
                // Define a pre-kick position behind the ball
                Eigen::Vector3d to_ball = (ball_pos - robot_pos).normalized();
                Eigen::Vector3d pre_kick_pos = ball_pos - to_ball * kicking_distance;
                double desired_angle = atan2(to_ball.y(), to_ball.x());

                state::Waypoint start_wp(robot_pos.x() * 1000, robot_pos.y() * 1000, robot_pos.z());
                // Use start angle for planning to avoid assertion, we'll set the final angle later
                state::Waypoint goal_wp(pre_kick_pos.x() * 1000, pre_kick_pos.y() * 1000, start_wp.angle);
                
                algos::RRTParams rrt_params = algos::DefaultRRTParams();
                rrt_params.use_rrt_star = true;
                
                rrt_path = algos::FindSinglePath(start_wp, goal_wp, rrt_params);
                
                if (rrt_path.empty()) {
                    std::cout << "[KickDemo] Failed to find path to ball!" << std::endl;
                    demo_state = COMPLETED;
                    break;
                }
                
                // Set the final, desired angle on the last waypoint of the generated path
                rrt_path.back().angle = desired_angle;

                trajectory_path.clear();
                for (const auto& wp : rrt_path) {
                    trajectory_path.push_back(Eigen::Vector3d(wp.x / 1000.0, wp.y / 1000.0, wp.angle));
                }
                
                robot_manager.SetPath(trajectory_path, current_time);
                
                demo_state = EXECUTING_TRAJECTORY;
                break;
            }
            
            case EXECUTING_TRAJECTORY: {
                if (current_time > robot_manager.trajectory_manager.active_traj_t_finish_s) {
                    std::cout << "[KickDemo] Robot reached pre-kick position!" << std::endl;
                    demo_state = KICKING_BALL;
                }
                break;
            }
            
            case KICKING_BALL: {
                std::cout << "[KickDemo] Executing kick!" << std::endl;
                
                state::SoccerObject* ball = nullptr;
                for (auto& obj : soccer_objects) {
                    if (obj.name == "ball") {
                        ball = &obj;
                        break;
                    }
                }
                
                if (ball) {
                    Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                    Eigen::Vector3d to_ball_vec = ball->position - robot_pos;
                    Eigen::Vector2d kick_direction(to_ball_vec.x(), to_ball_vec.y());
                    kick_direction.normalize();
                    
                    double kick_power = 4.0; // 4 m/s
                    kin::ApplyKickToBall(*ball, kick_direction, kick_power);
                } else {
                    std::cout << "[KickDemo] Ball not found!" << std::endl;
                }
                
                demo_state = COMPLETED;
                break;
            }
            
            case COMPLETED: {
                static bool completion_printed = false;
                if (!completion_printed) {
                    std::cout << "\n[KickDemo] Demo completed successfully!" << std::endl;
                    std::cout << "[KickDemo] Watch the ball move with realistic physics!" << std::endl;
                    std::cout << "[KickDemo] Press ESC to exit" << std::endl;
                    completion_printed = true;
                }
                break;
            }
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
            std::cout << "[KickDemo] Simulation window closed" << std::endl;
            break;
        }
        
        // Print ball status periodically
        static double last_ball_print = 0;
        if (current_time - last_ball_print > 2.0) {
            for (const auto& obj : soccer_objects) {
                if (obj.name == "ball") {
                    double speed = std::sqrt(obj.velocity.x()*obj.velocity.x() + obj.velocity.y()*obj.velocity.y());
                    std::cout << "[KickDemo] Ball: pos(" << obj.position.x() << ", " << obj.position.y() 
                              << "), speed=" << speed << " m/s" << std::endl;
                    break;
                }
            }
            last_ball_print = current_time;
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}