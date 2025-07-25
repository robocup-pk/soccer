#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>

#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "RRTX.h"
#include "Waypoint.h"
#include "RobotManager.h"
#include "Kinematics.h"
#include "Kick.h"
#include "Utils.h"

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Simple RRTX + TrajectoryManager + Kick Demo" << std::endl;
    
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[KickDemo] Set num_robots to 1. Exiting!" << std::endl;
        return 0;
    }

    // Initialize objects
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    rob::RobotManager robot_manager;
    
    // Set positions
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);     // Robot starts at origin
    Eigen::Vector3d ball_position(1.5, 0.5, 0.0);
    Eigen::Vector3d target_position(2.0, 1.5, 0.0);
    
    // Initialize robot pose
    Eigen::Vector3d robot_pose_ref = robot_start_pose;
    robot_manager.InitializePose(robot_pose_ref);
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot0") {
            obj.position = robot_start_pose;
            obj.velocity = Eigen::Vector3d::Zero();
        } else if (obj.name == "ball") {
            obj.position = ball_position;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    
    // Demo states
    enum State { PLANNING, MOVING, KICKING, COMPLETED };
    State state = PLANNING;
    double movement_start_time = 0.0;
    double demo_start_time = util::GetCurrentTime();
    
    std::cout << "[KickDemo] Press ESC to exit. Robot will move to ball and kick it." << std::endl;

    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        switch (state) {
            case PLANNING: {
                std::cout << "[KickDemo] Planning RRTX path to ball..." << std::endl;
                
                // Wait a bit for pose to initialize
                if (current_time - demo_start_time < 1.0) {
                    break;
                }
                
                // Get robot position and calculate pre-kick position
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                Eigen::Vector3d ball_to_target = (target_position - ball_position).normalized();
                Eigen::Vector3d pre_kick_pos = ball_position - ball_to_target * 0.25; // 25cm from ball
                
                // Create RRTX waypoints (convert to mm)
                state::Waypoint start_wp(robot_pos.x(), robot_pos.y(), 0);
                state::Waypoint goal_wp(ball_position.x(), ball_position.y(), 0);
                
                
                // Plan path with RRTX - try very small parameters
                state::Path rrt_path = algo::FindSinglePath_RRTX(start_wp, goal_wp);
                std::cout<<rrt_path.size() << " waypoints found in RRTX path." << std::endl;
                
                // Create fallback if RRTX fails
                if (rrt_path.empty()) {
                    std::cout << "[KickDemo] RRTX failed, using direct path" << std::endl;
                    Eigen::Vector3d mid_pos = (robot_pos + pre_kick_pos) * 0.5;
                    rrt_path.push_back(state::Waypoint(robot_pos.x() * 1000, robot_pos.y() * 1000, 0));
                    rrt_path.push_back(state::Waypoint(mid_pos.x() * 1000, mid_pos.y() * 1000, 0));
                    rrt_path.push_back(state::Waypoint(pre_kick_pos.x() * 1000, pre_kick_pos.y() * 1000, 0));
                    return 0; // Exit if no path found
                }
                
                // Convert to trajectory (convert back to meters)
                std::vector<Eigen::Vector3d> trajectory_path;
                for (const auto& wp : rrt_path) {
                    trajectory_path.push_back(Eigen::Vector3d(wp.x, wp.y, wp.angle));
                }
                
                // Set trajectory
                robot_manager.SetPath(trajectory_path);
                
                state = MOVING;
                movement_start_time = current_time;
                std::cout << "[KickDemo] Path set, moving to ball..." << std::endl;
                break;
            }
            
            case MOVING: {
                // Check if robot reached ball or trajectory completed
                Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
                double distance_to_ball = (ball_position - robot_pos).norm();
                std::string robot_state = robot_manager.GetRobotState();
                bool trajectory_done = (robot_state != "AUTONOMOUS_DRIVING");
                
                // Print progress occasionally
                static int counter = 0;
                if (counter++ % 60 == 0) {
                    std::cout << "[KickDemo] Robot at (" << robot_pos.x() << ", " << robot_pos.y() 
                              << "), distance to ball: " << distance_to_ball << "m" << std::endl;
                }
                
                // Transition to kicking
                if (distance_to_ball < 0.4 || trajectory_done || (current_time - movement_start_time > 10.0)) {
                    std::cout << "[KickDemo] Reached ball area, preparing to kick..." << std::endl;
                    state = KICKING;
                }
                break;
            }
            
            case KICKING: {
                std::cout << "[KickDemo] Kicking ball towards target!" << std::endl;
                
                // Find robot and ball objects
                state::SoccerObject* robot = nullptr;
                state::Ball* ball = nullptr;
                for (auto& obj : soccer_objects) {
                    if (obj.name == "robot0") robot = &obj;
                    else if (obj.name == "ball") ball = dynamic_cast<state::Ball*>(&obj);
                }
                
                if (robot && ball) {
                    // Calculate kick direction and power
                    Eigen::Vector2d kick_dir = (target_position.head<2>() - ball->position.head<2>()).normalized();
                    double kick_power = 3.0; // 3 m/s kick
                    
                    // Execute kick
                    bool success = kin::Kick(*robot, *ball, kick_power, true);
                    if (success) {
                        std::cout << "[KickDemo] Ball kicked successfully!" << std::endl;
                    } else {
                        std::cout << "[KickDemo] Kick failed" << std::endl;
                    }
                }
                
                state = COMPLETED;
                break;
            }
            
            case COMPLETED: {
                static bool printed = false;
                if (!printed) {
                    std::cout << "[KickDemo] Demo completed! Press ESC to exit." << std::endl;
                    printed = true;
                }
                break;
            }
        }
        
        // Update robot and physics
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Sync robot position to soccer objects
        for (auto& obj : soccer_objects) {
            if (obj.name == "robot0") {
                obj.position = robot_manager.GetPoseInWorldFrame();
                obj.velocity = robot_manager.GetVelocityInWorldFrame();
                break;
            }
        }
        
        kin::UpdateKinematics(soccer_objects, dt);
        kin::CheckAndResolveCollisions(soccer_objects);
        
        if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
            break;
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}