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

// Run demo without graphical output when true
static const bool HEADLESS = false;

// Demo configuration
enum class DemoMode {
    ORIGINAL,      // Use original trajectory system
    BangBang,      // Use BangBang-based M_TrajectoryManager
    PurePursuit,   // Use Pure Pursuit for multi-waypoint following
    HermiteSpline, // Use Hermite Spline for RRT* waypoints
    BSpline        // Use B-spline for smoother trajectories
};

using namespace std;
int main(int argc, char* argv[]) {
    // Parse command line arguments for demo mode
    DemoMode demo_mode = DemoMode::ORIGINAL;  // Default
    if (argc > 1) {
        std::string mode_arg = argv[1];
        if (mode_arg == "bangbang" || mode_arg == "m") {
            demo_mode = DemoMode::BangBang;
        } else if (mode_arg == "purepursuit" || mode_arg == "pp") {
            demo_mode = DemoMode::PurePursuit;
        } else if (mode_arg == "hermite" || mode_arg == "hs") {
            demo_mode = DemoMode::HermiteSpline;
        } else if (mode_arg == "bspline" || mode_arg == "bs") {
            demo_mode = DemoMode::BSpline;
        }
    }
    
    std::cout << "[KickDemo] RRTX + Multi-Waypoint Trajectory + Kick Demo" << std::endl;
    std::string mode_name = "ORIGINAL";
    if (demo_mode == DemoMode::BangBang) mode_name = "BANGBANG";
    else if (demo_mode == DemoMode::PurePursuit) mode_name = "PURE_PURSUIT";
    else if (demo_mode == DemoMode::HermiteSpline) mode_name = "HERMITE_SPLINE";
    else if (demo_mode == DemoMode::BSpline) mode_name = "B_SPLINE";
    std::cout << "[KickDemo] Demo Mode: " << mode_name << std::endl;
    
    if (cfg::SystemConfig::num_robots != 1) {
        std::cout << "[KickDemo] Set num_robots to 1. Exiting!" << std::endl;
        return 0;
    }

    // Initialize objects
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    if (!HEADLESS) {
        gl_simulation.InitGameObjects(soccer_objects);
    }
    rob::RobotManager robot_manager;
    
    // Set positions
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);     // Robot starts at origin
    Eigen::Vector3d ball_position(-1.5, 0.5, 0.0);
    
    Eigen::Vector3d direction = (ball_position - robot_start_pose);
    double angle = std::atan2(direction.y(), direction.x());
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
    //Give source and destination for RRTX
    state::Waypoint source(robot_start_pose[0], robot_start_pose[1], 0.0);
    state::Waypoint destination(ball_position[0], ball_position[1], 0.0);  // Use angle to face the ball  
    // Create RRTX instance
    state::Path path;
    // Find path from robot to ball
    path = algo::FindSinglePath_RRTX(source, destination);

    if (path.empty()) {
        std::cout << "[KickDemo] No path found from robot to ball!" << std::endl;
        return 1;
    }
    vector<Eigen::Vector3d> targetPath;
    for(int i = 0; i < path.size(); ++i){
        cout << "Waypoint " << i << ": (" << path[i].x << ", " << path[i].y << ", " << path[i].angle << ")" << std::endl;
    }
    // Convert RRTX waypoints based on demo mode
    if (demo_mode == DemoMode::PurePursuit || demo_mode == DemoMode::HermiteSpline || demo_mode == DemoMode::BSpline) {
        // For Pure Pursuit, Hermite Spline, and B-spline: Use ALL RRTX waypoints for smooth multi-waypoint following
        std::string planner_name = (demo_mode == DemoMode::PurePursuit) ? "Pure Pursuit" : 
                                  (demo_mode == DemoMode::HermiteSpline) ? "Hermite Spline" : "B-spline";
        std::cout << "[KickDemo] Using ALL " << path.size() << " RRTX waypoints for " << planner_name << std::endl;
        for(int i = 0; i < path.size(); ++i){
            // Calculate angle to face next waypoint (or ball for last waypoint) 
            double waypoint_angle = path[i].angle;
            if (i < path.size() - 1) {
                // Face towards next waypoint
                double dx = path[i+1].x - path[i].x;
                double dy = path[i+1].y - path[i].y;
                waypoint_angle = std::atan2(dy, dx);
            } else {
                // Last waypoint: face towards ball
                double dx = ball_position[0] - path[i].x;
                double dy = ball_position[1] - path[i].y;
                waypoint_angle = std::atan2(dy, dx);
            }
            
            targetPath.push_back(Eigen::Vector3d(path[i].x, path[i].y, waypoint_angle));
            std::cout << "  Waypoint " << i << ": (" << path[i].x << ", " << path[i].y 
                      << ", " << waypoint_angle << " rad)" << std::endl;
        }
    } else {
        // For other modes: Only use start and end (traditional approach)
        targetPath.push_back(robot_start_pose);  // Start position
        targetPath.push_back(Eigen::Vector3d(ball_position[0], ball_position[1], angle));  // Target position
    }
    // Set trajectory manager type and path based on demo mode
    if (demo_mode == DemoMode::BangBang) {
        std::cout << "[KickDemo] Using BangBang-based M_TrajectoryManager" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
        robot_manager.SetMPath(targetPath);
    } else if (demo_mode == DemoMode::PurePursuit) {
        std::cout << "[KickDemo] Using Pure Pursuit multi-waypoint following" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::PurePursuit);
        robot_manager.SetPurePursuitPath(targetPath);
    } else if (demo_mode == DemoMode::HermiteSpline) {
        std::cout << "[KickDemo] Using Hermite Spline for RRT* waypoints" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::HermiteSpline);
        robot_manager.SetHermiteSplinePath(targetPath);
    } else if (demo_mode == DemoMode::BSpline) {
        std::cout << "[KickDemo] Using B-spline for smooth RRT* waypoint following" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        robot_manager.SetBSplinePath(targetPath);
    } else {
        std::cout << "[KickDemo] Using original TrajectoryManager" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::ORIGINAL);
        robot_manager.SetPath(targetPath);
    }

    bool kick_executed = false;  // Flag to ensure kick is executed only once

    while (true) {
        // Check if path is finished before executing kick
        if (robot_manager.GetRobotState() == "IDLE" && !kick_executed) {
            // Debug: Check robot and ball positions
            Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d ball_pos = ball_position;
            
            double distance = (robot_pos - ball_pos).head<2>().norm();
            
            std::cout << "Robot position: [" << robot_pos.transpose() << "]" << std::endl;
            std::cout << "Ball position: [" << ball_pos.transpose() << "]" << std::endl;
            std::cout << "Distance to ball: " << distance << "m (threshold: 0.3m)" << std::endl;
            
            if (distance > 0.32) {  // Only create new path if robot is far from ball
                std::cout << "Robot too far from ball! Moving closer..." << std::endl;
                // Create a direct path to get very close to the ball
                vector<Eigen::Vector3d> direct_path;
                direct_path.push_back(robot_pos);
                
                // Calculate optimal approach direction (from robot towards ball)
                Eigen::Vector3d direction = (ball_pos - robot_pos).normalized();
                // Target position: 15cm away from ball in the direction the robot is coming from
                Eigen::Vector3d ball_approach = ball_pos - direction * 0.15;  // 15cm away
                
                std::cout << "Approach direction: [" << direction.transpose() << "]" << std::endl;
                std::cout << "Target approach position: [" << ball_approach.transpose() << "]" << std::endl;
                
                direct_path.push_back(ball_approach);  // Path now in meters
                robot_manager.SetPath(direct_path);
            } else if (distance <= 0.35) {  // Within kick range - increased threshold
                std::cout << "Robot close enough to ball! Distance: " << distance << "m" << std::endl;
                std::cout << "Executing kick action" << std::endl;
                kin::ExecuteKick(soccer_objects);  // Execute kick directly
                kick_executed = true;  // Set flag to true after kicking

                // Optional: exit after kicking
                std::cout << "Robot kicked the ball. Exiting demo." << std::endl;
                break;
            } else {
                std::cout << "Robot in intermediate zone. Distance: " << distance << "m (between 0.32-0.35m)" << std::endl;
                std::cout << "Waiting for robot to settle..." << std::endl;
            }
        }

        
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!HEADLESS)
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);

        // Control logic for RobotManager
        robot_manager.ControlLogic();
        
        // Sense logic for RobotManager
        robot_manager.SenseLogic();

        // Sync robot position to soccer objects
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
        soccer_objects[0].velocity = robot_manager.GetVelocityInWorldFrame();
        
        //kin::UpdateKinematics(soccer_objects, dt);
        //kin::CheckAndResolveCollisions(soccer_objects);
        
        if (!HEADLESS) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
                break;
            }
        } else {
            // In headless mode, print robot pose for debugging and sleep a bit
            Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
            std::cout << "[KickDemo] t=" << current_time
                      << " robot=" << robot_pos.transpose() << std::endl;
            util::WaitMs(20);  // rough 50Hz update
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}