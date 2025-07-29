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
using namespace std;
int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Simple RRTX + BangBangTrajectory + Kick Demo" << std::endl;
    
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
    Eigen::Vector3d ball_position(1.5, -0.5, 0.0);
    
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
    state::Waypoint destination(ball_position[0], ball_position[1], 0.0);
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
    // Convert path to Eigen::Vector3d format for RobotManager
    for(int i = 0; i < path.size(); ++i) {
        targetPath.push_back(Eigen::Vector3d(path[i].x * 1000, path[i].y * 1000, path[i].angle ));
    }
    // Add ball to robot manager's goal queue
    robot_manager.SetPath(targetPath);

    bool kick_executed = false;  // Flag to ensure kick is executed only once

    while (true) {
        // Check if path is finished before executing kick
        if (robot_manager.GetRobotState() == "IDLE" && !kick_executed) {
            // Debug: Check robot and ball positions
            Eigen::Vector3d robot_pos_raw = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d ball_pos = ball_position;
            
            // Check if robot position needs conversion from mm to m
            Eigen::Vector3d robot_pos = robot_pos_raw;
            if (std::abs(robot_pos_raw[0]) > 10.0 || std::abs(robot_pos_raw[1]) > 10.0) {
                robot_pos = robot_pos_raw / 1000.0;  // Convert mm to m
                std::cout << "Converting robot position from mm to m: [" << robot_pos_raw.transpose() << "] -> [" << robot_pos.transpose() << "]" << std::endl;
            }
            
            double distance = (robot_pos - ball_pos).head<2>().norm();
            
            std::cout << "Robot position: [" << robot_pos.transpose() << "]" << std::endl;
            std::cout << "Ball position: [" << ball_pos.transpose() << "]" << std::endl;
            std::cout << "Distance to ball: " << distance << "m (threshold: 0.3m)" << std::endl;
            
            if (distance > 0.35) {  // Only create new path if robot is far from ball
                std::cout << "Robot too far from ball! Moving closer..." << std::endl;
                // Create a direct path to get very close to the ball
                vector<Eigen::Vector3d> direct_path;
                direct_path.push_back(robot_pos);
                
                // Calculate optimal approach direction (from robot towards ball)
                Eigen::Vector3d direction = (ball_pos - robot_pos).normalized();
                // Target position: 20cm away from ball in the direction the robot is coming from
                Eigen::Vector3d ball_approach = ball_pos - direction * 0.20;  // 20cm away
                
                std::cout << "Approach direction: [" << direction.transpose() << "]" << std::endl;
                std::cout << "Target approach position: [" << ball_approach.transpose() << "]" << std::endl;
                
                direct_path.push_back(ball_approach * 1000);  // Convert to mm
                robot_manager.SetPath(direct_path);
            } else if (distance <= 0.30) {  // Within kick range
                std::cout << "Robot close enough to ball! Distance: " << distance << "m" << std::endl;
                std::cout << "Executing kick action" << std::endl;
                kin::ExecuteKick(soccer_objects);  // Execute kick directly
                kick_executed = true;  // Set flag to true after kicking

                // Optional: exit after kicking
                std::cout << "Robot kicked the ball. Exiting demo." << std::endl;
                break;
            } else {
                std::cout << "Robot in intermediate zone. Distance: " << distance << "m (between 0.30-0.35m)" << std::endl;
                std::cout << "Waiting for robot to settle..." << std::endl;
            }
        }
        
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects); 
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