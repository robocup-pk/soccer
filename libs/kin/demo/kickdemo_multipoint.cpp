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
#include "M_TrajectorySegment.h"

// Run demo without graphical output when true
static const bool HEADLESS = false;

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Multi-waypoint RRTX + Dynamic Trajectory Demo" << std::endl;
    
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
    Eigen::Vector3d ball_position(-1.5, -0.5, 0.0);
    
    // Initialize robot pose
    robot_manager.InitializePose(robot_start_pose);
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot0") {
            obj.position = robot_start_pose;
            obj.velocity = Eigen::Vector3d::Zero();
        } else if (obj.name == "ball") {
            obj.position = ball_position;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    
    // Use RRTX to find path from robot to ball
    state::Waypoint source(robot_start_pose[0], robot_start_pose[1], 0.0);
    state::Waypoint destination(ball_position[0], ball_position[1], 0.0);
    state::Path path = algo::FindSinglePath_RRTX(source, destination);
    
    if (path.empty()) {
        std::cout << "[KickDemo] No path found from robot to ball!" << std::endl;
        return 1;
    }
    
    std::cout << "[KickDemo] RRTX found path with " << path.size() << " waypoints:" << std::endl;
    for(int i = 0; i < path.size(); ++i){
        cout << "  Waypoint " << i << ": (" << path[i].x << ", " << path[i].y << ", " << path[i].angle << ")" << std::endl;
    }
    
    // Convert RRTX path to Eigen::Vector3d waypoints
    std::vector<Eigen::Vector3d> waypoints;
    for (const auto& wp : path) {
        // Calculate angle to face the next waypoint (or ball for last waypoint)
        double angle = wp.angle;
        if (&wp == &path.back()) {
            // For the last waypoint, face the ball
            Eigen::Vector3d to_ball = ball_position - Eigen::Vector3d(wp.x, wp.y, 0);
            angle = std::atan2(to_ball.y(), to_ball.x());
        }
        waypoints.push_back(Eigen::Vector3d(wp.x, wp.y, angle));
    }
    
    // Create movement constraints
    ctrl::M_MoveConstraints constraints;
    constraints.vel_max = 2.0;
    constraints.acc_max = 3.0;
    constraints.vel_max_w = 10.0;
    constraints.acc_max_w = 20.0;
    
    // Create initial robot state
    ctrl::M_RobotState robot_state;
    robot_state.position = robot_start_pose;
    robot_state.velocity = Eigen::Vector3d::Zero();
    robot_state.constraints = constraints;
    
    // Generate multi-waypoint trajectory
    auto trajectory = ctrl::M_MultiWaypointPlanner::generateMultiWaypointTrajectory(
        robot_state, waypoints, constraints
    );
    
    std::cout << "[KickDemo] Generated trajectory with total time: " << trajectory->getTotalTime() << "s" << std::endl;
    trajectory->print();
    
    // Tracking variables
    double trajectory_start_time = util::GetCurrentTime();
    double last_update_time = trajectory_start_time;
    bool kick_executed = false;
    int current_waypoint_idx = 0;
    std::vector<Eigen::Vector3d> remaining_waypoints = waypoints;
    
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        double trajectory_time = current_time - trajectory_start_time;
        
        if (!HEADLESS)
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Get current robot state from trajectory
        Eigen::Vector3d planned_position = trajectory->getPosition(trajectory_time);
        Eigen::Vector3d planned_velocity = trajectory->getVelocity(trajectory_time);
        
        // Update robot manager with planned state
        robot_manager.SetPoseInWorldFrame(planned_position);
        robot_manager.SetVelocityInWorldFrame(planned_velocity);
        
        // Check if we've reached a waypoint
        if (current_waypoint_idx < waypoints.size() && 
            ctrl::M_MultiWaypointPlanner::isWaypointReached(planned_position, waypoints[current_waypoint_idx])) {
            std::cout << "[KickDemo] Reached waypoint " << current_waypoint_idx 
                      << " at t=" << trajectory_time << "s" << std::endl;
            current_waypoint_idx++;
            remaining_waypoints.erase(remaining_waypoints.begin());
        }
        
        // Simulate dynamic replanning every 0.5 seconds
        if (current_time - last_update_time > 0.5 && !remaining_waypoints.empty()) {
            // Update robot state
            robot_state.position = planned_position;
            robot_state.velocity = planned_velocity;
            
            // Check if trajectory needs updating (e.g., due to drift or obstacles)
            auto updated_trajectory = ctrl::M_MultiWaypointPlanner::updateTrajectory(
                trajectory, robot_state, remaining_waypoints, constraints
            );
            
            if (updated_trajectory != trajectory) {
                std::cout << "[KickDemo] Trajectory updated at t=" << trajectory_time << "s" << std::endl;
                trajectory = updated_trajectory;
                trajectory_start_time = current_time;  // Reset time reference
            }
            
            last_update_time = current_time;
        }
        
        // Check if path is finished
        if (trajectory_time >= trajectory->getTotalTime() && !kick_executed) {
            Eigen::Vector3d robot_pos = planned_position;
            Eigen::Vector3d ball_pos = ball_position;
            double distance = (robot_pos - ball_pos).head<2>().norm();
            
            std::cout << "[KickDemo] Trajectory completed. Distance to ball: " << distance << "m" << std::endl;
            
            if (distance <= 0.35) {
                std::cout << "[KickDemo] Executing kick action" << std::endl;
                kin::ExecuteKick(soccer_objects);
                kick_executed = true;
                
                std::cout << "[KickDemo] Robot kicked the ball. Demo complete!" << std::endl;
                break;
            } else {
                // Need to get closer - create new trajectory
                std::cout << "[KickDemo] Still too far from ball. Creating approach trajectory..." << std::endl;
                
                // Update robot state
                robot_state.position = robot_pos;
                robot_state.velocity = Eigen::Vector3d::Zero();
                
                // Create approach waypoint
                Eigen::Vector3d direction = (ball_pos - robot_pos).normalized();
                Eigen::Vector3d approach_pos = ball_pos - direction * 0.15;
                approach_pos.z() = std::atan2(direction.y(), direction.x());
                
                trajectory = ctrl::M_TrajectorySegment::create(
                    constraints, robot_state.position, robot_state.velocity, approach_pos
                );
                trajectory_start_time = current_time;
            }
        }
        
        // Control and sense logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Sync robot position to soccer objects
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
        soccer_objects[0].velocity = robot_manager.GetVelocityInWorldFrame();
        
        if (!HEADLESS) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
                break;
            }
        } else {
            // In headless mode, print progress
            if (static_cast<int>(trajectory_time * 10) % 5 == 0) {  // Print every 0.5s
                std::cout << "[KickDemo] t=" << std::fixed << std::setprecision(2) << trajectory_time
                          << "s pos=(" << planned_position.x() << ", " << planned_position.y() 
                          << ") waypoint=" << current_waypoint_idx << "/" << waypoints.size() << std::endl;
            }
            util::WaitMs(20);  // 50Hz update
        }
    }
    
    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}