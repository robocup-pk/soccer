#define _USE_MATH_DEFINES
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "Waypoint.h"
#include "RobotManager.h"
#include "Kinematics.h"
#include "Kick.h"
#include "Utils.h"

// Run demo without graphical output when true
static const bool HEADLESS = false;

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Perfect Waypoints Demo - Testing trajectory following" << std::endl;
    
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
    
    // Set positions - same as original
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);     // Robot starts at origin
    Eigen::Vector3d ball_position(-1.5, -0.5, 0.0);      // Ball at (-1.5, -0.5)
    
    // Initialize robot pose
    robot_manager.InitializePose(robot_start_pose);
    
    for (auto& obj : soccer_objects) {
        if (obj.name == "robot0") {
            obj.position = robot_start_pose;
            obj.velocity = Eigen::Vector3d::Zero();
        } else if (obj.name == "ball") {
            obj.position = ball_position;
            obj.velocity = Eigen::Vector3d::Zero();
        }
    }
    
    // Create PERFECT waypoints from robot to ball
    // This creates a smooth path with gradual turns
    std::vector<Eigen::Vector3d> perfect_waypoints;
    
    // Start position
    perfect_waypoints.push_back(robot_start_pose);
    
    // Create a smooth arc path to the ball
    // We'll create waypoints that gradually move towards the ball with smooth angle transitions
    
    // Straight section first (moving in -X direction)
    perfect_waypoints.push_back(Eigen::Vector3d(-0.1, 0.0, -M_PI));        // Face left
    perfect_waypoints.push_back(Eigen::Vector3d(-0.2, 0.0, -M_PI));        // Continue left
    perfect_waypoints.push_back(Eigen::Vector3d(-0.3, 0.0, -M_PI));        // Continue left
    perfect_waypoints.push_back(Eigen::Vector3d(-0.4, 0.0, -M_PI));        // Continue left
    perfect_waypoints.push_back(Eigen::Vector3d(-0.5, 0.0, -M_PI));        // Continue left
    
    // Start gradual turn downward (towards -Y)
    perfect_waypoints.push_back(Eigen::Vector3d(-0.6, -0.05, -M_PI + 0.1));   // Slight turn
    perfect_waypoints.push_back(Eigen::Vector3d(-0.7, -0.1, -M_PI + 0.2));    // More turn
    perfect_waypoints.push_back(Eigen::Vector3d(-0.8, -0.15, -M_PI + 0.3));   // More turn
    perfect_waypoints.push_back(Eigen::Vector3d(-0.9, -0.2, -M_PI + 0.4));    // More turn
    perfect_waypoints.push_back(Eigen::Vector3d(-1.0, -0.25, -M_PI + 0.5));   // More turn
    
    // Continue towards ball
    perfect_waypoints.push_back(Eigen::Vector3d(-1.1, -0.3, -M_PI + 0.6));    
    perfect_waypoints.push_back(Eigen::Vector3d(-1.2, -0.35, -M_PI + 0.7));   
    perfect_waypoints.push_back(Eigen::Vector3d(-1.3, -0.4, -M_PI + 0.8));    
    
    // Final approach - face the ball directly
    double final_angle = std::atan2(ball_position.y() - (-0.4), ball_position.x() - (-1.3));
    perfect_waypoints.push_back(Eigen::Vector3d(-1.4, -0.45, final_angle));
    
    // Very close to ball - maintain facing angle
    perfect_waypoints.push_back(Eigen::Vector3d(-1.45, -0.475, final_angle));
    
    std::cout << "[KickDemo] Created " << perfect_waypoints.size() << " perfect waypoints:" << std::endl;
    for(int i = 0; i < perfect_waypoints.size(); ++i){
        cout << "  Waypoint " << i << ": (" 
             << perfect_waypoints[i].x() << ", " 
             << perfect_waypoints[i].y() << ", " 
             << perfect_waypoints[i].z() << " rad = " 
             << (perfect_waypoints[i].z() * 180.0 / M_PI) << " deg)" << std::endl;
    }
    
    // Calculate total path length
    double total_distance = 0.0;
    for (size_t i = 1; i < perfect_waypoints.size(); ++i) {
        double segment_dist = (perfect_waypoints[i] - perfect_waypoints[i-1]).head<2>().norm();
        total_distance += segment_dist;
        std::cout << "  Segment " << (i-1) << "->" << i << ": " << segment_dist << "m" << std::endl;
    }
    std::cout << "[KickDemo] Total path distance: " << total_distance << "m" << std::endl;
    
    // Set trajectory manager to BangBang mode
    std::cout << "[KickDemo] Using BangBang-based M_TrajectoryManager" << std::endl;
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
    robot_manager.SetMPath(perfect_waypoints);
    
    bool kick_executed = false;
    double last_print_time = util::GetCurrentTime();
    int waypoint_reached_count = 0;
    Eigen::Vector3d last_robot_pos = robot_start_pose;

    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        if (!HEADLESS)
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);

        // Control and sense logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();

        // Get current robot pose
        Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
        Eigen::Vector3d robot_vel = robot_manager.GetVelocityInWorldFrame();
        
        // Print status every 0.5 seconds
        if (current_time - last_print_time > 0.5) {
            std::cout << "[t=" << std::fixed << std::setprecision(2) << current_time 
                      << "] Robot pos: (" << robot_pos.x() << ", " << robot_pos.y() 
                      << "), angle: " << (robot_pos.z() * 180.0 / M_PI) << " deg"
                      << ", vel: " << robot_vel.head<2>().norm() << " m/s"
                      << ", state: " << robot_manager.GetRobotState() << std::endl;
            
            // Check which waypoint we're closest to
            double min_dist = std::numeric_limits<double>::max();
            int closest_waypoint = -1;
            for (int i = 0; i < perfect_waypoints.size(); ++i) {
                double dist = (robot_pos.head<2>() - perfect_waypoints[i].head<2>()).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_waypoint = i;
                }
            }
            std::cout << "  Closest to waypoint " << closest_waypoint 
                      << " (dist: " << min_dist << "m)" << std::endl;
            
            last_print_time = current_time;
        }

        // Check if path is finished
        if (robot_manager.GetRobotState() == "IDLE" && !kick_executed) {
            double distance = (robot_pos - ball_position).head<2>().norm();
            
            std::cout << "[KickDemo] Path completed. Distance to ball: " << distance << "m" << std::endl;
            
            if (distance <= 0.35) {
                std::cout << "[KickDemo] Executing kick action" << std::endl;
                kin::ExecuteKick(soccer_objects);
                kick_executed = true;
                std::cout << "[KickDemo] Robot kicked the ball. Demo complete!" << std::endl;
                break;
            } else {
                // Create final approach path
                std::cout << "[KickDemo] Still too far from ball. Creating final approach..." << std::endl;
                vector<Eigen::Vector3d> approach_path;
                approach_path.push_back(robot_pos);
                
                // Direct path to kick position
                Eigen::Vector3d direction = (ball_position - robot_pos).normalized();
                Eigen::Vector3d kick_position = ball_position - direction * 0.15;
                kick_position.z() = std::atan2(direction.y(), direction.x());
                approach_path.push_back(kick_position);
                
                robot_manager.SetMPath(approach_path);
            }
        }
        
        // Sync robot position to soccer objects
        soccer_objects[0].position = robot_pos;
        soccer_objects[0].velocity = robot_vel;
        
        if (!HEADLESS) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, dt)) {
                break;
            }
        } else {
            util::WaitMs(20);
        }
    }

    std::cout << "[KickDemo] Demo finished!" << std::endl;
    return 0;
}