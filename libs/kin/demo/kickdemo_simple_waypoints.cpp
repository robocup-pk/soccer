#define _USE_MATH_DEFINES
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>

#include "GLSimulation.h"
#include "SoccerObject.h" 
#include "RobotManager.h"
#include "Kinematics.h"
#include "Kick.h"
#include "Utils.h"

// Run demo without graphical output when true
static const bool HEADLESS = false;

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[KickDemo] Simple Direct Waypoint Demo - Testing basic trajectory following" << std::endl;
    
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
    
    // Create SIMPLE waypoints - only 3 points to test basic functionality
    std::vector<Eigen::Vector3d> simple_waypoints;
    
    // Start at robot position
    simple_waypoints.push_back(robot_start_pose);
    
    // One intermediate waypoint 
    double angle_to_intermediate = std::atan2(-0.2, -0.7);  // Face towards intermediate point
    simple_waypoints.push_back(Eigen::Vector3d(-0.7, -0.2, angle_to_intermediate));
    
    // Final waypoint close to ball
    double angle_to_ball = std::atan2(ball_position.y() - (-0.2), ball_position.x() - (-0.7));
    simple_waypoints.push_back(Eigen::Vector3d(-1.4, -0.45, angle_to_ball));
    
    std::cout << "[KickDemo] Created " << simple_waypoints.size() << " simple waypoints:" << std::endl;
    for(int i = 0; i < simple_waypoints.size(); ++i){
        cout << "  Waypoint " << i << ": (" 
             << simple_waypoints[i].x() << ", " 
             << simple_waypoints[i].y() << ", " 
             << simple_waypoints[i].z() << " rad = " 
             << (simple_waypoints[i].z() * 180.0 / M_PI) << " deg)" << std::endl;
    }
    
    // Set trajectory manager to BangBang mode
    std::cout << "[KickDemo] Using BangBang-based M_TrajectoryManager" << std::endl;
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
    robot_manager.SetMPath(simple_waypoints);
    
    bool kick_executed = false;
    double last_print_time = util::GetCurrentTime();
    double demo_start_time = util::GetCurrentTime();
    int waypoint_reached_count = 0;

    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        double elapsed_time = current_time - demo_start_time;
        
        // Exit after 30 seconds to prevent infinite loop
        if (elapsed_time > 30.0) {
            std::cout << "[KickDemo] Demo timeout after 30 seconds. Exiting." << std::endl;
            break;
        }
        
        if (!HEADLESS)
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);

        // Control and sense logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();

        // Get current robot pose
        Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
        Eigen::Vector3d robot_vel = robot_manager.GetVelocityInWorldFrame();
        std::string robot_state = robot_manager.GetRobotState();
        
        // Print status every 1 second
        if (current_time - last_print_time > 1.0) {
            std::cout << "[t=" << std::fixed << std::setprecision(1) << elapsed_time 
                      << "] Robot pos: (" << std::setprecision(3) << robot_pos.x() << ", " << robot_pos.y() 
                      << "), angle: " << std::setprecision(1) << (robot_pos.z() * 180.0 / M_PI) << " deg"
                      << ", vel: " << std::setprecision(2) << robot_vel.head<2>().norm() << " m/s"
                      << ", state: " << robot_state << std::endl;
            
            // Check which waypoint we're closest to
            double min_dist = std::numeric_limits<double>::max();
            int closest_waypoint = -1;
            for (int i = 0; i < simple_waypoints.size(); ++i) {
                double dist = (robot_pos.head<2>() - simple_waypoints[i].head<2>()).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_waypoint = i;
                }
            }
            std::cout << "  Closest to waypoint " << closest_waypoint 
                      << " (dist: " << std::setprecision(3) << min_dist << "m)" << std::endl;
            
            // Check if we've reached the current target waypoint
            if (min_dist < 0.1 && closest_waypoint > waypoint_reached_count) {
                std::cout << "  >>> REACHED waypoint " << closest_waypoint << "!" << std::endl;
                waypoint_reached_count = closest_waypoint;
            }
            
            last_print_time = current_time;
        }

        // Check if path is finished
        if (robot_state == "IDLE" && !kick_executed) {
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