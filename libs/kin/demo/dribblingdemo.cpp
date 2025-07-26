// Demo application to test RRT path planning with dribbling functionality
#define _USE_MATH_DEFINES
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdlib>
#include <vector>

// Project libs
#include "GLSimulation.h" 
#include "SoccerObject.h" 
#include "RRTX.h"
#include "Algos.h"
#include "Waypoint.h"
#include "Kinematics.h"
#include "Dribble.h"
#include "Coordinates.h"
#include "Utils.h"
#include "RobotManager.h"
#include "TrapezoidalTrajectoryVi3D.h"

using namespace std;

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
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0);    // Robot starts at bottom-left
    Eigen::Vector3d ball_position(1.5, 0.5, 0);         // Ball at center
    //Eigen::Vector3d target_position(2.0, 1.5, 0);       // Target at top-right
    
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
    state::Path path;
    path = algo::FindSinglePath_RRTX(state::Waypoint(robot_start_pose[0], robot_start_pose[1], 0.0),
                                                state::Waypoint(ball_position[0], ball_position[1], 0.0));

    //Now use TrajectoryManager to generate a path
    vector<Eigen::Vector3d> targetPath;
    for (int i = 0; i < path.size(); ++i) {
        targetPath.push_back(Eigen::Vector3d(path[i].x * 1000, path[i].y * 1000, path[i].angle));
    }
    // Add ball to robot manager's goal queue
    robot_manager.SetPath(targetPath);

    // Main simulation loop
    while (true) {
        double current_time = util::GetCurrentTime();
        double dt = util::CalculateDt();
        
        // Handle input
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        //calculate distance between robot and ball
        Eigen::Vector3d robot_pos = robot_manager.GetPoseInWorldFrame();
        double dist = (ball_position - robot_pos).head<2>().norm();
        if (dist < 0.3) {  // If close enough to ball, start dribbling
            std::cout << "[DribblingDemo] Robot close to ball, starting dribble..." << std::endl;
            kin::ExecuteDribble(soccer_objects);  // Execute dribble action directly
        } else {
            std::cout << "[DribblingDemo] Robot moving towards ball..." << std::endl;
            //robot_manager.MoveTowards(ball_position);  // Move towards the ball if not close enough
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