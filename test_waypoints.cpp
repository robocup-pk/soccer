#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"

int main() {
    std::cout << "[WaypointTest] Testing waypoint progression" << std::endl;

    // Initialize objects
    std::vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    
    // Create waypoints
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0));    // Start position (will be skipped)
    waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0));    // First waypoint
    waypoints.push_back(Eigen::Vector3d(-1.0, 0.0, 0));   // Second waypoint
    
    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BangBang);
    robot_manager.SetMPath(waypoints, util::GetCurrentTime());
    
    // Run for limited time to observe waypoint transitions
    double start_time = util::GetCurrentTime();
    double last_print_time = start_time;
    
    while (util::GetCurrentTime() - start_time < 10.0) { // Run for 10 seconds
        // Control logic
        robot_manager.ControlLogic();
        robot_manager.SenseLogic();
        
        // Update soccer objects
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
        
        // Print status every 0.5 seconds
        if (util::GetCurrentTime() - last_print_time >= 0.5) {
            Eigen::Vector3d pos = robot_manager.GetPoseInWorldFrame();
            std::cout << "[t=" << (util::GetCurrentTime() - start_time) << "s] "
                      << "Robot pos: (" << pos.x() << ", " << pos.y() << ") "
                      << "State: " << robot_manager.GetRobotState() << std::endl;
            last_print_time = util::GetCurrentTime();
        }
        
        // Small delay to simulate realistic timing
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    std::cout << "[WaypointTest] Test completed" << std::endl;
    return 0;
}