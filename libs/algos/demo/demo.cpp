#include <iostream>
#include <vector>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"
#include "Kick.h"
using namespace std;
int main(int argc, char* argv[]) {
    std::cout << "[Demo] Running RobotManager demo" << std::endl;

    // Initialize objects
   vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0); // Robot starts at origin facing up
    robot_manager.InitializePose(robot_start_pose);
    int N = 20; // Number of waypoints
    double theta_final = M_PI / 2.0;
    vector<Eigen::Vector3d> waypoints;
    //waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI_2/2.0)); // Generate waypoints in a circle
    waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0)); // Start at origin
    for (int i = 1; i <= N; ++i) {
        waypoints.push_back(Eigen::Vector3d(N/i, 0.0, 0.0)); // Generate waypoints in a circle
    }
    // for(int i = 1; i <= N; ++i) {
    //     waypoints.push_back(Eigen::Vector3d(-N/i, 0.0, 0.0)); // Add waypoints with increasing angles
    // }

    //waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0));
    
    // waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0));
    // waypoints.push_back(Eigen::Vector3d(-1.0, 0.0, 0));

    robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::HermiteSpline);
    robot_manager.SetHermiteSplinePath(waypoints, util::GetCurrentTime());
    // Add goals to the queue
    
    while (true) {
        // Run simulation step
        if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
            std::cout << "[Demo] Simulation finished" << std::endl;
            break;
        }
        
        // Process input and update robot state
        vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        
        // Control logic for RobotManager
        robot_manager.ControlLogic();
        
        // Sense logic for RobotManager
        robot_manager.SenseLogic();

        
        // Update soccer objects with current robot pose
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
    }

    return 0;
}