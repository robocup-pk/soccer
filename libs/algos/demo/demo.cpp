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
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    int N = 36;

    vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i <= N; ++i) {
        double θ = (2*M_PI*i)/N;
        waypoints.push_back(Eigen::Vector3d(cos(θ), sin(θ), 0.0));
    }

    robot_manager.SetPath(waypoints, util::GetCurrentTime());
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