#include <iostream>
#include <vector>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"
#include "Kick.h"
//#include "BsplineManager.h"
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
    ctrl::BSplineTrajectoryManager bspline_manager;
    bspline_manager.SetFeedbackGains(0.5, 0.1); // Set feedback gains for smoother control
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0); // Robot starts at origin facing up
    robot_manager.InitializePose(robot_start_pose);
    vector<Eigen::Vector3d> waypoints;
    
    // Choose a test case based on command line argument
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    switch (test_case) {
        case 1: {
            // Test 1: Simple forward movement with reasonable spacing
            std::cout << "Test 1: Simple forward movement" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0, 0, 0));
            waypoints.push_back(Eigen::Vector3d(-1, 0, 0));
            waypoints.push_back(Eigen::Vector3d(-1, 1, 0));
            waypoints.push_back(Eigen::Vector3d(0, 1, 0));
            waypoints.push_back(Eigen::Vector3d(0, 0, 0));
            break;
        }
        case 2: {
            // Test 2: Circular path
            std::cout << "Test 2: Circular path" << std::endl;
            int N = 8;
            double radius = 0.5;
            for (int i = 0; i <= N; ++i) {
                double angle = 2.0 * M_PI * i / N;
                waypoints.push_back(Eigen::Vector3d(
                    radius * std::cos(angle),
                    radius * std::sin(angle),
                    angle
                ));
            }
            break;
        }
        case 3: {
            // Test 3: Figure-8 pattern
            std::cout << "Test 3: Figure-8 pattern" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.3, 0.2, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.3, 0.8, 3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(-0.3, 0.8, -3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(-0.5, 0.5, -M_PI/2));
            waypoints.push_back(Eigen::Vector3d(-0.3, 0.2, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
        default: {
            // Default: The problematic sequence for testing
            std::cout << "Test 4: Problematic sequence (for debugging)" << std::endl;
            int N = 10;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            for (int i = 1; i <= N; ++i) {
                waypoints.push_back(Eigen::Vector3d(i/N, 0.0, 0.0));
            }
            break;
        }
    }
    
    // Print waypoints
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }

    // Choose trajectory type based on second argument
    int traj_type = 1;
    if (argc > 2) {
        traj_type = std::atoi(argv[2]);
    }
    
    switch (traj_type) {
        case 1:
            std::cout << "Using B-spline trajectory for smoother motion" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
        case 2:
            std::cout << "Using Uniform B-spline trajectory (EWOK-based) for robust motion" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::UniformBSpline);
            robot_manager.SetUniformBSplinePath(waypoints, util::GetCurrentTime());
            break;
        default:
            std::cout << "Using B-spline trajectory (default)" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
    }
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