#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "Waypoint.h"
#include "GLSimulation.h"
#include "SoccerObject.h"
#include "RobotManager.h"
#include "Utils.h"
#include "RRTX.h"
#include "Kick.h"

using namespace std;

int main(int argc, char* argv[]) {
    std::cout << "[Demo] Running RobotManager demo with trajectory logging" << std::endl;

    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    gl_simulation.InitGameObjects(soccer_objects);
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    ctrl::BSplineTrajectoryManager bspline_manager;
    bspline_manager.SetFeedbackGains(0.5, 0.1);
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    vector<Eigen::Vector3d> waypoints;
    robot_manager.GetUniformBSplinePlanner().SetLimits(0.8, 0.5, 0.8, 0.5);
    robot_manager.GetUniformBSplinePlanner().SetFeedbackGains(0.1, 0.05);
    
    // Choose a test case based on command line argument
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    // Open log file for trajectory data
    std::ofstream trajectory_log("trajectory_log.txt");
    trajectory_log << "# Trajectory Log File" << std::endl;
    trajectory_log << "# Format: timestamp(s) x(m) y(m) theta(rad) vx(m/s) vy(m/s) omega(rad/s)" << std::endl;
    
    // Log waypoints
    trajectory_log << "# WAYPOINTS" << std::endl;
    
    switch (test_case) {
        case 1: {
            // Test 1: Simple square path
            std::cout << "Test 1: Simple square path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0, 0, 0));
            waypoints.push_back(Eigen::Vector3d(-1, 0, 0));
            waypoints.push_back(Eigen::Vector3d(-1, 1, 0));
            waypoints.push_back(Eigen::Vector3d(0, 1, 0));
            waypoints.push_back(Eigen::Vector3d(0, 0, 0));
            break;
        }
        case 2: {
            // Test 2: Circular path centered at origin
            std::cout << "Test 2: Circular path" << std::endl;
            int N = 24;  // Fewer points for smoother circle
            double radius = 0.5;
            
            // Start from current robot position (0, 0) and move to circle smoothly
            // Add transition waypoints from origin to circle
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));  // Start at origin
            waypoints.push_back(Eigen::Vector3d(radius * 0.5, 0.0, 0.0));  // Halfway to circle
            
            // Generate circle waypoints starting from right side (radius, 0)
            for (int i = 0; i <= N; ++i) {
                double angle = 2.0 * M_PI * i / N;
                double x = radius * std::cos(angle);
                double y = radius * std::sin(angle);
                // Use atan2 for proper heading that follows the tangent
                double heading = angle + M_PI/2;  // Tangent to circle
                waypoints.push_back(Eigen::Vector3d(x, y, heading));
            }
            
            // Add transition back to origin if desired
            waypoints.push_back(Eigen::Vector3d(radius * 0.5, 0.0, M_PI));  // Halfway back
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));  // Back to origin
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
            // Default: Straight line
            std::cout << "Test 4: Straight line" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
        }
    }
    
    // Log waypoints to file
    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_log << "# WP " << i << " " << waypoints[i][0] << " " 
                      << waypoints[i][1] << " " << waypoints[i][2] << std::endl;
    }
    
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }
    
    // Choose trajectory type based on second argument
    int traj_type = 2; // Default to Uniform B-spline
    if (argc > 2) {
        traj_type = std::atoi(argv[2]);
    }
    
    trajectory_log << "# TRAJECTORY_TYPE " << traj_type << std::endl;
    
    switch (traj_type) {
        case 1:
            std::cout << "Using B-spline trajectory" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
        case 2:
            std::cout << "Using Uniform B-spline trajectory (EWOK-based)" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::UniformBSpline);
            robot_manager.SetUniformBSplinePath(waypoints, util::GetCurrentTime());
            break;
        case 3:
            std::cout << "Using Bezier trajectory" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BezierTrajectory);
            robot_manager.GetBezierTrajectoryPlanner().SetLimits(0.8, 0.5, 2.5, 3.0);
            robot_manager.GetBezierTrajectoryPlanner().SetFeedbackGains(0.05, 0.3);
            if (test_case == 1) {
                robot_manager.GetBezierTrajectoryPlanner().SetCornerCutDistance(0.05);
            }
            robot_manager.SetBezierTrajectoryPath(waypoints, util::GetCurrentTime());
            break;
        default:
            std::cout << "Using B-spline trajectory (default)" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
    }
    
    trajectory_log << "# DATA_START" << std::endl;
    
    // Timing
    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    const int max_frames = 1000; // Limit to prevent infinite logging
    
    // For DB-RRT tracking
    int current_waypoint_idx = 1;
    bool db_rrt_finished = true;
    
    while (frame_count < max_frames) {
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
        
        // Get current robot state
        Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();
        Eigen::Vector3d current_velocity = robot_manager.GetVelocityInWorldFrame();
        
        // Calculate timestamp
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        double timestamp = elapsed.count();
        
        // Log trajectory data
        trajectory_log << timestamp << " " 
                      << current_pose[0] << " " << current_pose[1] << " " << current_pose[2] << " "
                      << current_velocity[0] << " " << current_velocity[1] << " " << current_velocity[2] 
                      << std::endl;
        
        // Update soccer objects with current robot pose
        soccer_objects[0].position = current_pose;
        
        frame_count++;
        
        // Check if we should exit (press ESC in the window)
        if (glfwGetKey(gl_simulation.GetRawGLFW(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            break;
        }
    }
    
    trajectory_log.close();
    std::cout << "[Demo] Trajectory data saved to trajectory_log.txt" << std::endl;
    std::cout << "[Demo] Recorded " << frame_count << " frames" << std::endl;
    
    return 0;
}