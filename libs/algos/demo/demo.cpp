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
    robot_manager.GetUniformBSplinePlanner().SetLimits(0.8, 0.5, 0.8, 0.5); // v_max, a_max, omega_max, alpha_max
    robot_manager.GetUniformBSplinePlanner().SetFeedbackGains(0.02, 0.0); // kp, kd
    // Choose a test case based on command line argument
    int test_case = 1;
    if (argc > 1) {
        test_case = std::atoi(argv[1]);
    }
    
    switch (test_case) {
        case 1: {
            // Test 1: Straight line trajectory
            std::cout << "Test 1: Straight line trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, 0.0));
            // waypoints.push_back(Eigen::Vector3d(1.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
        case 2: {
            // Test 2: L-shaped path (90-degree turn)
            std::cout << "Test 2: L-shaped path (90-degree turn)" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            break;
        }
        case 3: {
            // Test 3: Square path
            std::cout << "Test 3: Square path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 1.0, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 1.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
            break;
        }
        case 4: {
            // Test 4: Circular path
            std::cout << "Test 4: Circular path" << std::endl;
            int N = 16;
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
        case 5: {
            // Test 5: S-curve trajectory
            std::cout << "Test 5: S-curve trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.2, M_PI/6));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.5, -0.2, -M_PI/6));
            waypoints.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
            break;
        }
        case 6: {
            // Test 6: Sharp zigzag (stress test for corners)
            std::cout << "Test 6: Sharp zigzag trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.3, 0.3, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.6, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.9, 0.3, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.2, 0.0, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(1.5, 0.3, M_PI/4));
            break;
        }
        case 7: {
            // Test 7: Lane change maneuver
            std::cout << "Test 7: Lane change maneuver" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.3, M_PI/8));
            waypoints.push_back(Eigen::Vector3d(1.5, 0.5, 0.0));
            waypoints.push_back(Eigen::Vector3d(2.0, 0.5, 0.0));
            waypoints.push_back(Eigen::Vector3d(2.5, 0.5, 0.0));
            break;
        }
        case 8: {
            // Test 8: Figure-8 pattern
            std::cout << "Test 8: Figure-8 pattern" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.25, 0.25, M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(-0.25, 0.25, 3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.25, -0.25, -3*M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, -0.5, -M_PI/2));
            waypoints.push_back(Eigen::Vector3d(-0.25, -0.25, -M_PI/4));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
        case 9: {
            // Test 9: Star pattern (multiple sharp turns)
            std::cout << "Test 9: Star pattern" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            for (int i = 0; i < 5; ++i) {
                double angle = 2.0 * M_PI * i / 5;
                waypoints.push_back(Eigen::Vector3d(
                    0.5 * std::cos(angle),
                    0.5 * std::sin(angle),
                    angle
                ));
                // Inner point
                double inner_angle = angle + 2.0 * M_PI / 10;
                waypoints.push_back(Eigen::Vector3d(
                    0.2 * std::cos(inner_angle),
                    0.2 * std::sin(inner_angle),
                    inner_angle
                ));
            }
            break;
        }
        case 10: {
            // Test 10: Spiral trajectory (increasing radius)
            std::cout << "Test 10: Spiral trajectory" << std::endl;
            int N = 20;
            for (int i = 0; i <= N; ++i) {
                double angle = 3.0 * M_PI * i / N;  // 1.5 full rotations
                double radius = 0.1 + 0.4 * i / N;  // Radius from 0.1 to 0.5
                waypoints.push_back(Eigen::Vector3d(
                    radius * std::cos(angle),
                    radius * std::sin(angle),
                    angle
                ));
            }
            break;
        }
        default: {
            // Default: Simple forward and back
            std::cout << "Default: Forward and back trajectory" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            break;
        }
    }
    
    // Print waypoints
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }
    
    std::cout << "\nTrajectory type options:" << std::endl;
    std::cout << "  1: B-spline (traditional)" << std::endl;
    std::cout << "  2: Uniform B-spline (EWOK-based)" << std::endl;
    std::cout << "  3: Bezier trajectory (RoboJackets-style)" << std::endl;
    std::cout << "  4: DB-RRT (Dynamically feasible B-spline based RRT)" << std::endl;

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
        case 3:
            std::cout << "Using Bezier trajectory (RoboJackets-style) for accurate waypoint following" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BezierTrajectory);
            
            // Configure planner
            robot_manager.GetBezierTrajectoryPlanner().SetLimits(0.8, 0.5, 2.5, 3.0);
            robot_manager.GetBezierTrajectoryPlanner().SetFeedbackGains(0.05, 0.3);
            
            // For square paths, use smaller corner cut distance
            if (test_case == 1) {
                robot_manager.GetBezierTrajectoryPlanner().SetCornerCutDistance(0.05);
            }
            
            robot_manager.SetBezierTrajectoryPath(waypoints, util::GetCurrentTime());
            break;
        case 4:
            std::cout << "Using DB-RRT trajectory planner for dynamic path planning" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::DBRRT);
            
            // Configure DB-RRT planner
            robot_manager.GetDBRRTPlanner().SetDynamicLimits(1.0, 0.8, 3.0, 4.0);
            robot_manager.GetDBRRTPlanner().SetBSplineParameters(3, 0.1);
            robot_manager.GetDBRRTPlanner().SetControlGains(2.0, 0.5, 1.5, 0.5);
            
            // For DB-RRT, we plan to each waypoint sequentially
            if (waypoints.size() > 1) {
                // Plan to first waypoint that's different from start
                Eigen::Vector3d goal = waypoints[2];
                robot_manager.SetDBRRTGoal(goal);
            }
            break;
        default:
            std::cout << "Using B-spline trajectory (default)" << std::endl;
            robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
            robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
            break;
    }
    // Add goals to the queue
    
    // For DB-RRT, track current waypoint
    int current_waypoint_idx = 1;
    bool db_rrt_finished = true;
    
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
        
        // For DB-RRT, check if we need to replan to next waypoint
        if (traj_type == 4 && waypoints.size() > 1 && !db_rrt_finished) {
            Eigen::Vector3d current_pose = robot_manager.GetPoseInWorldFrame();
            Eigen::Vector3d current_goal = waypoints[current_waypoint_idx];
            double dist_to_goal = (current_pose.head<2>() - current_goal.head<2>()).norm();
            
            // If close to current goal, plan to next waypoint
            if (dist_to_goal < 0.05 && robot_manager.GetRobotState() == "IDLE") {
                current_waypoint_idx++;
                
                // Check if we've reached all waypoints
                if (current_waypoint_idx >= waypoints.size()) {
                    // For cyclic paths (like square), go back to waypoint 1
                    if (waypoints.front().isApprox(waypoints.back(), 0.1)) {
                        current_waypoint_idx = 1;
                    } else {
                        db_rrt_finished = true;
                        std::cout << "[Demo] DB-RRT: Reached final waypoint!" << std::endl;
                        continue;
                    }
                }
                
                std::cout << "[Demo] Reached waypoint " << (current_waypoint_idx-1) 
                          << ", planning to waypoint " << current_waypoint_idx 
                          << ": " << waypoints[current_waypoint_idx].transpose() << std::endl;
                robot_manager.SetDBRRTGoal(waypoints[current_waypoint_idx]);
            }
        }
    }

    return 0;
}