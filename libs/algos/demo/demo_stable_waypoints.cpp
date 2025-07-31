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
    std::cout << "[Demo] Running Stable Waypoints Demo" << std::endl;
    std::cout << "Testing different waypoint configurations to avoid velocity explosions" << std::endl;

    // Check for headless mode
    bool headless = false;
    if (argc > 2 && std::string(argv[2]) == "headless") {
        headless = true;
        std::cout << "Running in headless mode" << std::endl;
    }
    
    // Initialize objects
    vector<state::SoccerObject> soccer_objects;
    state::InitSoccerObjects(soccer_objects);
    vis::GLSimulation gl_simulation;
    if (!headless) {
        gl_simulation.InitGameObjects(soccer_objects);
    }
    
    // Initialize RobotManager
    rob::RobotManager robot_manager;
    
    // Set initial robot pose
    Eigen::Vector3d robot_start_pose(0.0, 0.0, 0.0);
    robot_manager.InitializePose(robot_start_pose);
    
    // Test different waypoint configurations
    int test_case = 1;
    if (argc > 1) {
        test_case = std::stoi(argv[1]);
    }
    
    vector<Eigen::Vector3d> waypoints;
    
    switch (test_case) {
        case 1: {
            // Test 1: Gradual forward movement (should work well)
            std::cout << "Test 1: Gradual forward movement" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.2, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.4, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.6, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.8, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
            break;
        }
        case 2: {
            // Test 2: Square path (more challenging)
            std::cout << "Test 2: Square path" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.5, M_PI/2));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.5, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, -M_PI/2));
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
        case 4: {
            // Test 4: RRT*-like waypoints (typical output)
            std::cout << "Test 4: RRT*-like waypoints" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.15, 0.05, 0.2));
            waypoints.push_back(Eigen::Vector3d(0.35, 0.08, 0.3));
            waypoints.push_back(Eigen::Vector3d(0.48, 0.22, 0.6));
            waypoints.push_back(Eigen::Vector3d(0.65, 0.45, 0.9));
            waypoints.push_back(Eigen::Vector3d(0.8, 0.6, 0.7));
            waypoints.push_back(Eigen::Vector3d(1.0, 0.8, 0.5));
            break;
        }
        case 5: {
            // Test 5: Sharp turns (stress test)
            std::cout << "Test 5: Sharp turns (stress test)" << std::endl;
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
            waypoints.push_back(Eigen::Vector3d(0.5, 0.0, M_PI));  // 180 degree turn
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));   // Another 180 degree turn
            break;
        }
        case 6: {
            // Test 6: Pure rotation around axis (no translation)
            std::cout << "Test 6: Pure rotation around axis (no translation)" << std::endl;
            // Robot stays at exact same position but rotates continuously in one direction
            // Full 360 degree rotation broken into segments
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));           // 0 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI/4));        // 45 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI/2));        // 90 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 3*M_PI/4));      // 135 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, M_PI));          // 180 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 5*M_PI/4));      // 225 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 3*M_PI/2));      // 270 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 7*M_PI/4));      // 315 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 2*M_PI));        // 360 degrees (full rotation)
            break;
        }
        case 7: {
            // Test 7: Pure rotation test (using original trajectory manager)
            std::cout << "Test 7: Pure rotation (using original trajectory)" << std::endl;
            // Just two waypoints: start and end orientation
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));        // Start at 0 degrees
            waypoints.push_back(Eigen::Vector3d(0.0, 0.0, 2*M_PI));     // Full 360 degree rotation
            break;
        }
    }
    
    // Print waypoints
    std::cout << "Waypoints:" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  " << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")" << std::endl;
    }

    // Set trajectory based on test case
    if (test_case == 7) {
        // Use original trajectory manager for pure rotation
        std::cout << "Using original trajectory manager for pure rotation" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::ORIGINAL);
        robot_manager.SetPath(waypoints, util::GetCurrentTime());
    } else {
        // Use B-Spline for all other cases
        std::cout << "Using B-Spline trajectory manager" << std::endl;
        robot_manager.SetTrajectoryManagerType(rob::TrajectoryManagerType::BSpline);
        robot_manager.SetBSplinePath(waypoints, util::GetCurrentTime());
    }
    
    // Variables for monitoring
    double max_velocity_seen = 0.0;
    int velocity_violation_count = 0;
    double start_time = util::GetCurrentTime();
    
    while (true) {
        // Run simulation step
        if (!headless) {
            if (!gl_simulation.RunSimulationStep(soccer_objects, util::CalculateDt())) {
                std::cout << "[Demo] Simulation finished" << std::endl;
                break;
            }
            
            // Process input and update robot state
            vis::ProcessInput(gl_simulation.GetRawGLFW(), soccer_objects);
        } else {
            // In headless mode, just update time
            util::WaitMs(20);  // 50Hz update
        }
        
        // Control logic for RobotManager
        robot_manager.ControlLogic();
        
        // Sense logic for RobotManager
        robot_manager.SenseLogic();
        
        // Get current velocity and check for violations
        Eigen::Vector3d velocity = robot_manager.GetVelocityInWorldFrame();
        double linear_speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
        
        if (linear_speed > max_velocity_seen) {
            max_velocity_seen = linear_speed;
        }
        
        if (linear_speed > 1.0 || std::abs(velocity[2]) > 5.0) {
            velocity_violation_count++;
            std::cout << "[WARNING] Velocity limit exceeded! Linear: " << linear_speed 
                      << " m/s, Angular: " << velocity[2] << " rad/s" << std::endl;
        }
        
        // Update soccer objects with current robot pose
        soccer_objects[0].position = robot_manager.GetPoseInWorldFrame();
        soccer_objects[0].velocity = velocity;
        
        // Print status every second
        static double last_print_time = 0;
        double current_time = util::GetCurrentTime();
        if (current_time - last_print_time > 1.0) {
            Eigen::Vector3d pose = robot_manager.GetPoseInWorldFrame();
            std::cout << "[Status] Time: " << (current_time - start_time) 
                      << "s, Pose: (" << pose[0] << ", " << pose[1] << ", " << pose[2] 
                      << " rad = " << pose[2] * 180.0 / M_PI << " deg"
                      << "), Linear vel: " << linear_speed << " m/s, Angular vel: " 
                      << velocity[2] << " rad/s" << std::endl;
            last_print_time = current_time;
        }
        
        // Stop after 30 seconds or when trajectory is complete
        if (current_time - start_time > 30.0) {
            std::cout << "[Demo] Time limit reached" << std::endl;
            break;
        }
        
        // In test case 6, stop when rotation is complete
        if (test_case == 6 && robot_manager.GetRobotState() == "IDLE" && current_time - start_time > 3.0) {
            std::cout << "[Demo] Rotation completed" << std::endl;
            break;
        }
    }
    
    // Print summary
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Max velocity seen: " << max_velocity_seen << " m/s" << std::endl;
    std::cout << "Velocity violations: " << velocity_violation_count << std::endl;
    std::cout << "Test " << (velocity_violation_count == 0 ? "PASSED" : "FAILED") << std::endl;

    return 0;
}